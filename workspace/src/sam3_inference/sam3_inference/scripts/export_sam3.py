"""Exports SAM 3.1's image detector to three ONNX graphs: image encoder, text encoder, decoder.

Run with the Python of a venv that has torch and Meta's ``sam3`` package (not in the robot's image):

    python export_sam3.py --checkpoint sam3.1_multiplex.pt --out <dir> --bf16 --suffix .bf16

which writes the ``sam3_*.bf16.onnx`` graphs that sam3_inference's default ``precision_suffix``
loads. Without ``--bf16`` the graphs are fp32 (and unsuffixed unless ``--suffix`` is given).
With ``--image test.png --prompt cup`` it checks each fp32 graph against PyTorch with ONNX Runtime on
the CPU; under ``--bf16`` the check is skipped (bf16 and the CPU reference differ by more than the
check's sense).
"""

from __future__ import annotations

import argparse
import gzip
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
import torch
from sam3.model import vitdet
from sam3.model.data_misc import FindStage
from sam3.model_builder import build_sam3_image_model
from torch import nn

INPUT_SIZE_PX = 1008
CONTEXT_LENGTH = 32
# Best queries the decoder hands out (of 200); masks of the rest never leave the GPU.
TOP_K = 16
PARTS = ("image", "text", "decoder")


def patch_for_export(model: nn.Module) -> None:
    """Replaces the ops ONNX cannot express: complex RoPE, the bf16 fused MLP, flash attention.

    Args:
        model: The SAM 3 image model, patched in place.
    """

    def mlp_forward(self: Any, x: torch.Tensor) -> torch.Tensor:
        return self.drop2(self.fc2(self.norm(self.drop1(self.act(self.fc1(x))))))

    vitdet.Mlp.forward = mlp_forward
    for module in model.backbone.vision_backbone.modules():
        if isinstance(module, nn.ConvTranspose2d):
            # TensorRT has no bf16 transposed convolution; the neck's upsampling stays in fp32.
            def deconv_forward(x: torch.Tensor, conv: nn.ConvTranspose2d = module) -> torch.Tensor:
                with torch.autocast("cuda", enabled=False):
                    return nn.ConvTranspose2d.forward(conv, x.float())

            module.forward = deconv_forward
    geometry = model.geometry_encoder

    def geometry_text_only(
        geo_prompt: Any,
        img_feats: list[torch.Tensor],
        img_sizes: Any,
        img_pos_embeds: list[torch.Tensor] | None = None,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        # Without boxes or points the encoder output is its CLS token after cross-attention to the
        # image; the empty-tensor branches crash the tracer, so take this path directly.
        memory = img_feats[-1]
        pos = img_pos_embeds[-1] if img_pos_embeds is not None else torch.zeros_like(memory)
        batch = memory.shape[1]
        embeds = geometry.cls_embed.weight.view(1, 1, geometry.d_model).repeat(1, batch, 1)
        mask = torch.zeros(batch, 1, dtype=torch.bool, device=memory.device)
        if geometry.final_proj is not None:
            embeds = geometry.norm(geometry.final_proj(embeds))
        if geometry.encode is not None:
            for layer in geometry.encode:
                embeds = layer(tgt=embeds, memory=memory, tgt_key_padding_mask=mask, pos=pos)
            embeds = geometry.encode_norm(embeds)
        return embeds, mask

    geometry.forward = geometry_text_only

    def encode_prompt(
        backbone_out: dict[str, Any], find_input: Any, geometric_prompt: Any, **kwargs: Any
    ) -> tuple[torch.Tensor, torch.Tensor, dict[str, Any]]:
        txt_feats = backbone_out["language_features"][:, find_input.text_ids]
        txt_masks = backbone_out["language_mask"][find_input.text_ids]
        backbone_out, img_feats, img_pos_embeds, vis_feat_sizes = model._get_img_feats(
            backbone_out, find_input.img_ids
        )
        geo_feats, geo_masks = geometry_text_only(None, img_feats, vis_feat_sizes, img_pos_embeds)
        return (
            torch.cat([txt_feats, geo_feats], dim=0),
            torch.cat([txt_masks, geo_masks], dim=1),
            backbone_out,
        )

    model._encode_prompt = encode_prompt
    for module in model.modules():
        if hasattr(module, "use_fa3"):
            module.use_fa3 = False
        if isinstance(module, vitdet.Attention) and module.use_rope and not module.use_ve_rope:
            if not module.use_rope_real:
                module.register_buffer("freqs_cis_real", module.freqs_cis.real.contiguous())
                module.register_buffer("freqs_cis_imag", module.freqs_cis.imag.contiguous())
                module.use_rope_real = True


class ImageEncoder(nn.Module):
    """Normalised image [1, 3, 1008, 1008] -> three FPN levels and the coarsest position encoding.

    Args:
        model: The patched SAM 3 image model.
    """

    def __init__(self, model: nn.Module) -> None:
        super().__init__()
        self.backbone = model.backbone

    def forward(self, image: torch.Tensor) -> tuple[torch.Tensor, ...]:
        """Return the FPN levels fpn0 to fpn2 and the position encoding pos2."""
        out = self.backbone.forward_image(image)
        # The decoder reads the three FPN levels and only the coarsest position encoding.
        return (*out["backbone_fpn"], out["vision_pos_enc"][2])


class TextEncoder(nn.Module):
    """Token ids [B, 32] -> language mask [B, 32], features [32, B, 256], embeddings [32, B, 1024].

    Args:
        model: The patched SAM 3 image model.
    """

    def __init__(self, model: nn.Module) -> None:
        super().__init__()
        self.text = model.backbone.language_backbone

    def forward(self, tokens: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Return the padding mask, the resized features and the token embeddings."""
        inputs_embeds = self.text.encoder.token_embedding(tokens)
        _, memory = self.text.encoder(tokens)
        features = self.text.resizer(memory.transpose(0, 1))
        return tokens == 0, features, inputs_embeds.transpose(0, 1)


class Decoder(nn.Module):
    """Image features and one prompt's text features -> the TOP_K best scores, boxes, mask logits.

    Args:
        model: The patched SAM 3 image model.
    """

    def __init__(self, model: nn.Module) -> None:
        super().__init__()
        self.model = model
        self.find_stage = FindStage(
            img_ids=torch.tensor([0], device="cuda", dtype=torch.long),
            text_ids=torch.tensor([0], device="cuda", dtype=torch.long),
            input_boxes=None,
            input_boxes_mask=None,
            input_boxes_label=None,
            input_points=None,
            input_points_mask=None,
        )

    def forward(
        self,
        fpn0: torch.Tensor,
        fpn1: torch.Tensor,
        fpn2: torch.Tensor,
        pos2: torch.Tensor,
        language_mask: torch.Tensor,
        language_features: torch.Tensor,
        language_embeds: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Return the TOP_K scores, their boxes and their mask logits, best first."""
        # Only the coarsest level's position encoding is read; the finer slots never are.
        backbone_out = {
            "vision_features": fpn2,
            "vision_pos_enc": [pos2, pos2, pos2],
            "backbone_fpn": [fpn0, fpn1, fpn2],
            "sam2_backbone_out": None,
            "language_features": language_features,
            "language_mask": language_mask,
            "language_embeds": language_embeds,
        }
        out = self.model.forward_grounding(
            backbone_out=backbone_out,
            find_input=self.find_stage,
            geometric_prompt=self.model._get_dummy_prompt(),
            find_target=None,
        )
        scores = out["pred_logits"].sigmoid().squeeze(-1) * out["presence_logit_dec"].sigmoid()
        top_scores, top = torch.topk(scores, TOP_K, dim=1)
        boxes = out["pred_boxes"].gather(1, top.unsqueeze(-1).expand(-1, -1, 4))
        masks = out["pred_masks"][0, top[0]].unsqueeze(0)
        return top_scores, boxes, masks


def tokenize(model: nn.Module, prompt: str) -> torch.Tensor:
    """Token ids of one prompt from the model's own tokenizer, [1, 32] int64."""
    return model.backbone.language_backbone.tokenizer([prompt], context_length=CONTEXT_LENGTH)


def preprocess(path: Path) -> torch.Tensor:
    """Image file -> normalised [1, 3, 1008, 1008] float32, as Sam3Processor does it."""
    # Only the --image check needs these.
    from PIL import Image
    from torchvision.transforms import v2

    image = v2.functional.to_image(Image.open(path).convert("RGB"))
    transform = v2.Compose(
        [
            v2.ToDtype(torch.uint8, scale=True),
            v2.Resize(size=(INPUT_SIZE_PX, INPUT_SIZE_PX)),
            v2.ToDtype(torch.float32, scale=True),
            v2.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5]),
        ]
    )
    return transform(image).unsqueeze(0)


def export(
    module: nn.Module, args: tuple, path: Path, inputs: list[str], outputs: list[str]
) -> None:
    """Traces one module to ONNX (opset 18), weights in an external data file.

    Args:
        module: The module to trace.
        args: Example inputs for the trace.
        path: The .onnx file to write.
        inputs: Names of the graph's inputs.
        outputs: Names of the graph's outputs.
    """
    t0 = time.perf_counter()
    torch.onnx.export(
        module,
        args,
        str(path),
        input_names=inputs,
        output_names=outputs,
        opset_version=18,
        do_constant_folding=True,
        dynamo=False,
        external_data=True,
    )
    print(f"exported {path.name} in {time.perf_counter() - t0:.0f} s", flush=True)


def compare(name: str, onnx_path: Path, feeds: dict[str, np.ndarray], reference: tuple) -> list:
    """Runs a graph in ONNX Runtime (CPU) and prints the largest error per output.

    Args:
        name: The graph's name for the printout.
        onnx_path: The exported graph.
        feeds: Inputs by name; those the graph does not take are left out.
        reference: PyTorch's outputs, in the graph's output order.

    Returns:
        ONNX Runtime's outputs.
    """
    # Only the --image check needs ONNX Runtime.
    import onnxruntime as ort

    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    used = [i.name for i in session.get_inputs()]
    print(f"{name}: inputs {used}")
    t0 = time.perf_counter()
    results = session.run(None, {k: v for k, v in feeds.items() if k in used})
    print(f"{name}: ORT CPU {time.perf_counter() - t0:.1f} s")
    for out, ref, meta in zip(results, reference, session.get_outputs()):
        ref = ref.detach().float().cpu().numpy() if torch.is_tensor(ref) else ref
        err = np.abs(out.astype(np.float32) - ref.astype(np.float32))
        scale = np.abs(ref).max() + 1e-9
        print(f"  {meta.name} {out.shape}: max abs {err.max():.3e}, rel {err.max() / scale:.3e}")
    return results


def main() -> None:
    """Parse the arguments, export the chosen graphs and optionally check them."""
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--image", type=Path)
    parser.add_argument("--prompt", default="cup")
    parser.add_argument("--parts", nargs="+", choices=PARTS, default=list(PARTS))
    parser.add_argument(
        "--bf16", action="store_true", help="trace under bf16 autocast (as Meta runs it)"
    )
    parser.add_argument(
        "--suffix", default=None, help='file name tag; default ".bf16" with --bf16, else none'
    )
    args = parser.parse_args()
    suffix = args.suffix if args.suffix is not None else (".bf16" if args.bf16 else "")
    args.out.mkdir(parents=True, exist_ok=True)

    model = build_sam3_image_model(checkpoint_path=str(args.checkpoint), load_from_HF=False)
    model = model.float().eval()
    patch_for_export(model)

    bpe = Path(sys.modules["sam3"].__file__).parent / "assets" / "bpe_simple_vocab_16e6.txt.gz"
    (args.out / "bpe_simple_vocab_16e6.txt").write_bytes(gzip.decompress(bpe.read_bytes()))

    image = (
        preprocess(args.image).cuda()
        if args.image
        else torch.zeros(1, 3, INPUT_SIZE_PX, INPUT_SIZE_PX, device="cuda")
    )
    tokens = tokenize(model, args.prompt).cuda()
    print("tokens", tokens.tolist())
    image_encoder, text_encoder, decoder = ImageEncoder(model), TextEncoder(model), Decoder(model)

    def precision() -> Any:
        return torch.autocast("cuda", torch.bfloat16) if args.bf16 else torch.no_grad()

    with torch.inference_mode(), precision():
        image_out = image_encoder(image)
        text_out = text_encoder(tokens)
        decoder_out = decoder(*image_out, *text_out)
        print("top scores torch", decoder_out[0][0, :5].float().tolist())
    fpn_names = ["fpn0", "fpn1", "fpn2", "pos2"]
    text_names = ["language_mask", "language_features", "language_embeds"]
    decoder_names = ["scores", "boxes", "mask_logits"]
    paths = {
        part: args.out / f"sam3_{part}{suffix}.onnx"
        for part in ("image_encoder", "text_encoder", "decoder")
    }
    with torch.no_grad(), precision():
        if "image" in args.parts:
            export(image_encoder, (image,), paths["image_encoder"], ["image"], fpn_names)
        if "text" in args.parts:
            export(text_encoder, (tokens,), paths["text_encoder"], ["tokens"], text_names)
        if "decoder" in args.parts:
            export(
                decoder,
                (*image_out, *text_out),
                paths["decoder"],
                fpn_names + text_names,
                decoder_names,
            )
    if not args.image:
        return
    if args.bf16:
        print("check skipped under --bf16")
        return
    np_image = image.cpu().numpy()
    if "image" in args.parts:
        compare("image encoder", paths["image_encoder"], {"image": np_image}, image_out)
    if "text" in args.parts:
        compare("text encoder", paths["text_encoder"], {"tokens": tokens.cpu().numpy()}, text_out)
    if "decoder" in args.parts:
        feeds = {
            n: t.cpu().numpy() for n, t in zip(fpn_names + text_names, (*image_out, *text_out))
        }
        out = compare("decoder", paths["decoder"], feeds, decoder_out)
        print("top scores onnx", out[0][0, :5].tolist())


if __name__ == "__main__":
    main()
