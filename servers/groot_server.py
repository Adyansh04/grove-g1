#!/usr/bin/env python3
"""Serves a GR00T policy for the learned-grasp engine, on the host rather than in the container.

The container has no torch: g1_vla_groot_adapter speaks this server's ZMQ protocol instead.

Wraps upstream's entry point to load the checkpoint in bf16. Upstream's bare
AutoModel.from_pretrained upcasts to fp32 in host RAM first, about 4 GB more at peak, which on a
30 GB machine gets the load OOM-killed in a way that reads as a CUDA fault. Patching here keeps
the upstream checkout a clean tag.

    ./scripts/groot_server.py --model-path nvidia/GR00T-N1.7-3B --embodiment-tag real_g1

Run scripts/setup-groot.sh first. Every argument is upstream's; see --help.
"""

import os
import sys

VENV_HINT = "scripts/setup-groot.sh has not been run, or GROOT_HOME points somewhere else"

try:
    import torch
    import transformers
except ImportError as error:  # pragma: no cover - depends on the host environment
    sys.exit(f"{error}. {VENV_HINT}")


def _load_in_bfloat16():
    """Makes every AutoModel load keep the checkpoint's own precision."""
    original = transformers.AutoModel.from_pretrained

    def from_pretrained(*args, **kwargs):
        kwargs.setdefault("dtype", torch.bfloat16)
        kwargs.setdefault("low_cpu_mem_usage", True)
        return original(*args, **kwargs)

    transformers.AutoModel.from_pretrained = from_pretrained


def _add_groot_to_path():
    """Puts the upstream checkout on sys.path, where setup-groot.sh clones it.

    Needed when the clone was not installed into the virtualenv.
    """
    home = os.environ.get("GROOT_HOME", os.path.expanduser("~/ref/Isaac-GR00T"))
    if os.path.isdir(os.path.join(home, "gr00t")) and home not in sys.path:
        sys.path.insert(0, home)


def main():
    _load_in_bfloat16()
    try:
        import tyro
        from gr00t.eval.run_gr00t_server import ServerConfig, main as serve
    except ImportError:
        _add_groot_to_path()
        try:
            import tyro
            from gr00t.eval.run_gr00t_server import ServerConfig, main as serve
        except ImportError as error:
            sys.exit(f"{error}. {VENV_HINT}")

    # Every checkpoint pulls a gated backbone, and without credentials that fails deep in the
    # model constructor with no mention of authentication.
    if not (
        os.environ.get("HF_TOKEN")
        or os.path.exists(os.path.expanduser("~/.cache/huggingface/token"))
    ):
        print(
            "warning: no Hugging Face token found. nvidia/Cosmos-Reason2-2B is gated and every "
            "GR00T checkpoint loads it; run `hf auth login` and accept the terms on its model "
            "page first.",
            file=sys.stderr,
        )

    serve(tyro.cli(ServerConfig))


if __name__ == "__main__":
    main()
