#!/usr/bin/env python3
"""Serves a GR00T policy for the learned-grasp engine, on the host rather than in the container.

The container deliberately has no torch: g1_vla_groot_adapter speaks this server's ZMQ protocol
instead of importing the model's package, so the ROS image never grows a CUDA dependency tree.
That leaves the model needing somewhere to run, and this is it.

Wraps upstream's own entry point rather than replacing it, for one reason. Gr00tPolicy loads the
checkpoint with a bare AutoModel.from_pretrained, which upcasts bf16 weights to fp32 in host RAM
and only casts back down afterwards. Measured on this checkpoint: 15.2 GB peak resident unpatched
against 11.1 GB with the dtype set at load time. Four gigabytes is not interesting on a big
machine and is the whole margin on a 30 GB one, where the kernel kills the load partway through
and it reads as a CUDA fault rather than an out-of-memory one.

Fixing that here keeps the upstream checkout a clean tag, which matters because the alternative is
a patched clone that every future `git pull` silently reverts.

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
    """Upstream is normally run from inside its own checkout, which puts it on sys.path for free.

    Running it from here does not, and a clone that predates scripts/setup-groot.sh will not have
    been installed into the virtualenv either, so look where the setup script puts it.
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

    # The VLM backbone is a gated repo and every checkpoint pulls it, so an unauthenticated run
    # fails deep inside the model constructor with a stack trace about the backbone rather than
    # about credentials.
    if not (os.environ.get("HF_TOKEN") or os.path.exists(
            os.path.expanduser("~/.cache/huggingface/token"))):
        print(
            "warning: no Hugging Face token found. nvidia/Cosmos-Reason2-2B is gated and every "
            "GR00T checkpoint loads it; run `hf auth login` and accept the terms on its model "
            "page first.",
            file=sys.stderr,
        )

    serve(tyro.cli(ServerConfig))


if __name__ == "__main__":
    main()
