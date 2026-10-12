"""ndt demo — assembly only (see demos/demo_common.py).

    PYTHONPATH=$PWD/python python -m slam.demos.demo_ndt \
        --scenario maps/scenarios/room02_diagonal.yaml \
        --params configs/registration/ndt.yaml --trace out/trace.jsonl
"""

from __future__ import annotations

from ..registration.ndt import Ndt
from .demo_common import run


def main() -> None:
    run("ndt", Ndt)


if __name__ == "__main__":
    main()
