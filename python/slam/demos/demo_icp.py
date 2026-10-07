"""icp demo — assembly only (see demos/demo_common.py).

    PYTHONPATH=$PWD/python python -m slam.demos.demo_icp \
        --scenario maps/scenarios/room01_straight.yaml \
        --params configs/registration/icp.yaml --trace out/trace.jsonl
"""

from __future__ import annotations

from ..registration.icp import Icp
from .demo_common import run


def main() -> None:
    run("icp", Icp)


if __name__ == "__main__":
    main()
