"""mcl demo — assembly only (see demos/demo_common.py).

    PYTHONPATH=$PWD/python python -m slam.demos.demo_mcl \
        --scenario maps/scenarios/corridor03_drift.yaml \
        --params configs/filtering/mcl.yaml --trace out/trace.jsonl
"""

from __future__ import annotations

from ..filtering.mcl import Mcl
from .demo_common import run


def main() -> None:
    run("mcl", Mcl)


if __name__ == "__main__":
    main()
