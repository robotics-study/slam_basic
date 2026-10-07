"""grid_mapping demo — assembly only (see demos/demo_common.py).

    PYTHONPATH=$PWD/python python -m slam.demos.demo_grid_mapping \
        --scenario maps/scenarios/office01_tour.yaml \
        --params configs/filtering/grid_mapping.yaml --trace out/trace.jsonl
"""

from __future__ import annotations

from ..filtering.grid_mapping import GridMapping
from .demo_common import run


def main() -> None:
    run("grid_mapping", GridMapping)


if __name__ == "__main__":
    main()
