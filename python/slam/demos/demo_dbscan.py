"""dbscan demo — assembly only (see demos/demo_common.py).

    PYTHONPATH=$PWD/python python -m slam.demos.demo_dbscan \
        --scenario maps/scenarios/blobs01_line.yaml --params configs/features/dbscan.yaml \
        --trace out/trace.jsonl
"""

from __future__ import annotations

from ..features.dbscan import Dbscan
from .demo_common import run


def main() -> None:
    run("dbscan", Dbscan)


if __name__ == "__main__":
    main()
