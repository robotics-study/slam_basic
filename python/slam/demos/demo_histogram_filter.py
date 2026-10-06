"""histogram_filter demo — assembly only (see demos/demo_common.py).

    PYTHONPATH=$PWD/python python -m slam.demos.demo_histogram_filter \
        --scenario maps/scenarios/corridor01_back_and_forth.yaml \
        --params configs/filtering/histogram_filter.yaml --trace out/trace.jsonl
"""

from __future__ import annotations

from ..filtering.histogram_filter import HistogramFilter
from .demo_common import run


def main() -> None:
    run("histogram_filter", HistogramFilter)


if __name__ == "__main__":
    main()
