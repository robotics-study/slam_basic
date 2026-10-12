"""ekf_slam demo — assembly only (see demos/demo_common.py).

    PYTHONPATH=$PWD/python python -m slam.demos.demo_ekf_slam \
        --scenario maps/scenarios/office01_landmarks.yaml \
        --params configs/filter_based/ekf_slam.yaml \
        --trace out/trace.jsonl
"""

from __future__ import annotations

from ..filter_based.ekf_slam import EkfSlam
from .demo_common import run


def main() -> None:
    run("ekf_slam", EkfSlam)


if __name__ == "__main__":
    main()
