"""particle_filter demo — assembly only (see demos/demo_common.py).

    PYTHONPATH=$PWD/python python -m slam.demos.demo_particle_filter \
        --scenario maps/scenarios/corridor03_drift.yaml \
        --params configs/filtering/particle_filter.yaml --trace out/trace.jsonl
"""

from __future__ import annotations

from ..filtering.particle_filter import ParticleFilter
from .demo_common import run


def main() -> None:
    run("particle_filter", ParticleFilter)


if __name__ == "__main__":
    main()
