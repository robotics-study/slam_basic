"""Shared test helpers: repo paths + small in-memory grids."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from slam.maps.occupancy_grid import OccupancyGrid2D

REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = REPO_ROOT / "configs"
SCENARIO_DIR = REPO_ROOT / "maps" / "scenarios"


def config(algo: str) -> Path:
    found = sorted(CONFIG_DIR.rglob(f"{algo}.yaml"))
    assert len(found) == 1, f"expected exactly one configs/<section>/{algo}.yaml"
    return found[0]


def grid_from(rows: list[str], resolution: float = 1.0) -> OccupancyGrid2D:
    """Build a grid from ascii rows: '.' = free (255), '#' = occupied (0)."""
    pixels = np.array(
        [[255 if ch == "." else 0 for ch in row] for row in rows], dtype=np.uint16
    )
    return OccupancyGrid2D(pixels=pixels, resolution=resolution, origin=(0.0, 0.0))
