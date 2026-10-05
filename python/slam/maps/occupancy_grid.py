"""OccupancyGrid2D — ROS map_server-style raster implementing the ScanGrid protocol.

world<->grid conversion lives ONLY here (the map layer owns the coordinate frames,
per the repo rule): cell (r, c) center = (origin_x + (c+0.5)*res, origin_y +
((H-1-r)+0.5)*res); inverse c = floor((x-origin_x)/res), r = (H-1) - floor((y-
origin_y)/res). Row 0 is the TOP image row — both formulas encode that flip; C++
mirrors them operation for operation.

Occupancy follows spec/data_formats.md: occ = 1 - pixel/255, a cell is occupied iff
occ > free_thresh (default 0.65). The repo's maps use only 0/255 so the threshold
never wobbles; it exists because ROS yamls may carry unknowns.
"""

from __future__ import annotations

import math

import numpy as np

from ..core.types import Cell, Point, ScanGrid


class OccupancyGrid2D(ScanGrid):
    def __init__(
        self,
        pixels: np.ndarray,
        resolution: float,
        origin: tuple[float, float],
        free_thresh: float = 0.65,
    ) -> None:
        self._height, self._width = int(pixels.shape[0]), int(pixels.shape[1])
        self._resolution = resolution
        self._origin_x, self._origin_y = origin[0], origin[1]
        # occ = 1 - p/255; occupied iff occ > free_thresh (unknown counts as
        # occupied — only clearly-free cells are free).
        occ = 1.0 - pixels.astype(np.float64) / 255.0
        self._free = occ <= free_thresh

    # --- dimensions -------------------------------------------------------
    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height

    @property
    def resolution(self) -> float:
        return self._resolution

    @property
    def origin(self) -> tuple[float, float]:
        """World pose (x, y) of the bottom-left pixel — readers outside the map
        layer convert with the same frame."""
        return (self._origin_x, self._origin_y)

    def free_mask(self) -> np.ndarray:
        """Boolean [H, W] mask of traversable cells (read-only view for viz/metrics)."""
        return self._free

    # --- coordinate frames (owned here only) -------------------------------
    def cell_to_world(self, row: int, col: int) -> Point:
        x = self._origin_x + (col + 0.5) * self._resolution
        y = self._origin_y + ((self._height - 1 - row) + 0.5) * self._resolution
        return (x, y)

    def world_to_cell(self, x: float, y: float) -> Cell:
        col = int(math.floor((x - self._origin_x) / self._resolution))
        row = (self._height - 1) - int(math.floor((y - self._origin_y) / self._resolution))
        return (row, col)

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self._height and 0 <= col < self._width

    def occupied(self, row: int, col: int) -> bool:
        """Out of bounds counts as NOT occupied (a ray leaves the map = miss; the
        scenario guarantees the robot itself never leaves free space)."""
        return self.in_bounds(row, col) and not bool(self._free[row][col])
