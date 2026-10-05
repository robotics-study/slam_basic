"""PGM reader (P2 ascii + P5 binary) and the occupancy grid contract."""

import numpy as np
import pytest

from slam.maps.occupancy_grid import OccupancyGrid2D
from slam.maps.pgm import read_pgm


def test_pgm_p2_roundtrip(tmp_path) -> None:
    p = tmp_path / "m.pgm"
    p.write_text("P2\n# comment\n2 2\n255\n0 255\n128 42\n", encoding="utf-8")
    w, h, px = read_pgm(str(p))
    assert (w, h) == (2, 2)
    assert px.shape == (2, 2)
    assert px[0][0] == 0 and px[0][1] == 255 and px[1][0] == 128 and px[1][1] == 42


def test_pgm_p5_roundtrip(tmp_path) -> None:
    raw = bytearray(b"P5\n2 2\n255\n")
    raw += bytes([0, 255, 128, 42])
    p = tmp_path / "m.pgm"
    p.write_bytes(bytes(raw))
    w, h, px = read_pgm(str(p))
    assert (w, h) == (2, 2)
    assert px.tolist() == [[0, 255], [128, 42]]


def test_pgm_rejects_other_magic(tmp_path) -> None:
    p = tmp_path / "m.pgm"
    p.write_text("P6\n1 1\n255\n0", encoding="utf-8")
    with pytest.raises(ValueError):
        read_pgm(str(p))


def test_grid_threshold_and_frames() -> None:
    # 2x1 grid, resolution 0.5, origin (1.0, 2.0). Row 0 = TOP image row.
    pixels = np.array([[0, 255]], dtype=np.uint16)
    g = OccupancyGrid2D(pixels=pixels, resolution=0.5, origin=(1.0, 2.0))
    assert (g.height, g.width) == (1, 2)
    assert g.occupied(0, 0) and not g.occupied(0, 1)
    # cell_to_world fixed formula: x = ox + (c+0.5)r ; y = oy + ((H-1-r)+0.5)r
    assert g.cell_to_world(0, 0) == (1.0 + 0.5 * 0.5, 2.0 + 0.5 * 0.5)
    # world_to_cell inverts it exactly at the center.
    x, y = g.cell_to_world(0, 1)
    assert g.world_to_cell(x, y) == (0, 1)
    assert not g.in_bounds(1, 0) and not g.in_bounds(0, 2)
    # Out of bounds is never occupied.
    assert not g.occupied(5, 5)


def test_grid_free_mask_shape() -> None:
    pixels = np.array([[0, 255], [255, 255]], dtype=np.uint16)
    g = OccupancyGrid2D(pixels=pixels, resolution=0.25, origin=(0.0, 0.0))
    fm = g.free_mask()
    assert fm.shape == (2, 2)
    assert bool(fm[0][0]) is False and bool(fm[1][1]) is True
