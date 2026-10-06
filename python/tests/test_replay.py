"""replay 의 표시 좌표 계약 — 한 번 조용히 깨졌던 것 (대칭 맵에서는 티가 안 난다).

래스터는 flipud + origin="lower" 로 그려진다: 셀 행 r(row 0 = 위)이 디스플레이 밴드
[h-1-r, h-r) 을 차지한다. 셀 중심에 있는 점은 자기 밴드에 착지해야 한다 — 이건 체인의
양쪽(래스터 방향과 to_display)을 모두 고정한다. 대칭 맵에서는 미러가 무해하지만 그 외엔
점이 벽을 뚫고 지나간 것처럼 보인다.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
import numpy as np
import pytest

matplotlib.use("Agg")  # 헤드리스 렌더 — 창 없음
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools" / "viz"))

import replay  # noqa: E402

from slam.maps.occupancy_grid import OccupancyGrid2D  # noqa: E402


def _asymmetric_grid() -> OccupancyGrid2D:
    """row 0 = 위: 오른쪽 셀만 점유 — 두 축 모두 비대칭이라 미러는 숨을 데가 없다."""
    pixels = np.array([[255, 0], [255, 255]], dtype=np.uint16)
    return OccupancyGrid2D(pixels=pixels, resolution=1.0, origin=(0.0, 0.0))


def test_to_display_lands_on_own_cell_band() -> None:
    """cell_to_world 와 to_display 는 상호 역이다: 셀 (r,c) 중심의 점은 그 셀이 그려지는
    밴드 [h-1-r, h-r) 안에 착지한다."""
    grid = _asymmetric_grid()
    h = grid.height
    for r in range(h):
        for c in range(grid.width):
            u, v = replay.to_display(
                grid.cell_to_world(r, c), grid.origin, grid.resolution, h
            )
            assert u == pytest.approx(c + 0.5)
            assert v == pytest.approx(h - 1 - r + 0.5)


def test_raster_is_drawn_row_zero_on_top() -> None:
    """래스터의 방향 고정: row 0(세계 좌표의 위)이 화면 위쪽에 그려진다 — 점과 같은 프레임."""
    grid = _asymmetric_grid()
    scene = replay.Scene(grid=grid)
    fig, ax = plt.subplots(figsize=(2, 2), dpi=40)
    try:
        replay.draw(ax, scene, cutoff=0.0)
        img = ax.images[0]
        assert tuple(img.get_extent()) == (
            0.0, float(grid.width), 0.0, float(grid.height),
        )
        assert not ax.yaxis_inverted()  # y 는 위로 증가 — 점의 v 와 같은 방향
        assert np.array_equal(np.asarray(img.get_array()), np.flipud(grid.free_mask()))
    finally:
        plt.close(fig)
