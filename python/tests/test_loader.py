"""Scenario / map loaders against the real repo yaml files."""

from conftest import REPO_ROOT

from slam.maps.loader import load_map, load_scenario


def test_load_real_grid() -> None:
    g = load_map(REPO_ROOT / "maps" / "grid" / "corridor01.yaml")
    assert (g.height, g.width) == (7, 25)
    assert g.resolution == 0.5
    assert g.origin == (0.0, 0.0)
    # Border walls are occupied, corridor center is free.
    assert g.occupied(0, 0) and not g.occupied(3, 1)


def test_load_beam_scenario() -> None:
    sc = load_scenario(REPO_ROOT / "maps" / "scenarios" / "corridor01_back_and_forth.yaml")
    assert sc.sensor.type == "beam"
    assert sc.sensor.beams == 90 and sc.sensor.fov_deg == 180.0
    assert sc.sensor.sigma_bearing is None
    assert sc.landmarks is None
    assert sc.step_meters == 0.5
    assert sc.seed == 42
    assert len(sc.waypoints) == 3


def test_load_landmarks_scenario() -> None:
    sc = load_scenario(REPO_ROOT / "maps" / "scenarios" / "office01_landmarks.yaml")
    assert sc.sensor.type == "landmarks"
    assert sc.sensor.beams is None and sc.sensor.fov_deg is None
    assert sc.sensor.sigma_bearing == 0.03
    assert sc.landmarks is not None and len(sc.landmarks) == 10
