"""Map / scenario loaders. Dispatch on the yaml `type` field, not the extension.

A scenario IS an episode's input stream definition (spec/data_formats.md): the GT
waypoint polyline + step spacing, the sensor block, optional landmark points, the
odometry noise sigmas and the seed. The loader parses; core/sim builds the Step
stream from it (the loader never touches rng).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from ..core.types import Point, SensorConfig
from .occupancy_grid import OccupancyGrid2D
from .pgm import read_pgm


def load_map(path: str | Path) -> OccupancyGrid2D:
    path = Path(path)
    with open(path, encoding="utf-8") as fh:
        raw = yaml.safe_load(fh)
    map_type = raw.get("type")
    if map_type != "occupancy_grid":
        raise ValueError(f"unsupported map type {map_type!r} (only occupancy_grid)")
    image_path = (path.parent / raw["image"]).resolve()
    _, _, pixels = read_pgm(str(image_path))
    origin = raw["origin"]
    return OccupancyGrid2D(
        pixels=pixels,
        resolution=float(raw["resolution"]),
        origin=(float(origin[0]), float(origin[1])),
        free_thresh=float(raw.get("free_thresh", 0.65)),
    )


@dataclass(frozen=True)
class Scenario:
    """One parsed scenario yaml: the map, the GT polyline (world coords), the arc
    spacing, the sensor block, landmark points (id = list order; None for beam),
    the odometry noise sigmas and the splitmix64 seed."""

    map_path: str  # resolved absolute path to the map yaml
    grid: OccupancyGrid2D
    waypoints: tuple[Point, ...]
    step_meters: float
    sensor: SensorConfig
    landmarks: tuple[Point, ...] | None
    sigma_xy: float
    sigma_theta: float
    seed: int


def load_scenario(path: str | Path) -> Scenario:
    path = Path(path)
    with open(path, encoding="utf-8") as fh:
        raw = yaml.safe_load(fh)
    for key in ("map", "path", "step_meters", "sensor", "odom_noise", "seed"):
        if key not in raw:
            raise ValueError(f"scenario {path} missing required key '{key}'")
    grid = load_map(path.parent / raw["map"])
    waypoints = tuple((float(p[0]), float(p[1])) for p in raw["path"])
    sensor_raw = raw["sensor"]
    stype = sensor_raw.get("type")
    if stype not in ("beam", "landmarks"):
        raise ValueError(f"scenario {path} sensor.type must be beam|landmarks, got {stype!r}")
    landmarks: tuple[Point, ...] | None = None
    if stype == "landmarks":
        landmarks = tuple((float(p[0]), float(p[1])) for p in raw["landmarks"])
    sensor = SensorConfig(
        type=stype,
        range_max=float(sensor_raw["range_max"]),
        sigma_range=float(sensor_raw["sigma_range"]),
        beams=int(sensor_raw["beams"]) if "beams" in sensor_raw else None,
        fov_deg=float(sensor_raw["fov_deg"]) if "fov_deg" in sensor_raw else None,
        sigma_bearing=(
            float(sensor_raw["sigma_bearing"]) if "sigma_bearing" in sensor_raw else None
        ),
    )
    return Scenario(
        map_path=str((path.parent / raw["map"]).resolve()),
        grid=grid,
        waypoints=waypoints,
        step_meters=float(raw["step_meters"]),
        sensor=sensor,
        landmarks=landmarks,
        sigma_xy=float(raw["odom_noise"]["sigma_xy"]),
        sigma_theta=float(raw["odom_noise"]["sigma_theta"]),
        seed=int(raw["seed"]),
    )
