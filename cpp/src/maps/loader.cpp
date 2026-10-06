#include "slam/maps/loader.hpp"

#include <filesystem>
#include <stdexcept>

#include "slam/core/yaml.hpp"
#include "slam/maps/pgm.hpp"

namespace slam::maps {
namespace fs = std::filesystem;
using core::YamlNode;

namespace {

// Resolve a path referenced inside a yaml file relative to that file's directory.
std::string resolve(const std::string& base_file, const std::string& ref) {
  fs::path p(ref);
  if (p.is_absolute()) return fs::weakly_canonical(p).string();
  fs::path dir = fs::path(base_file).parent_path();
  return fs::weakly_canonical(dir / p).string();
}

}  // namespace

OccupancyGrid2D load_map(const std::string& path) {
  YamlNode root = core::parse_yaml_file(path);
  std::string type = root.at("type").as_string();
  if (type != "occupancy_grid") {
    throw std::runtime_error("unsupported map type '" + type + "' (only occupancy_grid)");
  }
  std::string image_path = resolve(path, root.at("image").as_string());
  PgmImage img = load_pgm(image_path);
  const YamlNode& origin = root.at("origin");
  std::array<double, 2> origin_xy{origin.seq.at(0).as_double(), origin.seq.at(1).as_double()};
  double free_thresh = root.has("free_thresh") ? root.at("free_thresh").as_double() : 0.65;
  return OccupancyGrid2D(std::move(img.pixels), img.height, img.width,
                         root.at("resolution").as_double(), origin_xy, free_thresh);
}

Scenario load_scenario(const std::string& path) {
  YamlNode root = core::parse_yaml_file(path);
  // Required keys throw through at() when missing (map/path/step_meters/sensor/
  // odom_noise/seed — the Python loader raises on exactly these too).
  Scenario sc;
  sc.map_path = resolve(path, root.at("map").as_string());
  sc.grid = load_map(sc.map_path);
  for (const YamlNode& p : root.at("path").seq) {
    sc.waypoints.emplace_back(p.seq.at(0).as_double(), p.seq.at(1).as_double());
  }
  sc.step_meters = root.at("step_meters").as_double();

  const YamlNode& sensor_raw = root.at("sensor");
  std::string stype = sensor_raw.at("type").as_string();
  if (stype != "beam" && stype != "landmarks") {
    throw std::runtime_error("scenario " + path + " sensor.type must be beam|landmarks, got '" +
                             stype + "'");
  }
  sc.sensor.type = stype;
  sc.sensor.range_max = sensor_raw.at("range_max").as_double();
  sc.sensor.sigma_range = sensor_raw.at("sigma_range").as_double();
  if (sensor_raw.has("beams")) {
    sc.sensor.has_beams = true;
    sc.sensor.beams = static_cast<int>(sensor_raw.at("beams").as_int());
  }
  if (sensor_raw.has("fov_deg")) {
    sc.sensor.has_fov_deg = true;
    sc.sensor.fov_deg = sensor_raw.at("fov_deg").as_double();
  }
  if (sensor_raw.has("sigma_bearing")) {
    sc.sensor.has_sigma_bearing = true;
    sc.sensor.sigma_bearing = sensor_raw.at("sigma_bearing").as_double();
  }
  if (stype == "landmarks") {
    sc.has_landmarks = true;
    for (const YamlNode& p : root.at("landmarks").seq) {
      sc.landmarks.emplace_back(p.seq.at(0).as_double(), p.seq.at(1).as_double());
    }
  }
  const YamlNode& odom_noise = root.at("odom_noise");
  sc.sigma_xy = odom_noise.at("sigma_xy").as_double();
  sc.sigma_theta = odom_noise.at("sigma_theta").as_double();
  sc.seed = static_cast<long long>(root.at("seed").as_int());
  return sc;
}

}  // namespace slam::maps
