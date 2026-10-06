#include "slam/core/trace.hpp"

#include <cmath>
#include <format>

namespace slam::core {
namespace {

void write_str(std::ostream& os, const std::string& s) {
  os << '"';
  for (char c : s) {
    switch (c) {
      case '"': os << "\\\""; break;
      case '\\': os << "\\\\"; break;
      case '\n': os << "\\n"; break;
      case '\t': os << "\\t"; break;
      case '\r': os << "\\r"; break;
      default: os << c;
    }
  }
  os << '"';
}

void write_point(std::ostream& os, const Point& p) {
  os << '[';
  write_num(os, p.x);
  os << ',';
  write_num(os, p.y);
  os << ']';
}

void write_pose(std::ostream& os, const Pose& p) {
  os << '[';
  write_num(os, p.x);
  os << ',';
  write_num(os, p.y);
  os << ',';
  write_num(os, p.theta);
  os << ']';
}

void write_param(std::ostream& os, const ParamValue& v) {
  std::visit(
      [&](const auto& v) {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<T, bool>) {
          os << (v ? "true" : "false");
        } else if constexpr (std::is_same_v<T, int>) {
          os << v;
        } else if constexpr (std::is_same_v<T, double>) {
          write_num(os, v);
        } else {
          write_str(os, v);
        }
      },
      v);
}

}  // namespace

void write_num(std::ostream& os, double v) {
  // Integral values emit integer bytes (Python's json writes 5.0 where this writes
  // 5 — parsed values are equal); everything else takes std::format's default float
  // format: the shortest decimal that re-reads to the same double, exactly what
  // Python's repr emits.
  if (std::isfinite(v) && std::abs(v) < 9e15 && v == std::floor(v)) {
    os << static_cast<long long>(v);
  } else {
    os << std::format("{}", v);
  }
}

void TraceRecorder::begin_event(const char* event) {
  // No wall-clock field: a SLAM trace is ordered by seq alone (see header).
  os_ << "{\"seq\":" << seq_++ << ",\"event\":\"" << event << '"';
}

void TraceRecorder::end_event() { os_ << "}\n"; }

void TraceRecorder::run_started(const std::string& algorithm, const std::string& scenario,
                               const std::map<std::string, ParamValue>& params, long long seed,
                               const SensorConfig& sensor,
                               const std::vector<Point>* landmarks) {
  begin_event("run_started");
  os_ << ",\"algorithm\":";
  write_str(os_, algorithm);
  os_ << ",\"scenario\":";
  write_str(os_, scenario);
  // std::map iterates sorted by key — the Python recorder sorts explicitly to match.
  os_ << ",\"params\":{";
  bool first = true;
  for (const auto& [k, v] : params) {
    if (!first) os_ << ',';
    first = false;
    write_str(os_, k);
    os_ << ':';
    write_param(os_, v);
  }
  os_ << '}';
  os_ << ",\"seed\":" << seed;
  // The sensor block keeps the scenario yaml's fixed field order (optional keys omitted).
  os_ << ",\"sensor\":{\"type\":";
  write_str(os_, sensor.type);
  if (sensor.has_beams) {
    os_ << ",\"beams\":" << sensor.beams;
    os_ << ",\"fov_deg\":";
    write_num(os_, sensor.fov_deg);
  }
  os_ << ",\"range_max\":";
  write_num(os_, sensor.range_max);
  os_ << ",\"sigma_range\":";
  write_num(os_, sensor.sigma_range);
  if (sensor.has_sigma_bearing) {
    os_ << ",\"sigma_bearing\":";
    write_num(os_, sensor.sigma_bearing);
  }
  os_ << '}';
  if (landmarks != nullptr) {
    os_ << ",\"landmarks\":[";
    for (size_t i = 0; i < landmarks->size(); ++i) {
      if (i) os_ << ',';
      write_point(os_, (*landmarks)[i]);
    }
    os_ << ']';
  }
  end_event();
}

void TraceRecorder::step_observed(const Step& step) {
  begin_event("step_observed");
  os_ << ",\"t\":" << step.t << ",\"gt\":";
  write_pose(os_, step.gt);
  if (step.has_odom) {
    os_ << ",\"odom\":[";
    write_num(os_, step.odom.dx);
    os_ << ',';
    write_num(os_, step.odom.dy);
    os_ << ',';
    write_num(os_, step.odom.dtheta);
    os_ << ']';
  }
  if (step.has_scan) {
    os_ << ",\"scan\":[";
    for (size_t i = 0; i < step.scan.size(); ++i) {
      if (i) os_ << ',';
      write_point(os_, step.scan[i]);
    }
    os_ << ']';
  }
  if (step.has_obs) {
    os_ << ",\"obs\":[";
    for (size_t i = 0; i < step.obs.size(); ++i) {
      const LandmarkObs& o = step.obs[i];
      if (i) os_ << ',';
      os_ << "{\"id\":" << o.id << ",\"bearing\":";
      write_num(os_, o.bearing);
      os_ << ",\"range\":";
      write_num(os_, o.range);
      os_ << '}';
    }
    os_ << ']';
  }
  end_event();
}

void TraceRecorder::pose_estimated(int t, const Pose& pose,
                                   const std::optional<std::array<double, 3>>& cov) {
  begin_event("pose_estimated");
  os_ << ",\"t\":" << t << ",\"pose\":";
  write_pose(os_, pose);
  if (cov.has_value()) {
    os_ << ",\"cov\":[";
    for (size_t i = 0; i < cov->size(); ++i) {
      if (i) os_ << ',';
      write_num(os_, (*cov)[i]);
    }
    os_ << ']';
  }
  end_event();
}

void TraceRecorder::belief_updated(int t, const std::vector<CellProb>& cells) {
  begin_event("belief_updated");
  os_ << ",\"t\":" << t << ",\"cells\":[";
  for (size_t i = 0; i < cells.size(); ++i) {
    if (i) os_ << ',';
    os_ << '[' << cells[i].row << ',' << cells[i].col << ',';
    write_num(os_, cells[i].p);
    os_ << ']';
  }
  os_ << ']';
  end_event();
}

void TraceRecorder::particles_updated(int t, const std::vector<Particle>& particles) {
  begin_event("particles_updated");
  os_ << ",\"t\":" << t << ",\"particles\":[";
  for (size_t i = 0; i < particles.size(); ++i) {
    const Particle& p = particles[i];
    if (i) os_ << ',';
    os_ << '[';
    write_num(os_, p.x);
    os_ << ',';
    write_num(os_, p.y);
    os_ << ',';
    write_num(os_, p.theta);
    os_ << ',';
    write_num(os_, p.w);
    os_ << ']';
  }
  os_ << ']';
  end_event();
}

void TraceRecorder::landmarks_updated(int t, const std::vector<LandmarkEstimate>& estimated) {
  begin_event("landmarks_updated");
  os_ << ",\"t\":" << t << ",\"estimated\":[";
  for (size_t i = 0; i < estimated.size(); ++i) {
    const LandmarkEstimate& e = estimated[i];
    if (i) os_ << ',';
    os_ << "{\"id\":" << e.id << ",\"x\":";
    write_num(os_, e.x);
    os_ << ",\"y\":";
    write_num(os_, e.y);
    if (e.has_sigma) {
      os_ << ",\"sx\":";
      write_num(os_, e.sx);
      os_ << ",\"sy\":";
      write_num(os_, e.sy);
    }
    os_ << '}';
  }
  os_ << ']';
  end_event();
}

void TraceRecorder::map_updated(int t, const std::vector<CellProb>& cells) {
  begin_event("map_updated");
  os_ << ",\"t\":" << t << ",\"cells\":[";
  for (size_t i = 0; i < cells.size(); ++i) {
    if (i) os_ << ',';
    os_ << '[' << cells[i].row << ',' << cells[i].col << ',';
    write_num(os_, cells[i].p);
    os_ << ']';
  }
  os_ << ']';
  end_event();
}

void TraceRecorder::constraint_added(int i, int j, const Twist& d,
                                    const std::optional<bool>& loop) {
  begin_event("constraint_added");
  os_ << ",\"i\":" << i << ",\"j\":" << j << ",\"d\":[";
  write_num(os_, d.dx);
  os_ << ',';
  write_num(os_, d.dy);
  os_ << ',';
  write_num(os_, d.dtheta);
  os_ << ']';
  if (loop.has_value()) {
    os_ << ",\"loop\":" << (*loop ? "true" : "false");
  }
  end_event();
}

void TraceRecorder::trajectory_found(const std::vector<Pose>& poses) {
  begin_event("trajectory_found");
  os_ << ",\"poses\":[";
  for (size_t i = 0; i < poses.size(); ++i) {
    if (i) os_ << ',';
    write_pose(os_, poses[i]);
  }
  os_ << ']';
  end_event();
}

void TraceRecorder::run_finished(const std::map<std::string, double>& metrics) {
  begin_event("run_finished");
  os_ << ",\"metrics\":{";
  bool first = true;
  for (const auto& [k, v] : metrics) {
    if (!first) os_ << ',';
    first = false;
    write_str(os_, k);
    os_ << ':';
    write_num(os_, v);
  }
  os_ << '}';
  end_event();
}

}  // namespace slam::core
