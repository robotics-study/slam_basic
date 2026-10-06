#include "slam/core/geometry.hpp"

#include <cmath>

#include "slam/core/libm.hpp"

namespace slam::core {

double wrap(double a) {
  return a - 2.0 * kPi * std::floor((a + kPi) / (2.0 * kPi));
}

Pose pose_compose(const Pose& p, const Twist& t) {
  // libm_cos/libm_sin (NOT std): the sin+cos pair on one argument is exactly what
  // Apple clang folds into __sincos_stret — see core/libm.hpp.
  double c = libm_cos(p.theta);
  double s = libm_sin(p.theta);
  return Pose{p.x + c * t.dx - s * t.dy, p.y + s * t.dx + c * t.dy, wrap(p.theta + t.dtheta)};
}

Twist pose_minus(const Pose& a, const Pose& b) {
  double c = libm_cos(a.theta);
  double s = libm_sin(a.theta);
  double dx = b.x - a.x;
  double dy = b.y - a.y;
  return Twist{c * dx + s * dy, -s * dx + c * dy, wrap(b.theta - a.theta)};
}

Point world_to_robot(const Point& e, const Pose& p) {
  double ex = e.x - p.x;
  double ey = e.y - p.y;
  double c = libm_cos(p.theta);
  double s = libm_sin(p.theta);
  return Point{c * ex + s * ey, -s * ex + c * ey};
}

Point robot_to_world(const Point& z, const Pose& p) {
  double c = libm_cos(p.theta);
  double s = libm_sin(p.theta);
  return Point{p.x + c * z.x - s * z.y, p.y + s * z.x + c * z.y};
}

namespace {
// Cross product (b-a) x (c-a); its sign is the orientation of a->b->c.
double orient(const Point& a, const Point& b, const Point& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// p is collinear with a->b (caller checked orient == 0): is it on the segment?
bool on_segment(const Point& a, const Point& b, const Point& p) {
  return std::min(a.x, b.x) <= p.x && p.x <= std::max(a.x, b.x) &&
         std::min(a.y, b.y) <= p.y && p.y <= std::max(a.y, b.y);
}
}  // namespace

bool segments_intersect(const Point& a, const Point& b, const Point& c, const Point& d) {
  double o1 = orient(a, b, c);
  double o2 = orient(a, b, d);
  double o3 = orient(c, d, a);
  double o4 = orient(c, d, b);
  if (o1 == 0.0 && on_segment(a, b, c)) return true;
  if (o2 == 0.0 && on_segment(a, b, d)) return true;
  if (o3 == 0.0 && on_segment(c, d, a)) return true;
  if (o4 == 0.0 && on_segment(c, d, b)) return true;
  return (o1 > 0.0) != (o2 > 0.0) && (o3 > 0.0) != (o4 > 0.0);
}

bool point_in_rect(const Point& p, const Rect& rect) {
  return rect.x_lo <= p.x && p.x <= rect.x_hi && rect.y_lo <= p.y && p.y <= rect.y_hi;
}

bool segment_intersects_rect(const Point& a, const Point& b, const Rect& r) {
  if (point_in_rect(a, r) || point_in_rect(b, r)) return true;
  Point c0{r.x_lo, r.y_lo}, c1{r.x_hi, r.y_lo}, c2{r.x_hi, r.y_hi}, c3{r.x_lo, r.y_hi};
  const std::pair<Point, Point> edges[4] = {{c0, c1}, {c1, c2}, {c2, c3}, {c3, c0}};
  for (const auto& e : edges) {
    if (segments_intersect(a, b, e.first, e.second)) return true;
  }
  return false;
}

}  // namespace slam::core
