#pragma once
// ROS-independent core, compiled by both the real node and the comparison runner.
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>

namespace vehicle_dynamics {
constexpr double gravity = 9.8;
constexpr double pi = 3.14159265358979323846;
struct Parameters {
  double mass = 1500, max_torque = 250, cd = 0.30, area = 2.2;
  double rolling_coeff = 0.015, wheel_radius = 0.32, final_drive = 4.06;
  double efficiency = 0.90, wheelbase = 2.65, a = 1.2;
  double cf = 80000, cr = 70000, idle_rpm = 800, max_rpm = 6200;
  std::vector<double> gears{3.55, 2.11, 1.42, 1.00, 0.78};
};
struct State {
  double t = 0, vx = 0, ax = 0, vy = 0, ay = 0, yaw_rate = 0;
  double heading = 0, position_x = 0, position_y = 0;
  int gear = 0;
  double engine_rpm = 0, engine_torque = 0;
};
class Model {
 public:
  explicit Model(Parameters parameters = Parameters{}, double initial_vx = 0)
      : p_(parameters) {
    for (double value : {p_.mass, p_.max_torque, p_.wheel_radius, p_.final_drive,
                         p_.efficiency, p_.wheelbase, p_.a, p_.cf, p_.cr,
                         p_.idle_rpm, p_.max_rpm}) {
      if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("positive finite parameters required");
    }
    if (p_.a >= p_.wheelbase || p_.efficiency > 1 || p_.idle_rpm >= p_.max_rpm || p_.gears.empty())
      throw std::invalid_argument("invalid vehicle geometry or powertrain");
    for (double value : {p_.cd, p_.area, p_.rolling_coeff, initial_vx})
      if (!std::isfinite(value) || value < 0) throw std::invalid_argument("non-negative parameters required");
    for (double gear : p_.gears)
      if (!std::isfinite(gear) || gear <= 0) throw std::invalid_argument("invalid gear ratio");
    const std::map<int, double> normalized{{800,.30},{1000,.50},{1500,.70},{2000,.86},
      {2500,.93},{3000,.97},{3500,1},{4000,.99},{4500,.95},{5000,.88},{5500,.78},{6000,.67}};
    for (auto [rpm, ratio] : normalized)
      if (rpm >= p_.idle_rpm && rpm <= p_.max_rpm) curve_[rpm] = ratio * p_.max_torque;
    for (double endpoint : {p_.idle_rpm, p_.max_rpm}) {
      auto closest = normalized.begin();
      for (auto it = normalized.begin(); it != normalized.end(); ++it)
        if (std::abs(it->first - endpoint) < std::abs(closest->first - endpoint)) closest = it;
      curve_[endpoint] = closest->second * p_.max_torque;
    }
    s_.vx = initial_vx;
  }
  const State& state() const { return s_; }
  const State& step(double dt, double throttle, double brake, double steer) {
    for (double value : {dt, throttle, brake, steer})
      if (!std::isfinite(value)) throw std::invalid_argument("finite controls required");
    if (dt <= 0 || dt > .1) throw std::invalid_argument("0 < dt <= 0.1 required");
    throttle = std::clamp(throttle, 0.0, 1.0);
    brake = std::clamp(brake, 0.0, 1.0);
    steer = std::clamp(steer, -.7, .7);
    const double old_vx = s_.vx, old_vy = s_.vy, old_heading = s_.heading;
    const int gear = s_.vx > 0 ? select_gear(s_.vx) : (throttle > 0 ? 1 : 0);
    const double ratio = gear ? p_.gears[gear - 1] * p_.final_drive : 0;
    const double rpm = s_.vx * ratio * 60 / (2 * pi * p_.wheel_radius);
    const double drive = throttle * torque(rpm) * ratio * p_.efficiency / p_.wheel_radius;
    const double resistance = p_.rolling_coeff * p_.mass * gravity + .5 * 1.225 * p_.cd * p_.area * s_.vx * s_.vx;
    double ax = (drive - resistance) / p_.mass - brake * gravity * .8;
    if (s_.vx == 0) ax = std::max(0.0, ax);
    const double moving_time = ax < 0 ? std::min(dt, -s_.vx / ax) : dt;
    const double distance = std::max(0.0, s_.vx * moving_time + .5 * ax * moving_time * moving_time);
    s_.vx = std::max(0.0, s_.vx + ax * dt);
    s_.ax = (s_.vx - old_vx) / dt;
    const double b = p_.wheelbase - p_.a;
    if (s_.vx < 1) {
      s_.yaw_rate = s_.vx * std::tan(steer) / p_.wheelbase;
      s_.vy = b * s_.yaw_rate;
      s_.ay = s_.vx * s_.yaw_rate;
    } else {
      const double iz = p_.mass * p_.a * b;
      const double rate = (p_.cf + p_.cr) / (p_.mass * s_.vx)
        + (p_.a * p_.a * p_.cf + b * b * p_.cr) / (iz * s_.vx);
      const int n = std::max(1, static_cast<int>(std::ceil(dt * rate / .2)));
      const double h = dt / n;
      for (int i = 0; i < n; ++i) {
        const auto [fyf, fyr] = forces(steer);
        const double dvy = (fyf + fyr) / p_.mass - s_.vx * s_.yaw_rate;
        const double dr = (p_.a * fyf - b * fyr) / iz;
        s_.vy += h * dvy;
        s_.yaw_rate += h * dr;
      }
      const auto [fyf, fyr] = forces(steer);
      s_.ay = (fyf + fyr) / p_.mass;
    }
    s_.heading += s_.yaw_rate * dt;
    const double heading = .5 * (old_heading + s_.heading);
    const double lateral_distance = .5 * (old_vy + s_.vy) * dt;
    s_.position_x += distance * std::cos(heading) - lateral_distance * std::sin(heading);
    s_.position_y += distance * std::sin(heading) + lateral_distance * std::cos(heading);
    s_.gear = s_.vx > 0 ? select_gear(s_.vx) : (throttle > 0 ? 1 : 0);
    s_.engine_rpm = s_.gear ? std::clamp(s_.vx * p_.gears[s_.gear-1] * p_.final_drive * 60
      / (2 * pi * p_.wheel_radius), p_.idle_rpm, p_.max_rpm) : 0;
    s_.engine_torque = throttle * torque(s_.engine_rpm);
    s_.t += dt;
    return s_;
  }
 private:
  Parameters p_;
  State s_;
  std::map<double, double> curve_;
  int select_gear(double vx) const {
    double best = std::numeric_limits<double>::infinity();
    int selected = 1;
    for (size_t i = 0; i < p_.gears.size(); ++i) {
      const double rpm = vx * p_.gears[i] * p_.final_drive * 60 / (2 * pi * p_.wheel_radius);
      if (rpm < p_.idle_rpm || rpm > p_.max_rpm) continue;
      if (std::abs(rpm - 2000) < best) { best = std::abs(rpm - 2000); selected = static_cast<int>(i) + 1; }
    }
    return selected;
  }
  double torque(double rpm) const {
    auto hi = curve_.lower_bound(rpm);
    if (hi == curve_.begin()) return hi->second;
    if (hi == curve_.end()) return curve_.rbegin()->second;
    auto lo = std::prev(hi);
    return lo->second + (hi->second - lo->second) * (rpm - lo->first) / (hi->first - lo->first);
  }
  std::pair<double, double> forces(double steer) const {
    return {-p_.cf * ((s_.vy + p_.a * s_.yaw_rate) / s_.vx - steer),
            -p_.cr * ((s_.vy - (p_.wheelbase - p_.a) * s_.yaw_rate) / s_.vx)};
  }
};
}  // namespace vehicle_dynamics
