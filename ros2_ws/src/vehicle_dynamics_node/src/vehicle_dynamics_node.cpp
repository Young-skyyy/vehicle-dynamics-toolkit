// ROS2 adapter: all physics live in dynamics.hpp, also used by the parity runner.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "vehicle_msgs/msg/vehicle_state.hpp"
#include "vehicle_msgs/msg/vehicle_control.hpp"
#include "std_msgs/msg/float64.hpp"
#include "vehicle_dynamics_node/dynamics.hpp"

class VehicleDynamicsNode : public rclcpp::Node {
 public:
  VehicleDynamicsNode() : Node("vehicle_dynamics_node") {
    vehicle_dynamics::Parameters p;
    dt_ = declare_parameter("dt", 0.01);
    if (!std::isfinite(dt_) || dt_ <= 0 || dt_ > .1)
      throw std::invalid_argument("dt must be in (0, 0.1]");
    p.mass = declare_parameter("mass", p.mass);
    p.max_torque = declare_parameter("max_torque", p.max_torque);
    declare_parameter("max_power_kw", 140.0);  // metadata; torque curve supplies drive force
    p.cd = declare_parameter("cd", p.cd);
    p.area = declare_parameter("frontal_area", p.area);
    p.rolling_coeff = declare_parameter("rolling_coeff", p.rolling_coeff);
    p.wheel_radius = declare_parameter("wheel_radius", p.wheel_radius);
    p.final_drive = declare_parameter("final_drive", p.final_drive);
    p.efficiency = declare_parameter("trans_efficiency", p.efficiency);
    p.gears = declare_parameter("gear_ratios", p.gears);
    p.wheelbase = declare_parameter("wheelbase", p.wheelbase);
    p.a = declare_parameter("cg_to_front", p.a);
    p.cf = declare_parameter("cornering_stiffness_f", p.cf);
    p.cr = declare_parameter("cornering_stiffness_r", p.cr);
    p.idle_rpm = declare_parameter("idle_rpm", p.idle_rpm);
    p.max_rpm = declare_parameter("max_rpm", p.max_rpm);
    declare_parameter("publish_jitter", false);
    model_ = std::make_unique<vehicle_dynamics::Model>(p);
    state_pub_ = create_publisher<vehicle_msgs::msg::VehicleState>("/vehicle/state", 10);
    jitter_pub_ = create_publisher<std_msgs::msg::Float64>("/vehicle/jitter_us", 10);
    control_sub_ = create_subscription<vehicle_msgs::msg::VehicleControl>("/vehicle/control", 10,
      [this](vehicle_msgs::msg::VehicleControl::SharedPtr msg) {
        if (!std::isfinite(msg->throttle) || !std::isfinite(msg->brake) || !std::isfinite(msg->steer_angle)) {
          RCLCPP_WARN(get_logger(), "Ignored non-finite control input"); return;
        }
        throttle_ = std::clamp(msg->throttle, 0.0, 1.0);
        brake_ = std::clamp(msg->brake, 0.0, 1.0);
        steer_ = std::clamp(msg->steer_angle, -.7, .7);
      });
    throttle_sub_ = create_subscription<std_msgs::msg::Float64>("/vehicle/throttle", 10,
      [this](std_msgs::msg::Float64::SharedPtr msg) {
        if (std::isfinite(msg->data)) throttle_ = std::clamp(msg->data, 0.0, 1.0);
      });
    timer_ = create_wall_timer(std::chrono::duration<double>(dt_), [this]() { step(); });
    diagnostics_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      if (jitter_window_.empty()) return;
      if (get_parameter("publish_jitter").as_bool()) {
        std_msgs::msg::Float64 msg; msg.data = jitter_average_;
        jitter_pub_->publish(msg);
      }
      auto sorted = jitter_window_;
      std::sort(sorted.begin(), sorted.end());
      const double p99 = sorted[static_cast<size_t>((sorted.size() - 1) * .99)];
      RCLCPP_INFO(get_logger(), "JITTER STATS: avg=%.1fus max=%.1fus P99=%.1fus samples=%llu",
                  jitter_average_, jitter_max_, p99, static_cast<unsigned long long>(samples_));
    });
    RCLCPP_INFO(get_logger(), "Vehicle simulation at %.0f Hz; fixed dt, linear tires", 1 / dt_);
  }
 private:
  void step() {
    // Wall scheduling jitter must not use ROS simulation time.
    auto now = std::chrono::steady_clock::now();
    if (have_last_step_) {
      const double jitter = std::abs(std::chrono::duration<double>(now - last_step_).count() - dt_) * 1e6;
      jitter_average_ = .99 * jitter_average_ + .01 * jitter;
      jitter_max_ = std::max(jitter_max_, jitter);
      if (jitter_window_.size() < 100) jitter_window_.push_back(jitter);
      else jitter_window_[samples_ % 100] = jitter;
      ++samples_;
    }
    last_step_ = now;
    have_last_step_ = true;
    const auto& s = model_->step(dt_, throttle_, brake_, steer_);
    vehicle_msgs::msg::VehicleState msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "vehicle";
    msg.vx = s.vx; msg.ax = s.ax; msg.vy = s.vy; msg.ay = s.ay;
    msg.yaw_rate = s.yaw_rate; msg.steer_angle = steer_;
    msg.position_x = s.position_x; msg.position_y = s.position_y; msg.heading = s.heading;
    msg.engine_rpm = s.engine_rpm; msg.gear = static_cast<uint8_t>(s.gear);
    msg.engine_torque = s.engine_torque; msg.throttle = throttle_; msg.brake = brake_;
    state_pub_->publish(msg);
  }
  double dt_ = .01, throttle_ = 0, brake_ = 0, steer_ = 0;
  bool have_last_step_ = false;
  double jitter_average_ = 0, jitter_max_ = 0;
  uint64_t samples_ = 0;
  std::vector<double> jitter_window_;
  std::chrono::steady_clock::time_point last_step_;
  std::unique_ptr<vehicle_dynamics::Model> model_;
  rclcpp::Publisher<vehicle_msgs::msg::VehicleState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr jitter_pub_;
  rclcpp::Subscription<vehicle_msgs::msg::VehicleControl>::SharedPtr control_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr throttle_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleDynamicsNode>());
  rclcpp::shutdown();
  return 0;
}
