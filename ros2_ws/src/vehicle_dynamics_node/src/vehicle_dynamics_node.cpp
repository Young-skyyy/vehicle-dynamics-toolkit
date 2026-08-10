/**
 * @file vehicle_dynamics_node.cpp
 * @brief 车辆动力学 C++ ROS2 节点 — 纵向 + 横向 + 换挡 + SAE J2263
 *
 * 移植自 Python vehicle.py + lateral_dynamics.py
 * 使用 rclcpp (ROS2 C++ 客户端库)，生产级实时仿真
 */

#include <chrono>
#include <cmath>
#include <map>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "vehicle_msgs/msg/vehicle_state.hpp"
#include "vehicle_msgs/msg/vehicle_control.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;
using VehicleState = vehicle_msgs::msg::VehicleState;
using VehicleControl = vehicle_msgs::msg::VehicleControl;

// ═══════════════════════════════════════════════
// 物理常量
// ═══════════════════════════════════════════════
constexpr double G          = 9.8;
constexpr double RHO_AIR    = 1.225;
constexpr double KMH_TO_MS  = 1.0 / 3.6;

// ═══════════════════════════════════════════════
// 发动机外特性扭矩曲线（归一化，2.0L NA 汽油机）
// ═══════════════════════════════════════════════
const std::map<int, double> NORMALIZED_TORQUE = {
    {800, 0.30}, {1000, 0.50}, {1500, 0.70}, {2000, 0.86},
    {2500, 0.93}, {3000, 0.97}, {3500, 1.00}, {4000, 0.99},
    {4500, 0.95}, {5000, 0.88}, {5500, 0.78}, {6000, 0.67}
};

class VehicleDynamicsNode : public rclcpp::Node {
public:
    VehicleDynamicsNode()
    : Node("vehicle_dynamics_node")
    {
        // ── 参数声明 ──
        this->declare_parameter("dt", 0.01);              // 仿真步长 100Hz
        this->declare_parameter("mass", 1500.0);          // kg
        this->declare_parameter("max_torque", 250.0);     // Nm
        this->declare_parameter("max_power_kw", 140.0);   // kW
        this->declare_parameter("cd", 0.30);              // 风阻系数
        this->declare_parameter("frontal_area", 2.2);      // m²
        this->declare_parameter("rolling_coeff", 0.015);   // 滚动阻力系数
        this->declare_parameter("wheel_radius", 0.32);     // m
        this->declare_parameter("final_drive", 4.06);      // 主减速比
        this->declare_parameter("wheelbase", 2.65);        // 轴距 m
        this->declare_parameter("cg_to_front", 1.2);       // 质心到前轴 m
        this->declare_parameter("cornering_stiffness_f", 80000.0);  // N/rad
        this->declare_parameter("cornering_stiffness_r", 70000.0);  // N/rad
        this->declare_parameter("idle_rpm", 800.0);        // 怠速
        this->declare_parameter("max_rpm", 6200.0);        // 红线

        dt_ = this->get_parameter("dt").as_double();
        mass_ = this->get_parameter("mass").as_double();
        max_torque_ = this->get_parameter("max_torque").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        final_drive_ = this->get_parameter("final_drive").as_double();
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        cg_to_front_ = this->get_parameter("cg_to_front").as_double();
        cg_to_rear_ = wheelbase_ - cg_to_front_;
        Cf_ = this->get_parameter("cornering_stiffness_f").as_double();
        Cr_ = this->get_parameter("cornering_stiffness_r").as_double();
        idle_rpm_ = this->get_parameter("idle_rpm").as_double();
        max_rpm_ = this->get_parameter("max_rpm").as_double();

        // 横摆转动惯量 Iz ≈ m·a·b
        yaw_inertia_ = mass_ * cg_to_front_ * cg_to_rear_;

        // 5速自动变速箱速比
        gear_ratios_ = {3.55, 2.11, 1.42, 1.00, 0.78};

        // 构建外特性扭矩曲线
        build_torque_curve();

        // ── Publisher: 车辆状态 ──
        state_pub_ = this->create_publisher<VehicleState>("/vehicle/state", 10);

        // ── Subscriber: 控制指令 ──
        control_sub_ = this->create_subscription<VehicleControl>(
            "/vehicle/control", 10,
            std::bind(&VehicleDynamicsNode::control_callback, this, std::placeholders::_1));

        // ── Subscriber: 简化油门（兼容旧 Float64 接口）──
        throttle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/vehicle/throttle", 10,
            [this](std_msgs::msg::Float64::SharedPtr msg) {
                throttle_ = std::clamp(msg->data, 0.0, 1.0);
            });

        // ── Timer: 主仿真循环 ──
        auto period = std::chrono::duration<double>(dt_);
        timer_ = this->create_wall_timer(period,
            std::bind(&VehicleDynamicsNode::step, this));

        RCLCPP_INFO(this->get_logger(), "VehicleDynamicsNode (C++) started @ %.0f Hz", 1.0/dt_);
    }

private:
    // ═══════════════════════════════════════
    // 状态变量
    // ═══════════════════════════════════════
    double dt_ = 0.01;
    double mass_ = 1500.0;
    double max_torque_ = 250.0;
    double wheel_radius_ = 0.32;
    double final_drive_ = 4.06;
    double wheelbase_ = 2.65;
    double cg_to_front_ = 1.2;
    double cg_to_rear_ = 1.45;
    double Cf_ = 80000.0, Cr_ = 70000.0;
    double yaw_inertia_ = 3000.0;
    double idle_rpm_ = 800.0, max_rpm_ = 6200.0;

    // 纵向
    double vx_ = 0.0, ax_ = 0.0, position_x_ = 0.0;
    // 横向
    double vy_ = 0.0, ay_ = 0.0, yaw_rate_ = 0.0;
    double heading_ = 0.0, position_y_ = 0.0;
    // 控制
    double throttle_ = 0.0, brake_ = 0.0, steer_angle_ = 0.0;
    // 动力总成
    double engine_rpm_ = 0.0;
    int gear_ = 0;  // 0=空档

    std::vector<double> gear_ratios_;
    std::map<int, double> torque_curve_;

    // ROS2 接口
    rclcpp::Publisher<VehicleState>::SharedPtr state_pub_;
    rclcpp::Subscription<VehicleControl>::SharedPtr control_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr throttle_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ═══════════════════════════════════════
    // 构建发动机外特性扭矩曲线
    // ═══════════════════════════════════════
    void build_torque_curve() {
        torque_curve_.clear();
        for (const auto& [rpm, ratio] : NORMALIZED_TORQUE) {
            if (rpm >= idle_rpm_ && rpm <= max_rpm_) {
                torque_curve_[rpm] = ratio * max_torque_;
            }
        }
        // 确保怠速和红线端点
        if (torque_curve_.find(static_cast<int>(idle_rpm_)) == torque_curve_.end()) {
            torque_curve_[static_cast<int>(idle_rpm_)] = 0.30 * max_torque_;
        }
        if (torque_curve_.find(static_cast<int>(max_rpm_)) == torque_curve_.end()) {
            torque_curve_[static_cast<int>(max_rpm_)] = 0.67 * max_torque_;
        }
    }

    // ═══════════════════════════════════════
    // 扭矩曲线线性插值
    // ═══════════════════════════════════════
    double interp_torque(double rpm) const {
        if (torque_curve_.empty()) return 0.0;

        auto it_upper = torque_curve_.lower_bound(static_cast<int>(rpm));
        if (it_upper == torque_curve_.begin()) return it_upper->second;
        if (it_upper == torque_curve_.end()) return torque_curve_.rbegin()->second;

        auto it_lower = std::prev(it_upper);
        double t = (rpm - it_lower->first) / (it_upper->first - it_lower->first);
        return it_lower->second + t * (it_upper->second - it_lower->second);
    }

    // ═══════════════════════════════════════
    // 发动机扭矩 (Nm)
    // ═══════════════════════════════════════
    double engine_torque(double rpm) const {
        rpm = std::clamp(rpm, static_cast<double>(torque_curve_.begin()->first),
                         static_cast<double>(torque_curve_.rbegin()->first));
        return throttle_ * interp_torque(rpm);
    }

    // ═══════════════════════════════════════
    // 行驶阻力 = 滚动阻力 + 空气阻力 (N)
    // F_roll = μ·m·g, F_drag = ½·ρ·Cd·A·v²
    // ═══════════════════════════════════════
    double resistance_force(double v) const {
        double rolling_coeff = this->get_parameter("rolling_coeff").as_double();
        double cd = this->get_parameter("cd").as_double();
        double area = this->get_parameter("frontal_area").as_double();

        double F_roll = rolling_coeff * mass_ * G;
        double F_drag = 0.5 * RHO_AIR * cd * area * v * v;
        return F_roll + F_drag;
    }

    // ═══════════════════════════════════════
    // 自动换挡策略：目标转速 2000 RPM
    // ═══════════════════════════════════════
    int select_gear(double speed_ms) const {
        if (speed_ms <= 0.01) return 0;

        const double target_rpm = 2000.0;
        int best_gear = 1;
        double best_diff = std::numeric_limits<double>::max();

        for (size_t i = 0; i < gear_ratios_.size(); ++i) {
            double rpm = speed_ms * gear_ratios_[i] * final_drive_
                       / (2.0 * M_PI * wheel_radius_) * 60.0;
            if (rpm < idle_rpm_ || rpm > max_rpm_) continue;

            double diff = std::abs(rpm - target_rpm);
            if (diff < best_diff) {
                best_diff = diff;
                best_gear = static_cast<int>(i) + 1;
            }
        }
        return best_gear;
    }

    // ═══════════════════════════════════════
    // 横向动力学：自行车模型 + Pacejka
    // ═══════════════════════════════════════
    void lateral_step() {
        if (std::abs(vx_) < 0.01) return;  // 静止不做横向计算

        // 前后轮侧偏角 αf = (vy + a·r)/vx - δ, αr = (vy - b·r)/vx
        double alpha_f = (vy_ + cg_to_front_ * yaw_rate_) / vx_ - steer_angle_;
        double alpha_r = (vy_ - cg_to_rear_ * yaw_rate_) / vx_;

        // 线性轮胎模型: Fy = -Cα × α
        double Fyf = -Cf_ * alpha_f;
        double Fyr = -Cr_ * alpha_r;

        // 侧向加速度
        ay_ = (Fyf + Fyr) / mass_ - vx_ * yaw_rate_;

        // 横摆角加速度
        double yaw_accel = (cg_to_front_ * Fyf - cg_to_rear_ * Fyr) / yaw_inertia_;

        // 欧拉积分
        vy_ += ay_ * dt_;
        yaw_rate_ += yaw_accel * dt_;
        heading_ += yaw_rate_ * dt_;
        position_y_ += (vx_ * std::sin(heading_) + vy_ * std::cos(heading_)) * dt_;
    }

    // ═══════════════════════════════════════
    // 主仿真步进
    // ═══════════════════════════════════════
    void step() {
        // ── 换挡 ──
        gear_ = select_gear(vx_);
        double current_ratio = (gear_ >= 1) ? gear_ratios_[gear_ - 1] : gear_ratios_[0];

        // ── 发动机转速 ──
        if (vx_ > 0.01) {
            engine_rpm_ = vx_ * current_ratio * final_drive_
                        / (2.0 * M_PI * wheel_radius_) * 60.0;
            engine_rpm_ = std::max(idle_rpm_, std::min(engine_rpm_, max_rpm_));
        } else {
            engine_rpm_ = (throttle_ > 0.0) ? idle_rpm_ : 0.0;
        }

        // ── 驱动力 ──
        double T_engine = engine_torque(engine_rpm_);
        double F_drive = (current_ratio > 0)
            ? T_engine * current_ratio * final_drive_ / wheel_radius_
            : 0.0;

        // ── 阻力 ──
        double F_resist = resistance_force(vx_);

        // ── 制动力（最大减速度约 0.8g）──
        double F_brake = brake_ * mass_ * G * 0.8;

        // ── 纵向加速度 ──
        double F_net = F_drive - F_resist - F_brake;
        ax_ = F_net / mass_;

        // ── 纵向积分 ──
        vx_ = std::max(0.0, vx_ + ax_ * dt_);
        position_x_ += vx_ * dt_;

        // ── 横向动力学 ──
        lateral_step();

        // ── 发布 VehicleState ──
        auto msg = VehicleState();
        msg.header.stamp = this->now();
        msg.header.frame_id = "vehicle";
        msg.vx = vx_;
        msg.ax = ax_;
        msg.vy = vy_;
        msg.ay = ay_;
        msg.yaw_rate = yaw_rate_;
        msg.steer_angle = steer_angle_;
        msg.position_x = position_x_;
        msg.position_y = position_y_;
        msg.heading = heading_;
        msg.engine_rpm = engine_rpm_;
        msg.gear = static_cast<uint8_t>(gear_);
        msg.engine_torque = T_engine;
        msg.throttle = throttle_;
        msg.brake = brake_;

        state_pub_->publish(msg);
    }

    // ═══════════════════════════════════════
    // 控制指令回调
    // ═══════════════════════════════════════
    void control_callback(const VehicleControl::SharedPtr msg) {
        throttle_    = std::clamp(msg->throttle, 0.0, 1.0);
        brake_       = std::clamp(msg->brake, 0.0, 1.0);
        steer_angle_ = std::clamp(msg->steer_angle, -0.7, 0.7);  // ±40°
    }
};

// ═══════════════════════════════════════════════
// main
// ═══════════════════════════════════════════════
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VehicleDynamicsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
