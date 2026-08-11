/**
 * @file vehicle_dynamics_node.cpp
 * @brief 车辆动力学 C++ ROS2 节点 — 纵向 + 横向 + 换挡 + SAE J2263
 *
 * 移植自 Python vehicle.py + lateral_dynamics.py
 * 使用 rclcpp (ROS2 C++ 客户端库)
 *
 * 轮胎模型: 线性（Fy = -Cα·α）与 Pacejka 魔术公式可切换
 *   Pacejka: Fy(α) = D·sin(C·arctan(Bα − E(Bα − arctan(Bα))))
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
        this->declare_parameter("publish_jitter", false);

        // ── 轮胎模型 ──
        this->declare_parameter("tire_model", "linear");   // "linear" 或 "pacejka"
        // Pacejka 魔术公式参数 (前轴)
        this->declare_parameter("pacejka_B_f", 10.0);      // 刚度因子 (1/rad)
        this->declare_parameter("pacejka_C_f", 1.3);        // 形状因子
        this->declare_parameter("pacejka_D_f", 7644.0);     // 峰值因子 ≈ 前轴轴荷 (N)
        this->declare_parameter("pacejka_E_f", -1.5);       // 曲率因子
        // Pacejka 魔术公式参数 (后轴)
        this->declare_parameter("pacejka_B_r", 10.0);
        this->declare_parameter("pacejka_C_r", 1.3);
        this->declare_parameter("pacejka_D_r", 7056.0);     // 峰值因子 ≈ 后轴轴荷 (N)
        this->declare_parameter("pacejka_E_r", -1.5);

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

        // 轮胎模型参数
        tire_model_ = this->get_parameter("tire_model").as_string();
        pbj_B_f_ = this->get_parameter("pacejka_B_f").as_double();
        pbj_C_f_ = this->get_parameter("pacejka_C_f").as_double();
        pbj_D_f_ = this->get_parameter("pacejka_D_f").as_double();
        pbj_E_f_ = this->get_parameter("pacejka_E_f").as_double();
        pbj_B_r_ = this->get_parameter("pacejka_B_r").as_double();
        pbj_C_r_ = this->get_parameter("pacejka_C_r").as_double();
        pbj_D_r_ = this->get_parameter("pacejka_D_r").as_double();
        pbj_E_r_ = this->get_parameter("pacejka_E_r").as_double();

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

        // ── Publisher: Jitter 统计 ──
        jitter_pub_ = this->create_publisher<std_msgs::msg::Float64>("/vehicle/jitter_us", 10);

        // ── Timer: 主仿真循环 ──
        auto period = std::chrono::duration<double>(dt_);
        timer_ = this->create_wall_timer(period,
            std::bind(&VehicleDynamicsNode::step, this));

        // ── Timer: 每秒打印诊断信息 ──
        diag_timer_ = this->create_wall_timer(1s,
            [this]() {
                RCLCPP_INFO(this->get_logger(),
                    "t=%.0fs vx=%.3f ax=%.3f gear=%d rpm=%.0f Tq=%.1f pos=%.1f jitter=%.1fus",
                    step_count_ * dt_, vx_, ax_, gear_, engine_rpm_,
                    engine_torque(engine_rpm_), position_x_, jitter_avg_us_);

                if (this->get_parameter("publish_jitter").as_bool() && jitter_sample_count_ > 0) {
                    auto jitter_msg = std_msgs::msg::Float64();
                    jitter_msg.data = jitter_avg_us_;
                    jitter_pub_->publish(jitter_msg);
                }

                if (step_count_ % 1000 == 0 && jitter_sample_count_ > 0) {  // every 10s
                    int valid_samples = std::min(100, (int)jitter_sample_count_);
                    std::vector<double> sorted(jitter_window_, jitter_window_ + valid_samples);
                    std::sort(sorted.begin(), sorted.end());
                    double p99 = sorted[(int)(valid_samples * 0.99)];

                    RCLCPP_INFO(this->get_logger(),
                        "JITTER STATS: avg=%.1fus max=%.1fus P99=%.1fus samples=%lu",
                        jitter_avg_us_, jitter_max_us_, p99, jitter_sample_count_);
                }
            });

        RCLCPP_INFO(this->get_logger(), "VehicleDynamicsNode (C++) started @ %.0f Hz (tire: %s, jitter: %s)",
            1.0/dt_, tire_model_.c_str(), this->get_parameter("publish_jitter").as_bool() ? "ON" : "OFF");
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

    // 轮胎模型
    std::string tire_model_ = "linear";
    double pbj_B_f_ = 10.0, pbj_C_f_ = 1.3, pbj_D_f_ = 7644.0, pbj_E_f_ = -1.5;
    double pbj_B_r_ = 10.0, pbj_C_r_ = 1.3, pbj_D_r_ = 7056.0, pbj_E_r_ = -1.5;

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
    rclcpp::TimerBase::SharedPtr diag_timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr jitter_pub_;
    uint64_t step_count_ = 0;

    // Jitter monitoring
    double last_step_time_ = 0.0;           // 上次 step() 时间戳
    int64_t last_step_time_ns_ = 0;
    double jitter_max_us_ = 0.0;            // 最大 jitter (us)
    double jitter_avg_us_ = 0.0;            // 平均 jitter (us)
    uint64_t jitter_sample_count_ = 0;      // 采样计数
    double jitter_window_[100] = {0};       // 最近 100 次 jitter
    int jitter_window_idx_ = 0;

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
    // Pacejka 魔术公式 — 纯侧偏轮胎侧向力 (N)
    // Fy(α) = D · sin(C · arctan(Bα − E·(Bα − arctan(Bα))))
    // ═══════════════════════════════════════
    double pacejka_lateral_force(double B, double C, double D, double E,
                                  double alpha) const {
        double Bx = B * alpha;
        return D * std::sin(C * std::atan(Bx - E * (Bx - std::atan(Bx))));
    }

    // ═══════════════════════════════════════
    // 横向动力学：自行车模型 + 线性/Pacejka 可切换
    // ═══════════════════════════════════════
    void lateral_step() {
        if (std::abs(vx_) < 0.01) return;  // 静止不做横向计算

        // 前后轮侧偏角 αf = (vy + a·r)/vx - δ, αr = (vy - b·r)/vx
        double alpha_f = (vy_ + cg_to_front_ * yaw_rate_) / vx_ - steer_angle_;
        double alpha_r = (vy_ - cg_to_rear_ * yaw_rate_) / vx_;

        // 侧向力 — 根据 tire_model 选择线性或 Pacejka
        double Fyf, Fyr;
        if (tire_model_ == "pacejka") {
            // Pacejka 公式输出 magnitude，负号对应 SAE 坐标系
            Fyf = -pacejka_lateral_force(pbj_B_f_, pbj_C_f_, pbj_D_f_, pbj_E_f_, alpha_f);
            Fyr = -pacejka_lateral_force(pbj_B_r_, pbj_C_r_, pbj_D_r_, pbj_E_r_, alpha_r);
        } else {
            // 线性轮胎模型: Fy = -Cα × α
            Fyf = -Cf_ * alpha_f;
            Fyr = -Cr_ * alpha_r;
        }

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
        // ── Jitter 计时 ──
        auto now = this->now();
        if (last_step_time_ns_ > 0) {
            double expected_dt = dt_;  // seconds
            double actual_dt = (now.nanoseconds() - last_step_time_ns_) * 1e-9;
            double jitter_us = std::abs(actual_dt - expected_dt) * 1e6;

            // 指数移动平均
            jitter_avg_us_ = 0.99 * jitter_avg_us_ + 0.01 * jitter_us;
            if (jitter_us > jitter_max_us_) jitter_max_us_ = jitter_us;

            // 滑动窗口
            jitter_window_[jitter_window_idx_ % 100] = jitter_us;
            jitter_window_idx_++;
            jitter_sample_count_++;
        }
        last_step_time_ns_ = now.nanoseconds();

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

        step_count_++;
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
