/**
 * @file uds_server_node.cpp
 * @brief UDS (ISO 14229) 诊断协议栈 ROS2 服务节点
 *
 * 完整移植自 uds.py，支持 6 种 UDS 服务：
 *   0x10 DiagnosticSessionControl | 0x11 ECU Reset
 *   0x22 ReadDataByIdentifier     | 0x27 SecurityAccess
 *   0x19 ReadDTCInformation       | 0x3E TesterPresent
 *
 * @architecture
 *   ROS2 Service  /uds/request  (vehicle_msgs::srv::UdsRequest)
 *     → ECUDiagnosticServer::handle_request()
 *     → 正响应 或 负响应 (NRC)
 */

#include <chrono>
#include <cstdint>
#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "vehicle_msgs/srv/uds_request.hpp"

using namespace std::chrono_literals;

// ═══════════════════════════════════════════════
// ISO 14229 常量
// ═══════════════════════════════════════════════
constexpr uint8_t POS_RESP_OFFSET = 0x40;
constexpr uint8_t NEG_RESP_SID    = 0x7F;

// UDS Service IDs
constexpr uint8_t SID_SESSION   = 0x10;
constexpr uint8_t SID_RESET     = 0x11;
constexpr uint8_t SID_READ_DID  = 0x22;
constexpr uint8_t SID_SECURITY  = 0x27;
constexpr uint8_t SID_READ_DTC  = 0x19;
constexpr uint8_t SID_TESTER    = 0x3E;

// NRC: Negative Response Codes
constexpr uint8_t NRC_GENERAL_REJECT          = 0x10;
constexpr uint8_t NRC_SERVICE_NOT_SUPPORTED   = 0x11;
constexpr uint8_t NRC_SUB_FUNC_NOT_SUPPORTED  = 0x12;
constexpr uint8_t NRC_INCORRECT_MSG_LEN       = 0x13;
constexpr uint8_t NRC_CONDITIONS_NOT_CORRECT  = 0x22;
constexpr uint8_t NRC_REQUEST_OUT_OF_RANGE    = 0x31;
constexpr uint8_t NRC_SECURITY_DENIED         = 0x33;

// NRC 名称对照表
const std::map<uint8_t, std::string> NRC_NAMES = {
    {NRC_GENERAL_REJECT,         "General Reject"},
    {NRC_SERVICE_NOT_SUPPORTED,  "Service Not Supported"},
    {NRC_SUB_FUNC_NOT_SUPPORTED, "Sub-Function Not Supported"},
    {NRC_INCORRECT_MSG_LEN,      "Incorrect Message Length"},
    {NRC_CONDITIONS_NOT_CORRECT, "Conditions Not Correct"},
    {NRC_REQUEST_OUT_OF_RANGE,   "Request Out Of Range"},
    {NRC_SECURITY_DENIED,        "Security Access Denied"},
};

// ═══════════════════════════════════════════════
// DTC 数据库 (同 uds.py DTC_DATABASE)
// ═══════════════════════════════════════════════
struct DtcEntry {
    std::string code;
    std::string desc;
    std::string ecu;
    uint8_t     status;
};

const std::vector<DtcEntry> DTC_DB = {
    {"P0301", "1缸失火检测",           "EMS", 0x2B},  // testFailed+confirmed+MIL
    {"P0420", "催化转化器效率低于阈值",  "EMS", 0x04},  // pending
    {"U0100", "与 ECM/PCM 失去通讯",   "EMS", 0x0B},  // testFailed+confirmed
    {"C0035", "左前轮速传感器电路故障",  "ABS", 0x2B},
    {"B1A00", "环境光传感器故障",       "BCM", 0x04},
    {"P0A7F", "电池组劣化",            "BMS", 0x2B},
};

// ═══════════════════════════════════════════════
// 诊断会话状态机 (同 uds.py DiagnosticSession)
// ═══════════════════════════════════════════════
class DiagnosticSession {
public:
    std::string type    = "default";  // default | extended | programming
    int  security_level = 0;          // 0=locked, 1=unlocked
    int  pending_seed    = -1;        // 0x27 reqSeed 暂存的种子, -1=无
    rclcpp::Time last_tp;             // 上次 TesterPresent 时间

    bool check_timeout(const rclcpp::Time& now, double s3 = 5.0) const {
        if (type == "default") return false;
        return (now - last_tp).seconds() > s3;
    }

    void on_tester_present(const rclcpp::Time& now) { last_tp = now; }

    void goto_default() { type = "default"; security_level = 0; }
};

// ═══════════════════════════════════════════════
// DID 数据库 (同 uds.py _STANDARD_DIDS)
// ═══════════════════════════════════════════════
struct DidInfo {
    std::string name;
    int    len;       // 原始值字节数
    double scale  = 1.0;
    double offset = 0.0;
};

const std::map<int, DidInfo> DID_DB = {
    {0x000C, {"发动机转速",   2, 1.0,  0.0}},
    {0x000D, {"车速",        2, 0.01, 0.0}},
    {0x0005, {"冷却液温度",   1, 1.0, -40.0}},
    {0x0011, {"节气门位置",   1, 0.4,  0.0}},
    {0x004C, {"油门踏板位置", 1, 0.4,  0.0}},
    {0x000F, {"进气温度",     1, 1.0, -40.0}},
};

// ═══════════════════════════════════════════════
// ECU 诊断服务器 (同 uds.py ECUDiagnosticServer)
// ═══════════════════════════════════════════════
class ECUDiagnosticServer {
public:
    std::string ecu_name;
    std::map<int, double> did_values;  // DID → 物理值

    // Session 服务权限表
    const std::map<std::string, std::set<uint8_t>> SESSION_SERVICES = {
        {"default",     {SID_TESTER, SID_READ_DID, SID_READ_DTC, SID_SESSION}},
        {"extended",    {SID_TESTER, SID_READ_DID, SID_READ_DTC, SID_SESSION,
                         SID_RESET, SID_SECURITY}},
        {"programming", {SID_TESTER, SID_SESSION, SID_RESET, SID_SECURITY}},
    };

    DiagnosticSession session;

    explicit ECUDiagnosticServer(const std::string& name) : ecu_name(name) {
        // 初始化默认 DID 值
        if (name == "EMS") {
            did_values[0x000C] = 800;   // 发动机转速 800 RPM
            did_values[0x000D] = 0;     // 车速 0 km/h
            did_values[0x0005] = 90;    // 冷却液 90°C
            did_values[0x0011] = 15;    // 节气门 15%
            did_values[0x004C] = 20;    // 油门 20%
            did_values[0x000F] = 35;    // 进气 35°C
        }
    }

    void update_did(int did, double value) { did_values[did] = value; }

    // ── 主请求分发 ──
    struct UdsResult {
        bool success;
        std::vector<uint8_t> data;
        uint8_t nrc = 0;
        std::string message;
        std::vector<std::string> dtc_list;
    };

    UdsResult handle_request(uint8_t sid, uint8_t sub,
                              const std::vector<uint8_t>& payload,
                              bool suppress, const rclcpp::Time& now) {
        // 0x3E 始终可处理
        if (sid == SID_TESTER) {
            return handle_tester_present(now);
        }

        // S3 超时
        if (session.check_timeout(now)) {
            session.goto_default();
        }

        // 权限检查
        auto it = SESSION_SERVICES.find(session.type);
        if (it == SESSION_SERVICES.end() || !it->second.count(sid)) {
            return make_nr(sid, NRC_CONDITIONS_NOT_CORRECT);
        }

        // 分发
        switch (sid) {
            case SID_SESSION:  return handle_session_control(sub, suppress, now);
            case SID_READ_DID: return handle_read_did(payload);
            case SID_READ_DTC: return handle_read_dtc(sub, payload);
            case SID_RESET:     return handle_ecu_reset(sub, suppress);
            case SID_SECURITY: return handle_security_access(sub, payload, suppress);
        }

        return make_nr(sid, NRC_SERVICE_NOT_SUPPORTED);
    }

private:
    UdsResult make_nr(uint8_t sid, uint8_t nrc) {
        auto it = NRC_NAMES.find(nrc);
        return {false, {}, nrc,
                "NR: 0x7F " + std::to_string(sid) + " " +
                (it != NRC_NAMES.end() ? it->second : "Unknown NRC"),
                {}};
    }

    // ── 0x3E Tester Present ──
    UdsResult handle_tester_present(const rclcpp::Time& now) {
        session.on_tester_present(now);
        return {true, {SID_TESTER + POS_RESP_OFFSET}, 0,
                "TesterPresent ACK", {}};
    }

    // ── 0x10 Diagnostic Session Control ──
    UdsResult handle_session_control(uint8_t sub, bool suppress,
                                      const rclcpp::Time& now) {
        std::map<uint8_t, std::string> sm = {
            {0x01, "default"}, {0x02, "programming"}, {0x03, "extended"}
        };
        auto it = sm.find(sub);
        if (it == sm.end()) {
            return make_nr(SID_SESSION, NRC_SUB_FUNC_NOT_SUPPORTED);
        }
        session.type = it->second;
        session.on_tester_present(now);
        if (suppress) return {true, {}, 0, "Session switched (SPR)", {}};
        return {true, {SID_SESSION + POS_RESP_OFFSET, sub}, 0,
                "Session → " + it->second, {}};
    }

    // ── 0x22 Read Data By Identifier ──
    UdsResult handle_read_did(const std::vector<uint8_t>& payload) {
        if (payload.size() < 2) {
            return make_nr(SID_READ_DID, NRC_INCORRECT_MSG_LEN);
        }
        int did = (payload[0] << 8) | payload[1];
        auto dv = did_values.find(did);
        if (dv == did_values.end()) {
            return make_nr(SID_READ_DID, NRC_REQUEST_OUT_OF_RANGE);
        }
        auto di = DID_DB.find(did);
        DidInfo info = (di != DID_DB.end()) ? di->second
            : DidInfo{"DID_" + std::to_string(did), 2};

        // 物理值 → 原始值
        double raw_d = (dv->second - info.offset) / info.scale;
        int raw = static_cast<int>(raw_d);

        std::vector<uint8_t> resp = {
            SID_READ_DID + POS_RESP_OFFSET,
            static_cast<uint8_t>(payload[0]),
            static_cast<uint8_t>(payload[1])
        };
        for (int i = info.len - 1; i >= 0; --i) {
            resp.push_back(static_cast<uint8_t>((raw >> (8 * i)) & 0xFF));
        }
        return {true, resp, 0,
                info.name + "=" + std::to_string(static_cast<int>(dv->second)), {}};
    }

    // ── 0x19 Read DTC Information ──
    UdsResult handle_read_dtc(uint8_t sub, const std::vector<uint8_t>& payload) {
        if (sub == 0x02) {  // Report DTC by Status Mask
            uint8_t mask = payload.size() >= 1 ? payload[0] : 0xFF;

            std::vector<uint8_t> resp = {
                SID_READ_DTC + POS_RESP_OFFSET, 0x02,
                0x01,  // DTC Availability Mask (1 byte)
                0xFF   // all status bits available
            };

            std::vector<std::string> dtcs;
            for (const auto& d : DTC_DB) {
                if (d.ecu == ecu_name && (d.status & mask)) {
                    // DTC code → 3-byte number: e.g. P0301 → 0x0301
                    int dtc_num = std::stoi(d.code.substr(1), nullptr, 16);
                    resp.push_back((dtc_num >> 16) & 0xFF);
                    resp.push_back((dtc_num >> 8) & 0xFF);
                    resp.push_back(dtc_num & 0xFF);
                    resp.push_back(d.status);

                    std::string status_str = (d.status & 0x08) ? "confirmed" : "pending";
                    dtcs.push_back(d.code + " | " + d.desc + " | " + status_str);
                }
            }
            return {true, resp, 0, std::to_string(dtcs.size()) + " DTCs", dtcs};
        }
        else if (sub == 0x0A) {  // Report Supported DTCs
            int count = 0;
            for (const auto& d : DTC_DB) { if (d.ecu == ecu_name) count++; }
            return {true,
                    {SID_READ_DTC + POS_RESP_OFFSET, 0x0A,
                     static_cast<uint8_t>(count)},
                    0, std::to_string(count) + " supported DTCs", {}};
        }
        return make_nr(SID_READ_DTC, NRC_SUB_FUNC_NOT_SUPPORTED);
    }

    // ── 0x11 ECU Reset ──
    UdsResult handle_ecu_reset(uint8_t sub, bool suppress) {
        if (sub == 0x01) {
            if (suppress) return {true, {}, 0, "Hard Reset (SPR)", {}};
            return {true, {SID_RESET + POS_RESP_OFFSET, 0x01}, 0,
                    "Hard Reset", {}};
        }
        return make_nr(SID_RESET, NRC_SUB_FUNC_NOT_SUPPORTED);
    }

    // ── 0x27 Security Access ──
    UdsResult handle_security_access(uint8_t sub,
                                      const std::vector<uint8_t>& payload,
                                      bool suppress) {
        uint8_t actual = sub & 0x7F;  // strip SPR bit

        if (actual == 0x01) {  // requestSeed
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<int> dist(0, 0xFFFF);
            session.pending_seed = dist(gen);

            if (suppress) return {true, {}, 0, "Seed sent (SPR)", {}};
            std::vector<uint8_t> resp = {
                SID_SECURITY + POS_RESP_OFFSET, 0x01,
                static_cast<uint8_t>((session.pending_seed >> 8) & 0xFF),
                static_cast<uint8_t>(session.pending_seed & 0xFF)
            };
            return {true, resp, 0,
                    "Seed=0x" + std::to_string(session.pending_seed), {}};
        }
        else if (actual == 0x02) {  // sendKey
            if (session.pending_seed < 0) {
                return make_nr(SID_SECURITY, NRC_CONDITIONS_NOT_CORRECT);
            }
            if (payload.size() < 2) {
                return make_nr(SID_SECURITY, NRC_INCORRECT_MSG_LEN);
            }
            int key = (payload[0] << 8) | payload[1];
            int expected = session.pending_seed ^ 0x5555;

            if (key == expected) {
                session.security_level = 1;
                session.pending_seed = -1;
                if (suppress) return {true, {}, 0, "Unlocked (SPR)", {}};
                return {true, {SID_SECURITY + POS_RESP_OFFSET, 0x02}, 0,
                        "Security Unlocked", {}};
            }
            return make_nr(SID_SECURITY, NRC_REQUEST_OUT_OF_RANGE);
        }
        return make_nr(SID_SECURITY, NRC_SUB_FUNC_NOT_SUPPORTED);
    }
};

// ═══════════════════════════════════════════════
// ROS2 节点：UDS 诊断服务器
// ═══════════════════════════════════════════════
class UdsServerNode : public rclcpp::Node {
public:
    UdsServerNode() : Node("uds_server_node") {
        // 初始化 5 个 ECU
        ecus_["EMS"] = std::make_shared<ECUDiagnosticServer>("EMS");
        ecus_["BMS"] = std::make_shared<ECUDiagnosticServer>("BMS");
        ecus_["ABS"] = std::make_shared<ECUDiagnosticServer>("ABS");
        ecus_["TCU"] = std::make_shared<ECUDiagnosticServer>("TCU");
        ecus_["BCM"] = std::make_shared<ECUDiagnosticServer>("BCM");

        // ROS2 Service
        srv_ = this->create_service<vehicle_msgs::srv::UdsRequest>(
            "/uds/request",
            std::bind(&UdsServerNode::on_request, this,
                      std::placeholders::_1, std::placeholders::_2));

        // S3 超时定时器 (每秒检查)
        s3_timer_ = this->create_wall_timer(1s,
            [this]() {
                auto now = this->now();
                for (auto& [name, ecu] : ecus_) {
                    if (ecu->session.check_timeout(now)) {
                        ecu->session.goto_default();
                        RCLCPP_INFO(this->get_logger(),
                            "ECU %s: S3 timeout → default session", name.c_str());
                    }
                }
            });

        RCLCPP_INFO(this->get_logger(),
            "UDS Server started — 5 ECUs: EMS, BMS, ABS, TCU, BCM");
    }

private:
    rclcpp::Service<vehicle_msgs::srv::UdsRequest>::SharedPtr srv_;
    rclcpp::TimerBase::SharedPtr s3_timer_;
    std::map<std::string, std::shared_ptr<ECUDiagnosticServer>> ecus_;

    void on_request(
        const vehicle_msgs::srv::UdsRequest::Request::SharedPtr req,
        vehicle_msgs::srv::UdsRequest::Response::SharedPtr res) {

        auto it = ecus_.find(req->ecu_name);
        if (it == ecus_.end()) {
            res->success = false;
            res->nrc = NRC_GENERAL_REJECT;
            res->message = "Unknown ECU: " + req->ecu_name;
            return;
        }

        std::vector<uint8_t> payload(req->data.begin(), req->data.end());
        auto result = it->second->handle_request(
            req->service_id, req->sub_function, payload,
            req->suppress_response, this->now());

        res->success       = result.success;
        res->response_data = result.data;
        res->nrc           = result.nrc;
        res->message       = result.message;
        res->dtc_list      = result.dtc_list;

        RCLCPP_INFO(this->get_logger(),
            "UDS %s: SID=0x%02X sub=0x%02X → %s",
            req->ecu_name.c_str(), req->service_id, req->sub_function,
            result.success ? "PASS" : "NR");
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UdsServerNode>());
    rclcpp::shutdown();
    return 0;
}
