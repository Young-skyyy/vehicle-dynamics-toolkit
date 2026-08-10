#!/usr/bin/env python3
"""UDS 诊断测试客户端 — 验证所有 6 种 UDS 服务 (ISO 14229)

通过 ROS2 Service /uds/request 调用 C++ UDS 服务器
"""

import sys
import rclpy
from rclpy.node import Node
from vehicle_msgs.srv import UdsRequest


class UdsTestClient(Node):
    """UDS 诊断仪模拟：发送标准诊断请求，打印响应"""

    # UDS Service ID → 名称
    SID_NAMES = {
        0x10: "DiagnosticSessionControl",
        0x11: "ECU Reset",
        0x22: "ReadDataByIdentifier",
        0x27: "SecurityAccess",
        0x19: "ReadDTCInformation",
        0x3E: "TesterPresent",
    }

    def __init__(self, ecu: str = "EMS"):
        super().__init__('uds_test_client')
        self.ecu = ecu
        self.cli = self.create_client(UdsRequest, '/uds/request')
        while not self.cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('Waiting for /uds/request...')
        self.get_logger().info(f'Connected to UDS Server — testing ECU={ecu}')

    def send(self, sid: int, sub: int = 0, data: list = None,
             suppress: bool = False, label: str = "") -> dict:
        """发送一条 UDS 请求"""
        data = data or []
        req = UdsRequest.Request()
        req.ecu_name = self.ecu
        req.service_id = sid
        req.sub_function = sub
        req.data = data
        req.suppress_response = suppress

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if not future.done():
            self.get_logger().error(f'{label}: TIMEOUT')
            return {"error": "timeout"}

        res = future.result()
        status = "PASS" if res.success else f"NR 0x{res.nrc:02X}"
        msg = res.message
        dtcs = res.dtc_list

        print(f"\n{'='*55}")
        print(f"  {label}")
        print(f"  Request:  SID=0x{sid:02X} sub=0x{sub:02X}"
              f" data={[f'0x{b:02X}' for b in data]}")
        print(f"  Response: {status} | {msg}")
        if res.success and res.response_data:
            print(f"  Raw:      {res.response_data.hex(' ')}" if hasattr(res.response_data, 'hex') else f"  Raw:      {list(res.response_data)}")
        if dtcs:
            for d in dtcs:
                print(f"    DTC: {d}")

        return {
            "sid": sid,
            "sub": sub,
            "success": res.success,
            "nrc": res.nrc,
            "message": msg,
        }

    def run_full_test(self):
        """执行完整 UDS 诊断会话测试"""
        results = []

        # 1. 切换到 Extended Session
        r = self.send(0x10, 0x03, label="[0x10 03] Extended Session")
        results.append(r)

        # 2. Tester Present
        r = self.send(0x3E, 0x00, label="[0x3E] Tester Present")
        results.append(r)

        # 3. 读取发动机转速 DID 0x000C
        r = self.send(0x22, 0x00, data=[0x00, 0x0C],
                      label="[0x22 000C] Read RPM")
        results.append(r)

        # 4. 读取车速 DID 0x000D
        r = self.send(0x22, 0x00, data=[0x00, 0x0D],
                      label="[0x22 000D] Read Speed")
        results.append(r)

        # 5. SecurityAccess: requestSeed
        r = self.send(0x27, 0x01, label="[0x27 01] Request Seed")
        results.append(r)

        # 6. SecurityAccess: sendKey (需解析 seed 并计算 key)
        # 从 response 中提取 seed
        if r.get("success"):
            seed, key = None, None
            # 重新发送 requestSeed 获取原始响应
            req = UdsRequest.Request()
            req.ecu_name = self.ecu
            req.service_id = 0x27
            req.sub_function = 0x01
            req.data = []
            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done():
                resp_data = future.result().response_data
                if len(resp_data) >= 4:
                    seed = (resp_data[2] << 8) | resp_data[3]
                    key = seed ^ 0x5555
                    r = self.send(0x27, 0x02, data=[(key >> 8) & 0xFF, key & 0xFF],
                                  label=f"[0x27 02] SendKey (seed=0x{seed:04X})")
                    results.append(r)

        # 7. 读取 DTC (Status Mask = 0xFF)
        r = self.send(0x19, 0x02, data=[0xFF],
                      label="[0x19 02] Read DTC (all)")
        results.append(r)

        # 8. 切换到 Default Session
        r = self.send(0x10, 0x01, label="[0x10 01] Default Session")
        results.append(r)

        # 9. 测试负响应：Default Session 下发送 ECU Reset
        r = self.send(0x11, 0x01,
                      label="[0x11 01] ECU Reset (should fail)")
        results.append(r)

        # 10. 测试 SPR: SecurityAccess requestSeed with suppress
        r = self.send(0x27, 0x81, suppress=True,
                      label="[0x27 81] Request Seed (SPR)")
        results.append(r)

        # ── 打印摘要 ──
        print(f"\n{'='*55}")
        print(f"  UDS TEST SUMMARY — {self.ecu}")
        print(f"{'='*55}")
        passed = sum(1 for r in results if r.get("success"))
        failed = len(results) - passed
        for i, r in enumerate(results):
            sid = r.get("sid", 0)
            sub = r.get("sub", 0)
            name = self.SID_NAMES.get(sid, f"0x{sid:02X}")
            status = "PASS" if r.get("success") else f"NR 0x{r.get('nrc',0):02X}"
            print(f"  {i+1:2d}. 0x{sid:02X}/{sub:02X} {name:<28s} → {status}")
        print(f"  {'─'*45}")
        print(f"  Total: {len(results)} | Pass: {passed} | Fail: {failed}")
        print(f"{'='*55}\n")


def main():
    rclpy.init()
    ecu = sys.argv[1] if len(sys.argv) > 1 else "EMS"
    client = UdsTestClient(ecu)
    client.run_full_test()
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
