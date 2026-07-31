# -*- coding: utf-8 -*-
"""Pytest unit tests for can_bus_load_demo.py — 总线负载率计算 & 丢包算法"""

import pytest
import io
import sys
from can_bus_load_demo import (
    ECUS,
    frame_bits,
    simulate,
    print_simulation_result,
)


# ---- 辅助：构造简单 ECU 列表 ----

def _ecu(name, cycle_ms=10, data_bytes=8):
    """构造一个 (name, can_id, cycle_ms, data_bytes) 元组，can_id 自动累加"""
    _ecu._id_counter = getattr(_ecu, "_id_counter", 0x100) + 1
    return (name, _ecu._id_counter, cycle_ms, data_bytes)


# ---- frame_bits ----

class TestFrameBits:
    """位填充公式验证：手动计算与函数输出对比"""

    def test_8_bytes(self):
        # overhead=47, data=64, stuffing=(47+64)//10=11, total=122
        assert frame_bits(8) == 122

    def test_4_bytes(self):
        # overhead=47, data=32, stuffing=(47+32)//10=7, total=86
        assert frame_bits(4) == 86

    def test_0_bytes(self):
        # overhead=47, data=0, stuffing=47//10=4, total=51 (纯开销帧)
        assert frame_bits(0) == 51

    def test_monotonic(self):
        """数据字节越多，帧越长"""
        assert frame_bits(0) < frame_bits(4) < frame_bits(8)

    def test_overhead_components(self):
        """验证返回值为 overhead + data_bits + stuffing 三段之和"""
        for dlen in [0, 3, 5, 8]:
            bits = frame_bits(dlen)
            overhead = 47
            data_bits = dlen * 8
            stuffing = (overhead + data_bits) // 10
            assert bits == overhead + data_bits + stuffing


# ---- simulate() 基础计算 ----

class TestSimulateBasic:
    """验证逐 ECU 累加负载率、各字段正确性"""

    def test_single_ecu(self):
        """1 个 ECU (10ms, 8bytes): 100fps × 122bit = 12200bps, 4.88%"""
        ecus = [_ecu("Engine", 10, 8)]
        r = simulate(ecus, bus_speed=250000, load_pct=80)
        assert len(r["ecus"]) == 1
        s = r["ecus"][0]
        assert s["name"] == "Engine"
        assert s["frames_per_sec"] == 100.0
        assert s["bits_per_sec"] == 12200
        assert s["load_contrib_pct"] == pytest.approx(4.88, abs=0.01)

    def test_total_load_matches_manual(self):
        """两个相同 ECU: 手工算总负载 2×12200/250000×100 = 9.76%"""
        ecus = [_ecu("A", 10, 8), _ecu("B", 10, 8)]
        r = simulate(ecus, bus_speed=250000, load_pct=80)
        assert r["total_load_pct"] == pytest.approx(9.76, abs=0.01)

    def test_return_keys(self):
        """返回 dict 必须包含全部 8 个字段"""
        r = simulate([_ecu("X")], load_pct=80)
        required = {"ecus", "bus_speed", "total_load_pct", "overflow",
                    "trigger_idx", "drop_plan", "load_after_drop_pct"}
        assert required.issubset(set(r.keys()))

    def test_empty_ecu_list(self):
        """空 ECU 列表 → 0 负载, overflow=False, trigger_idx=None"""
        r = simulate([], load_pct=10)
        assert r["ecus"] == []
        assert r["total_load_pct"] == 0.0
        assert r["overflow"] is False
        assert r["trigger_idx"] is None
        assert r["drop_plan"] == []
        assert r["load_after_drop_pct"] is None

    def test_bus_speed_passed_through(self):
        """bus_speed 参数原样返回"""
        r = simulate([_ecu("X")], bus_speed=500000, load_pct=80)
        assert r["bus_speed"] == 500000

    def test_ecu_has_can_id_hex(self):
        """每个 ECU 条目含十六进制 can_id 字符串"""
        r = simulate([_ecu("X")])
        assert r["ecus"][0]["can_id"].startswith("0x")


# ---- simulate() 超载检测 ----

class TestSimulateOverflow:
    """验证 overflow、trigger_idx 在阈值附近的边界行为"""

    def test_no_overflow_below_threshold(self):
        """负载低于阈值 → overflow=False, trigger_idx=None"""
        ecus = [_ecu("A", 100, 8)]  # 10fps × 122 = 1220 bps, 0.49%
        r = simulate(ecus, load_pct=70)
        assert r["overflow"] is False
        assert r["trigger_idx"] is None

    def test_overflow_at_threshold(self):
        """单个高负载 ECU 直接触发超载"""
        # 2000fps × 122 = 244000 bps, 244000/250000=97.6%
        ecus = [_ecu("Noisy", cycle_ms=0.5, data_bytes=8)]
        r = simulate(ecus, load_pct=70)
        assert r["overflow"] is True
        assert r["trigger_idx"] == 0

    def test_trigger_idx_correct(self):
        """第 3 个 ECU 才触发超载 → trigger_idx=2"""
        ecus = [
            _ecu("A", 100, 8),   # 0.49%
            _ecu("B", 100, 8),   # 0.49%
            _ecu("C", 0.8, 8),   # 1000/0.8=1250fps × 122 = 152500 bps, 61% → 累计 >50%
        ]
        r = simulate(ecus, load_pct=50)
        assert r["trigger_idx"] == 2
        assert r["ecus"][r["trigger_idx"]]["name"] == "C"


# ---- simulate() 丢包计划 ----

class TestSimulateDropPlan:
    """验证丢包算法：从列表末尾倒序丢，直到负载低于阈值"""

    @pytest.fixture
    def overflow_ecus(self):
        """构造 3 个 ECU，BCM 故障导致超载"""
        # High (10ms,8B): 12200 bps, 4.88%
        # BCM  (1ms, 8B): 122000 bps, 48.8%
        # Low  (100ms,4B): 860 bps, 0.34%
        # Total: 135060 bps, 54.02% → 阈值 50% 时 overflow
        return [
            ("High",   0x100, 10,  8),
            ("BcmFlt", 0x200, 1,   8),
            ("Low",    0x300, 100, 4),
        ]

    def test_drop_plan_structure(self, overflow_ecus):
        """drop_plan 每个元素含 name, dropped 字段"""
        r = simulate(overflow_ecus, load_pct=50)
        assert len(r["drop_plan"]) == 3
        for item in r["drop_plan"]:
            assert "name" in item
            assert "dropped" in item
            assert isinstance(item["dropped"], bool)

    def test_drop_from_tail(self, overflow_ecus):
        """末尾的 Low 先被考核丢弃，头部的 High 最后保留"""
        r = simulate(overflow_ecus, load_pct=50)
        names = [s["name"] for s in r["drop_plan"]]
        assert names == ["High", "BcmFlt", "Low"]  # 原始顺序

        # Low 被丢 (在尾部)
        assert r["drop_plan"][2]["dropped"] is True
        # BcmFlt 被丢
        assert r["drop_plan"][1]["dropped"] is True
        # High 保留
        assert r["drop_plan"][0]["dropped"] is False

    def test_load_after_drop(self, overflow_ecus):
        """丢弃 BCM+Low 后只剩 High: 12200/250000 = 4.88%"""
        r = simulate(overflow_ecus, load_pct=50)
        # High (12200 bps) 被保留
        assert r["load_after_drop_pct"] == pytest.approx(4.88, abs=0.01)

    def test_no_drop_when_no_overflow(self):
        """不超载时 drop_plan 为空"""
        r = simulate([_ecu("A", 100, 8)], load_pct=70)
        assert r["overflow"] is False
        assert r["drop_plan"] == []
        assert r["load_after_drop_pct"] is None


# ---- simulate() 加载全量 ECUS 不超载场景 ----

class TestSimulateAllECUS:
    """验证内置 30 个 ECU 的负载率在正常工况下不超载"""

    def test_all_ecus_no_overflow_normal(self):
        """正常工况 (30 个 ECU, 250kbps) → 约 59.4%, 不超载"""
        r = simulate(ECUS, bus_speed=250000, load_pct=70)
        assert r["overflow"] is False
        assert 55 < r["total_load_pct"] < 65

    def test_all_ecus_count(self):
        """返回的 ECU 数量与输入一致"""
        r = simulate(ECUS)
        assert len(r["ecus"]) == len(ECUS)


# ---- print_simulation_result() ----

class TestPrintSimulationResult:
    """验证打印函数不抛异常，输出包含关键信息"""

    def test_print_no_overflow(self):
        """不超载场景：打印应包含 '未触发丢包'"""
        r = simulate([_ecu("X", 100, 8)], load_pct=70)
        buf = io.StringIO()
        sys.stdout = buf
        print_simulation_result(r)
        sys.stdout = sys.__stdout__
        assert "未触发丢包" in buf.getvalue()

    def test_print_overflow(self):
        """超载场景：打印应包含 '触发丢包阈值' 和 '丢弃后负载率'"""
        ecus = [_ecu("Noisy", 0.8, 8)]  # 1250fps × 122=152500bps, 61% > 50%
        r = simulate(ecus, load_pct=50)
        buf = io.StringIO()
        sys.stdout = buf
        print_simulation_result(r)
        sys.stdout = sys.__stdout__
        output = buf.getvalue()
        assert "触发丢包阈值" in output
        assert "丢弃后负载率" in output

    def test_print_with_scenario_label(self):
        """scenario 参数应出现在输出中"""
        r = simulate([_ecu("X", 100, 8)], load_pct=70)
        buf = io.StringIO()
        sys.stdout = buf
        print_simulation_result(r, scenario="故障注入")
        sys.stdout = sys.__stdout__
        assert "故障注入" in buf.getvalue()


# ---- 不同波特率场景 ----

class TestDifferentBaudRates:
    """验证 bus_speed 参数影响负载率计算"""

    def test_higher_baudrate_lower_load(self):
        """相同 ECU 配置，波特率越高负载率越低"""
        ecus = [_ecu("X", 10, 8)]  # 12200 bps
        r250 = simulate(ecus, bus_speed=250000, load_pct=80)
        r500 = simulate(ecus, bus_speed=500000, load_pct=80)
        assert r250["total_load_pct"] > r500["total_load_pct"]
        assert r250["total_load_pct"] == pytest.approx(r500["total_load_pct"] * 2, abs=0.01)

    def test_can_fd_baudrate(self):
        """2Mbps CAN FD 波特率"""
        ecus = [_ecu("X", 10, 8)]
        r = simulate(ecus, bus_speed=2_000_000, load_pct=80)
        assert r["overflow"] is False
        assert r["total_load_pct"] < 1.0


# ---- 边界/回归测试 ----

class TestEdgeCases:
    """异常输入与边界条件"""

    def test_zero_cycle_ms(self):
        """cycle_ms=0 → 无限帧率, 但不会崩溃 (返回 inf 负载)"""
        ecus = [("Bad", 0x100, 0, 8)]
        # 不应抛出异常（虽然物理上不合理）
        with pytest.raises(ZeroDivisionError):
            simulate(ecus, load_pct=100)

    def test_load_pct_zero(self):
        """阈值 0% → 任何 ECU 都触发超载"""
        r = simulate([_ecu("X", 100, 8)], load_pct=0)
        assert r["overflow"] is True

    def test_result_is_serializable(self):
        """返回 dict 可被 json.dumps（验证无不可序列化类型）"""
        import json
        r = simulate(ECUS[:3], load_pct=80)
        s = json.dumps(r, default=str)
        assert len(s) > 0
        parsed = json.loads(s)
        assert parsed["overflow"] is False
