# -*- coding: utf-8 -*-
"""
CAN 总线负载率模拟：当负载超过 70% 时演示丢包机制
不需要任何外部依赖，纯 Python 标准库
"""

import logging

logger = logging.getLogger(__name__)

# 1. 定义一个简化的 CAN 网络：~30 个 ECU，各自周期不同
#    (名称, ID, 周期_ms, 报文长度_字节)
ECUS = [
    # --- 动力域 (Powertrain) ---
    ("发动机",     0x0C9, 10,   8),
    ("变速箱TCU",  0x288, 10,   8),
    ("油门踏板",   0x0A0, 10,   4),
    # --- 底盘域 (Chassis) ---
    ("ABS制动",    0x210, 10,   8),
    ("ESP稳定",    0x150, 10,   8),
    ("EPB电子手刹",0x1A0, 20,   4),
    ("EPS转向",    0x0D0, 10,   8),
    ("胎压监测",   0x350, 100,  6),
    # --- 电池/电驱 ---
    ("BMS电池1",   0x180, 20,   8),
    ("BMS电池2",   0x181, 50,   8),
    ("电机控制器", 0x1F0, 10,   8),
    ("DC-DC",      0x1F1, 50,   4),
    ("车载充电机", 0x2A0, 100,  6),
    # --- 车身域 (Body) ---
    ("BCM车门",    0x320, 50,   8),
    ("灯光控制",   0x330, 100,  4),
    ("雨刮器",     0x340, 100,  3),
    ("座椅控制",   0x360, 200,  5),
    ("空调控制",   0x370, 100,  6),
    ("天窗",       0x380, 500,  3),
    # --- ADAS/智驾 ---
    ("前雷达",     0x3A0, 30,   8),
    ("后雷达",     0x3A1, 30,   8),
    ("摄像头L",    0x3B0, 20,   8),
    ("摄像头R",    0x3B1, 20,   8),
    ("毫米波雷达", 0x3C0, 20,   8),
    # --- 信息娱乐 ---
    ("仪表盘",     0x400, 50,   8),
    ("中控屏",     0x410, 100,  8),
    ("T-Box",      0x420, 500,  8),
    ("功放音响",   0x430, 200,  6),
    # --- 网关/诊断 ---
    ("中央网关",   0x700, 10,   8),
    ("OBD诊断",    0x7DF, 100,  8),
]


def frame_bits(data_bytes):
    """计算一帧 CAN 2.0A 标准帧的总位数（含位填充估计）。

    CAN 2.0A 帧结构：
      SOF(1) + ID(11) + RTR(1) + IDE(1) + r0(1) + DLC(4)
      + Data(N×8) + CRC(15) + CRC_Delim(1) + ACK(1) + ACK_Delim(1)
      + EOF(7) + IFS(3) = 47 + 8×N

    位填充规则：SOF 到 CRC（不含 CRC_Delim）之间，每连续 5 个相同 bit
    插入 1 个反 bit。这里用 (overhead + data_bits) // 10 做简化估算（≈10%）。
    """
    overhead = 47          # SOF+ID+RTR+IDE+r0+DLC+CRC+CRC_Delim+ACK+ACK_Delim+EOF+IFS
    data_bits = data_bytes * 8
    stuffing = (overhead + data_bits) // 10  # 位填充估算 ~10%
    return overhead + data_bits + stuffing


# 2. 负载计算（纯计算，不打印）


def simulate(ecus, bus_speed=250000, load_pct=70):
    """逐 ECU 累加计算 CAN 总线负载率，返回结构化结果。

    Args:
        ecus: list of (name, can_id, cycle_ms, data_bytes)
        bus_speed: CAN 波特率 (bps)，默认 250000
        load_pct: 负载率阈值 (%)

    Returns:
        dict: {
            "ecus":           所有 ECU 的负载明细,
            "bus_speed":      波特率,
            "total_load_pct": 总负载率,
            "overflow":       是否超过阈值,
            "trigger_idx":    触发超载的 ECU 索引（未触发时为 None）,
            "drop_plan":      丢包计划（未触发时为空列表）,
            "load_after_drop_pct": 丢包后负载率（未触发时为 None）,
        }
    """
    logger.info("simulate: 开始计算 — %d 个 ECU, 波特率=%dkbps, 阈值=%d%%",
                 len(ecus), bus_speed // 1000, load_pct)

    total_bits_per_sec = 0
    senders = []
    trigger_idx = None

    for idx, (name, cid, cycle_ms, data_len) in enumerate(ecus):
        frames_per_sec = 1000 / cycle_ms
        bits_for_this_ecu = frames_per_sec * frame_bits(data_len)
        total_bits_per_sec += bits_for_this_ecu
        senders.append({
            "name": name,
            "can_id": hex(cid),
            "cycle_ms": cycle_ms,
            "frames_per_sec": round(frames_per_sec, 2),
            "bits_per_sec": bits_for_this_ecu,
            "load_contrib_pct": round(bits_for_this_ecu / bus_speed * 100, 2),
        })
        bus_load = total_bits_per_sec / bus_speed * 100

        if trigger_idx is None and bus_load >= load_pct:
            trigger_idx = idx
            logger.warning("simulate: 触发超载 — 第 %d 个 ECU '%s' (0x%03X), "
                           "累计负载=%.1f%% (阈值=%d%%)",
                           idx + 1, name, cid, bus_load, load_pct)

    total_load = total_bits_per_sec / bus_speed * 100
    overflow = total_load >= load_pct

    logger.info("simulate: 累加完成 — 总比特率=%d bps, 总负载=%.1f%%, overflow=%s",
                 total_bits_per_sec, total_load, overflow)

    # 丢包计划：从列表末尾（低优先级 ECU）开始丢
    drop_plan = []
    load_after = None
    if overflow:
        logger.info("simulate: 进入丢包计划 — 从列表末尾开始逐 ECU 裁减")
        remaining = total_bits_per_sec
        for sender in reversed(senders):
            current_load = remaining / bus_speed * 100
            if current_load > load_pct:
                drop_plan.append(dict(sender, dropped=True))
                remaining -= sender["bits_per_sec"]
                logger.debug("simulate: 丢弃 '%s' — 丢弃后剩余负载=%.1f%%",
                             sender["name"], remaining / bus_speed * 100)
            else:
                drop_plan.append(dict(sender, dropped=False))
        drop_plan.reverse()  # 恢复原始顺序
        load_after = round(remaining / bus_speed * 100, 2)
        logger.info("simulate: 丢包计划完成 — 保留 %d 个 ECU, 丢弃后负载=%.1f%%",
                     sum(1 for s in drop_plan if not s["dropped"]), load_after)
    else:
        logger.info("simulate: 未触发超载，跳过丢包计划")

    return {
        "ecus": senders,
        "bus_speed": bus_speed,
        "total_load_pct": round(total_load, 2),
        "overflow": overflow,
        "trigger_idx": trigger_idx,
        "drop_plan": drop_plan,
        "load_after_drop_pct": load_after,
    }


# 3. 结果打印（纯显示，不做计算）


def print_simulation_result(result, scenario=""):
    """格式化打印 simulate() 的返回结果。

    Args:
        result: simulate() 返回的 dict
        scenario: 场景描述（如 "正常工况" / "故障注入"）
    """
    label = f"  [{scenario}]" if scenario else ""

    if not result["overflow"]:
        print(f"所有 {len(result['ecus'])} 个 ECU 全部添加，负载率仅 "
              f"{result['total_load_pct']:.1f}%，未触发丢包{label}")
        return

    trigger = result["trigger_idx"]
    print(f"{'='*60}")
    print(f"  触发丢包阈值：{result['total_load_pct']:.1f}% ≥ 70%  "
          f"(第 {trigger + 1} 个 ECU: {result['ecus'][trigger]['name']}){label}")
    print(f"{'='*60}")
    print()
    print(f"{'ECU名称':<12} {'周期ms':<8} {'帧/秒':<8} {'比特/秒':<10} {'负载贡献':<10}")
    print("-" * 60)

    for s in result["ecus"]:
        print(f"{s['name']:<10}  {s['cycle_ms']:>6}   {s['frames_per_sec']:>6.0f}   "
              f"{s['bits_per_sec']:>8.0f}    {s['load_contrib_pct']:>6.1f}%")

    print("-" * 60)
    print(f"{'总负载率':>46}   {result['total_load_pct']:>6.1f}%")
    print()

    # 丢包清单
    print("丢包模拟（列表末尾的低优先级 ECU 先被丢弃）：")
    print()
    dropped = False
    for s in result["drop_plan"]:
        if s["dropped"]:
            print(f"  ✗ {s['name']} ({s['cycle_ms']}ms) — 超出负载，丢弃")
            dropped = True
        else:
            print(f"  ✓ {s['name']} ({s['cycle_ms']}ms) — 保留")
    print()
    print(f"  丢弃后负载率：{result['load_after_drop_pct']:.1f}%")

    if not dropped:
        print("  (无需丢包)")


# 4. 运行模拟

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    BUS_SPEED = 250_000  # 250 kbps，中速 CAN 车身网络

    print(f"CAN 总线负载率模拟 — {BUS_SPEED // 1000} kbps 中速 CAN")
    print()
    print("【正常工况】30 个 ECU 正常工作：")
    result = simulate(ECUS, bus_speed=BUS_SPEED, load_pct=70)
    print_simulation_result(result, scenario="正常工况")

    print()
    print()
    print("【故障注入】车门控制器 BCM 电路短路，以 1ms 周期疯狂发包：")

    # 构造故障 ECU 列表（不改原 ECUS，避免污染模块级全局）
    faulty_ecus = list(ECUS)
    for i, (name, cid, cycle, dlen) in enumerate(faulty_ecus):
        if name == "BCM车门":
            faulty_ecus[i] = (name, cid, 1, dlen)   # 周期从 50ms → 1ms
            break

    result = simulate(faulty_ecus, bus_speed=BUS_SPEED, load_pct=70)
    print_simulation_result(result, scenario="故障注入")
