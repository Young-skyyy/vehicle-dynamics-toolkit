# -*- coding: utf-8 -*-
"""
matplotlib 中文显示工具 —— 按平台检测可用中文字体，不再靠 try-luck。
"""

import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import platform


# 各平台常用中文字体（按优先级排序）
_CHINESE_FONT_CANDIDATES = {
    "Windows": ["Microsoft YaHei", "SimHei", "KaiTi", "FangSong"],
    "Darwin":  ["PingFang SC", "Heiti SC", "STHeiti", "Songti SC"],
    "Linux":   ["WenQuanYi Micro Hei", "WenQuanYi Zen Hei", "Noto Sans CJK SC",
                "Droid Sans Fallback", "AR PL UMing CN"],
}

_HAS_CHINESE_FONT = None


def setup_chinese_font() -> tuple[bool, str | None]:
    """检测并设置中文字体，返回 (是否成功, 使用的字体名)。

    如果系统无中文字体，打印 Warning 并启用英文标签模式。
    """
    global _HAS_CHINESE_FONT
    if _HAS_CHINESE_FONT is not None:
        return _HAS_CHINESE_FONT

    system = platform.system()
    candidates = _CHINESE_FONT_CANDIDATES.get(system, [])

    # 拿到系统实际安装的所有字体名
    installed = {f.name for f in fm.fontManager.ttflist}

    chosen = None
    for name in candidates:
        if name in installed:
            chosen = name
            break

    if chosen:
        plt.rcParams["font.sans-serif"] = [chosen]
        plt.rcParams["axes.unicode_minus"] = False
        _HAS_CHINESE_FONT = (True, chosen)
    else:
        import warnings
        warnings.warn(
            f"当前系统 ({system}) 未检测到中文字体，图表中文将显示为英文。\n"
            f"  已检测字体候选: {candidates}\n"
            f"  Linux 用户可执行: sudo apt install fonts-wqy-microhei"
        )
        # 不设字体，matplotlib 用默认英文字体
        _HAS_CHINESE_FONT = (False, None)

    return _HAS_CHINESE_FONT


def get_label(key: str) -> str:
    """根据中文字体是否可用，返回中文或英文 label。

    Args:
        key: 标签键名，如 "发动机转速"

    Returns:
        str: 中文 label（如果有字体）或英文 fallback
    """
    has_font, _ = setup_chinese_font()
    if has_font:
        return key
    # 英文 fallback 映射表
    _EN_FALLBACK = {
        "发动机转速 (RPM)":        "Engine Speed (RPM)",
        "扭矩负荷比 (%)":          "Torque Load (%)",
        "BSFC 万有特性 Map + 等功率线": "BSFC Map + Iso-Power Lines",
        "发动机 BSFC 万有特性 Map (2.0L 汽油机)": "BSFC Map (2.0L Gasoline)",
        "怠速":                    "Idle",
        "经济巡航":                "Cruise",
        "全油门":                  "WOT",
        "全油门加速":              "WOT Accel",
        "稳态转向响应 (方向盘 3°)": "Steady-State Cornering (3° steer)",
        "横摆角速度":              "Yaw Rate",
        "侧向加速度":              "Lat. Acceleration",
        "车速 (km/h)":             "Speed (km/h)",
        "横摆角速度 (deg/s)":      "Yaw Rate (deg/s)",
        "侧向加速度 (g)":          "Lat. Acceleration (g)",
        "转弯半径 vs 车速 + 中性转向参考": "Turn Radius vs Speed + Neutral Ref",
        "转弯半径 (m)":            "Turn Radius (m)",
        "阶跃转向瞬态响应 (80km/h, 3°)": "Step Steer Response (80km/h, 3°)",
        "时间 (s)":                "Time (s)",
        "中性转向(参考)":          "Neutral Steer (ref)",
        "不足转向特征":            "Understeer",
        "高效率":                  "High Eff.",
        "高油耗":                  "High Cons.",
        "后车速度":                "Ego Speed",
        "前车速度":                "Lead Speed",
        "间距 (m)":                "Gap (m)",
        "IDM 自适应巡航 (ACC) 响应": "IDM ACC Response",
    }
    return _EN_FALLBACK.get(key, key)
