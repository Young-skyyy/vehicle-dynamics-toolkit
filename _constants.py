# -*- coding: utf-8 -*-
"""
车辆动力学物理常量 —— 消除硬编码魔法数字
"""

# 重力加速度 (m/s²)
G = 9.8

# 海平面标准空气密度 (kg/m³)
RHO_AIR = 1.225

# km/h ↔ m/s 转换系数
KMH_TO_MS = 1.0 / 3.6
MS_TO_KMH = 3.6

# 默认滚动阻力系数（常量模型）
DEFAULT_ROLLING_COEFF = 0.015

# 每小时的秒数
SECONDS_PER_HOUR = 3600

# 每分钟的秒数
SECONDS_PER_MINUTE = 60

# 默认质心到前轴距离占轴距的比例（45%，典型前置前驱轿车）
DEFAULT_CG_FRONT_RATIO = 0.45
