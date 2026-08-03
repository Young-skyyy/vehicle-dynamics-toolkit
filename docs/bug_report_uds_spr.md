# 缺陷报告示例 — BUG-2026-001

> 用途说明：本文件是项目「测试作品集」的一部分，展示缺陷管理闭环（发现 → 复现 → 根因 → 修复 → 验证 → 回归评估）。缺陷内容取自项目真实提交 `c64b1da`，非虚构。

## 一、缺陷摘要

| 字段 | 内容 |
|---|---|
| 缺陷编号 | BUG-2026-001 |
| 缺陷标题 | UDS 0x27 SecurityAccess：Suppress Positive Response 分支为死代码，0x81/0x82 子功能无法正确响应 |
| 严重程度 | Major（中）—— 协议符合性缺陷，当前演示流程未触发 |
| 优先级 | P1 —— 协议行为错误，应尽快修复 |
| 状态 | Closed（已修复并验证） |
| 所属模块 | `uds.py` → `ECUDiagnosticServer._handle_security_access()` |
| 影响版本 | v0.3.0 |
| 发现方式 | 代码评审（Code Review） |
| 发现人 / 日期 | 项目作者 / 2026-08-03 |
| 修复提交 | `c64b1da` |
| 修复版本 | v0.3.1（待发布） |

## 二、缺陷描述

ISO 14229-1 规定 0x27 SecurityAccess 的子功能字节最高位（bit7）为 **Suppress Positive Response（SPR）**：

- requestSeed：`0x01`（正常）/ `0x81`（SPR）
- sendKey：`0x02`（正常）/ `0x82`（SPR）

SPR 置位时，ECU 应**抑制正响应**（不回复），但**负响应（0x7F）仍正常发送**。

原实现把 `sub & 0x80` 的判断放在了 `sub == 0x01` 分支内部，导致：

1. `0x01 & 0x80` 恒为 0，`suppress` 永远为 False —— SPR 分支是**不可达死代码**；
2. 真实发送 `0x81` 时，因 `0x81 != 0x01`，请求落入「不支持子功能」分支，返回错误 NRC。

## 三、复现步骤

前置条件：先切换到 Extended Session（`0x10 03`）。

1. 发送 `0x27 0x81`（requestSeed + SPR）
2. 观察 ECU 响应

## 四、实际结果 vs 预期结果

| | 内容 |
|---|---|
| 预期结果 | 不返回任何响应（SPR 抑制正响应），且内部记录 seed |
| 实际结果 | 返回 `7F 27 12`（SUB_FUNCTION_NOT_SUPPORTED） |

## 五、根因分析

```python
# 修复前（问题代码）
if sub == 0x01:  # requestSeed
    suppress = (sub & 0x80) != 0   # 恒为 False：0x01 的 bit7 为 0
    actual_sub = sub & 0x7F        # 恒为 0x01，变量冗余
    ...
elif sub == 0x02:  # sendKey
    ...
```

- SPR 位与子功能位是**同一字节的两个位**，却把 SPR 位提取放在具体分支**内部**，逻辑上无法成立；
- `0x81`（SPR requestSeed）因不等于 `0x01` 无法进入任何分支 → 错误返回 NRC `0x12`；
- 死代码未被发现，本质原因是**测试只覆盖了 `0x01/0x02` 路径，SPR 分支无任何用例**。

## 六、修复方案

在分发前**一次性**提取 SPR 位与有效子功能位：

```python
sub = request[1]
suppress = (sub & 0x80) != 0   # bit7 = Suppress Positive Response
actual_sub = sub & 0x7F

if actual_sub == 0x01:  # requestSeed
    self._pending_seed = random.randint(0, 0xFFFF)
    if suppress:
        return b""       # 抑制正响应
    ...
if actual_sub == 0x02:  # sendKey
    ...
    if received_key == expected_key:
        self.session.security_level = 1
        if suppress:
            return b""   # 抑制正响应
```

同时补齐 sendKey（`0x82`）路径的 SPR 支持：解锁成功但正响应被抑制。

## 七、验证结果

| 验证项 | 结果 |
|---|---|
| 新增自动化用例 | 3 条（0x81 requestSeed SPR / 0x82 sendKey SPR / SPR 下负响应不抑制） |
| 全量测试 | 240 条 pytest 全部通过（修复前 237 条） |
| CI 矩阵 | Python 3.10 / 3.11 / 3.12 全绿（GitHub Actions） |

## 八、回归风险评估

- 影响面：仅 `0x81/0x82` 两个子功能路径；`0x01/0x02` 正常路径行为不变（由既有 4 条用例保护）；
- 结论：回归风险低，可以合入。

## 九、经验总结

1. **状态位与子功能位共字节时，应先整体解析再分发**，避免「位判断藏在分支里」导致的死代码；
2. 死代码的直接原因是**测试未覆盖 SPR 路径**——设计用例时应先做等价类划分，保证每个协议分支至少一条用例；
3. 本缺陷由 Code Review 发现，属于静态分析可拦截的一类问题，可结合 linter/复杂度检查在 CI 中预防。
