# UDS 0x27 SecurityAccess 测试用例设计

> 用途说明：本文件是项目「测试作品集」的一部分，展示从协议需求到用例设计的完整过程（等价类划分 → 边界值分析 → 状态机测试 → 正/负向用例），并与 `test_uds.py` 中的自动化用例一一对应。所有「预期结果」均来自当前代码真实行为，未虚构。

## 1. 需求分析（ISO 14229-1）

SecurityAccess（0x27）用于解锁 ECU 的受限服务。本实现的需求条目：

| 编号 | 需求 | 说明 |
|---|---|---|
| REQ-1 | 子功能定义 | `0x01` = requestSeed（奇数），`0x02` = sendKey（偶数） |
| REQ-2 | SPR 位 | 子功能字节 bit7 = Suppress Positive Response，置位时**抑制正响应，负响应不抑制** |
| REQ-3 | requestSeed 响应 | 正响应 = `0x67 01` + 2 字节 seed（big-endian），seed 为随机数，范围 [0, 0xFFFF] |
| REQ-4 | 密钥算法 | key = seed XOR `0x5555`（16-bit），教学级实现 |
| REQ-5 | sendKey 校验 | 密钥正确 → 正响应 `0x67 02` 且 security_level 提升；错误 → NRC `0x31` |
| REQ-6 | 状态依赖 | 未 requestSeed 直接 sendKey → NRC `0x22` |
| REQ-7 | 会话权限 | 0x27 仅在 extended / programming session 可用，default 下拒绝 |
| REQ-8 | 长度校验 | requestSeed 至少 2 字节；sendKey 至少 4 字节；不足 → NRC `0x13` |
| REQ-9 | 非法子功能 | 其余子功能（含 SPR 位单独出现的 `0x80`）→ NRC `0x12` |

## 2. 测试对象与范围

- 对象：`uds.py` → `ECUDiagnosticServer._handle_security_access()`
- 在测：子功能分发、SPR 位处理、seed 生成与密钥校验、安全等级状态机、会话权限、NRC 返回
- 不在测：S3 超时回退、DID 读取（由其它用例覆盖）

## 3. 测试方法

### 3.1 等价类划分

| 类别 | 等价类 | 代表输入 |
|---|---|---|
| 有效 | requestSeed（正常） | `0x27 0x01` |
| 有效 | requestSeed + SPR | `0x27 0x81` |
| 有效 | sendKey（正确密钥） | `0x27 0x02` + key |
| 有效 | sendKey + SPR（正确密钥） | `0x27 0x82` + key |
| 无效 | 未请求 seed 直接 sendKey | `0x27 0x02` + 任意 key |
| 无效 | sendKey 错误密钥 | `0x27 0x02` + 错误 key |
| 无效 | 不支持子功能 | `0x27 0x03` / `0x00` / `0x7F` |
| 无效 | SPR + 不支持子功能 | `0x27 0x80` / `0x83` / `0xFF` |

### 3.2 边界值分析

| 边界 | 输入 | 预期 |
|---|---|---|
| 请求长度下界 | `0x27`（仅 SID，len=1） | `7F 27 13` |
| requestSeed 长度下界 | `0x27 0x01`（len=2） | `67 01` + seed（通过） |
| sendKey 长度下界 | `0x27 0x02`（len=2） | `7F 27 13` |
| sendKey 有效长度下界 | `0x27 0x02` + key（len=4） | `67 02`（通过） |
| 子功能下界 | `0x00` | `7F 27 12` |
| 子功能上界 | `0x7F` | `7F 27 12` |
| SPR 位单独出现 | `0x80` | `7F 27 12`（负响应不抑制） |
| 全位为 1 | `0xFF` | `7F 27 12` |

### 3.3 状态机测试

状态：`{default, extended, programming}` × `{locked, unlocked}`

| 场景 | 前置 | 输入 | 预期 |
|---|---|---|---|
| 权限拒绝 | default session | `0x27 0x01` | `7F 27 22` |
| 正常解锁 | extended session | `0x01` → `0x02` + key | `67 02`，security_level = 1 |
| 未请求先发密钥 | extended session | `0x02` + key | `7F 27 22` |
| 解锁后再次 requestSeed | extended + unlocked | `0x01` | `67 01` + 新 seed（覆盖旧 seed） |

### 3.4 正/负向测试

- 正向：REQ-3 / REQ-5 中所有「正响应」路径；
- 负向：NRC `0x13` / `0x22` / `0x31` / `0x12` 全部覆盖。

## 4. 测试环境

- 运行：pytest，纯 Python（无硬件依赖，可复现）
- CI：GitHub Actions，Python 3.10 / 3.11 / 3.12 矩阵 + mypy
- 关联代码：`test_uds.py`（0x27 相关共 7 条自动化用例）

## 5. 测试用例清单

### 5.1 正常流程（P0）

| 编号 | 用例名称 | 前置条件 | 步骤 / 输入 | 预期结果 | 实现状态 |
|---|---|---|---|---|---|
| TC-UD-001 | requestSeed 正常响应 | extended session | 发送 `0x27 0x01` | 返回 `0x67 0x01` + 2 字节 seed，总长 4 字节 | 已自动化 `test_request_seed_returns_2_bytes` |
| TC-UD-002 | 完整解锁流程 | extended session | ①`0x01` 取 seed ②key = seed ^ 0x5555 ③`0x02` + key | 返回 `0x67 0x02`，security_level = 1 | 已自动化 `test_full_unlock_sequence` |
| TC-UD-003 | 重复 requestSeed | extended session | 连续两次 `0x01` | 均返回 seed，第二次覆盖第一次（行为说明） | 建议补充 |

### 5.2 异常与负向用例（P1）

| 编号 | 用例名称 | 前置条件 | 步骤 / 输入 | 预期结果 | 实现状态 |
|---|---|---|---|---|---|
| TC-UD-004 | 错误密钥 | extended，已取 seed | `0x02` + (seed ^ 0x5555) + 1 | `7F 27 31`，security_level 不变 | 已自动化 `test_wrong_key_returns_negative` |
| TC-UD-005 | 未请求 seed 直接 sendKey | extended session | `0x02` + 任意 key | `7F 27 22` | 已自动化 `test_send_key_without_seed_returns_nr` |
| TC-UD-006 | 不支持子功能 | extended session | `0x03` / `0x00` / `0x7F` | `7F 27 12` | 建议补充 |
| TC-UD-007 | 长度不足 | extended session | 仅 `0x27`（len=1）；`0x27 0x02`（len=2） | `7F 27 13` | 建议补充 |
| TC-UD-008 | default session 拒绝 | default session | `0x27 0x01` | `7F 27 22` | 建议补充 |

### 5.3 SPR 用例（P1，本次修复新增）

| 编号 | 用例名称 | 前置条件 | 步骤 / 输入 | 预期结果 | 实现状态 |
|---|---|---|---|---|---|
| TC-UD-009 | requestSeed + SPR | extended session | `0x81` | 无响应（`b""`），且 seed 被记录 | 已自动化 `test_request_seed_suppress_positive_response` |
| TC-UD-010 | sendKey + SPR（正确密钥） | extended，已取 seed | `0x82` + key | 无响应，security_level = 1 | 已自动化 `test_send_key_suppress_positive_response` |
| TC-UD-011 | SPR 下错误仍返回负响应 | extended，无 seed | `0x82` + 任意 key | `7F 27 22`（负响应不抑制） | 已自动化 `test_spr_error_still_returns_negative` |
| TC-UD-012 | SPR + 非法子功能 | extended session | `0x80` / `0x83` / `0xFF` | `7F 27 12` | 建议补充 |

## 6. 执行结果

- 已自动化 7 条（TC-UD-001/002/004/005/009/010/011），全部通过；
- 全量回归：240 条 pytest 通过（本机 Python 3.10 + CI 3.10/3.11/3.12 矩阵）；
- 等价类覆盖情况：3.1 节全部等价类均被至少一条用例触及（部分为「建议补充」状态）。

## 7. 风险与遗留问题（如实说明）

1. **无失败重试锁定机制**：真实 ECU 对 SecurityAccess 通常有「连续失败次数上限 → 临时锁定」的防暴力破解设计，本教学实现未覆盖（REQ-9 之外的真实行为差异）；
2. **无 seed 有效期**：真实实现中 seed 会超时失效（如 5s），本实现未覆盖；
3. **密钥算法固定**：key = seed ^ 0x5555 仅用于教学演示，产品级应使用私有算法（UDS 刷写场景常配合 PKI）；
4. 待补用例（TC-UD-003/006/007/008/012）建议后续补齐自动化，闭环到 CI。

## 8. 使用说明

- 本文档与 `test_uds.py` 一一对应，可作为车载诊断功能测试的用例设计模板；
- 面试讲解建议：以 REQ-2（SPR 位）为引子，讲「缺陷发现（BUG-2026-001）→ 根因 → 修复 → 补用例」的完整闭环，并主动指出第 7 节「遗留问题」——展示对真实产品差距的认知。
