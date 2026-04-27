## 1. 总线配置

### 1.1 CAN1（内部总线）

**用途**：主控与从控、霍尔传感器、显示屏（数采）、校准工具通信

| 项目 | 值 | 说明 |
|---|---|---|
| 波特率 | 250kbps | 固定 |
| 帧类型 | 标准帧 + 扩展帧 | 混合使用 |
| 终端电阻 | 120Ω | 总线两端需配置 |

### 1.2 CAN2（车辆/充电机总线，车上的CANB）

**用途**：主控与车辆控制器（VCU）、显示屏（数采）或充电机通信

| 模式 | 条件 | 波特率 | 接收过滤行为 | 说明 |
|---|---|---:|---|---|
| 充电模式 | `CurrentChargeMode!=0` | 250kbps | 全通（Mask全0） | 接收充电机反馈 |
| 放电模式 | `CurrentChargeMode==0` | 500kbps | 精确匹配（Mask全1） | 实际近似不接收 |

---

## 2. 报文ID总表

### 2.1 CAN1 接收（主控 RX）

| ID | 帧型 | DLC | 方向 | 功能 |
|---|---|---:|---|---|
| `0x180050F3 + (n<<16)` | 扩展 | 8 | 从控→主控 | 单体电压（`n=0..35`，6个从控×6帧） |
| `0x184050F3 + (i<<16)` | 扩展 | 8 | 从控→主控 | 单体温度（`i=0..5`，6个从控） |
| `0x03C0` | 标准 | 8 | 霍尔→主控 | 电流传感器数据（CS09HV-C20） |
| `0x188050F5` | 扩展 | ≥5 | 工具→主控 | 修改充电参数 |
| `0x188150F5` | 扩展 | ≥6 | 工具→主控 | 修改告警阈值 |
| `0x188250F5` | 扩展 | ≥2 | 工具→主控 | 修改告警开关 |
| `0x188350F5` | 扩展 | ≥4 | 工具→主控 | 故障复位专用命令（含防误触校验） |
| `0x18A050F5` | 扩展 | ≥4 | 工具→主控 | ADC 电压校准命令 |
| `0x18A150F5` | 扩展 | ≥1 | 工具→主控 | 电流方向设置 |
| `0x18A350F5` | 扩展 | 8 | 工具→主控 | RTC 校时命令 |

> **长度约束**：所有接收帧均有最小长度检查，长度不足直接丢弃，不处理。

### 2.2 CAN1 发送（主控 TX）

| ID | DLC | 帧型 | 方向 | 功能 | 发送周期 |
|---|---:|---|---|---|---|
| `0x186050F4` | 7 | 扩展 | 主控→显示 | 电池总览（总压/总流/SOC/状态） | 500ms |
| `0x186150F4` | 6 | 扩展 | 主控→显示 | 单体电压极值 | 500ms |
| `0x186250F4` | 5 | 扩展 | 主控→显示 | 温度极值 | 500ms |
| `0x186350F4` | 8 | 扩展 | 主控→显示 | 继电器/充电状态 | 500ms |
| `0x186450F4` | 6 | 扩展 | 主控→显示 | 均衡状态分片0 | 250ms（条件） |
| `0x186550F4` | 6 | 扩展 | 主控→显示 | 均衡状态分片1 | 250ms（条件） |
| `0x186650F4` | 6 | 扩展 | 主控→显示 | 均衡状态分片2 | 250ms（条件） |
| `0x186750F4` | 2 | 扩展 | 主控→显示 | 单体电压累加值 | 500ms |
| `0x186850F4` | 8 | 扩展 | 主控→显示 | IMD 诊断信息 | 500ms |
| `0x187650F4` | 6 | 扩展 | 主控→显示 | 告警状态 | 500ms |
| `0x187750F4` | 6 | 扩展 | 主控→显示 | 告警阈值 | 500ms |
| `0x187F50F4` | 2 | 扩展 | 主控→显示 | 告警开关 | 500ms |
| `0x18A250F4` | 1 | 扩展 | 主控→工具 | 电流方向设置应答 | 事件触发 |
| `0x18A150F4` | 8 | 扩展 | 主控→工具 | ADC 校准应答 | 事件触发 |
| `0x18A450F4` | 8 | 扩展 | 主控→工具 | RTC 校时应答 | 事件触发 |

> **条件发送**：均衡状态帧仅在 `CellBalSwitch==1` 时发送。

### 2.3 CAN2 报文

| ID | DLC | 帧型 | 方向 | 功能 | 发送/接收条件 |
|---|---:|---|---|---|---|
| `0x1806E5F4` | 5 | 扩展 | 主控→充电机 | 充电请求（电压/电流/控制） | 充电模式，1s 周期 |
| `0x18FF50E5` | ≥5 | 扩展 | 充电机→主控 | 充电机反馈（电压/电流/状态） | 充电模式接收 |
| `0x401` | 8 | 标准 | 主控→VCU | 功率状态（电压/功率/电流/SOC/温度） | 放电模式，500ms 周期 |
| `0x402` | 8 | 标准 | 主控→VCU | 故障诊断（状态/告警/从控在线） | 放电模式，500ms 周期 |


## 3. 发送时序

### 3.1 CAN1 发送时序

| 软定时条件（主循环） | 周期 | 发送内容 |
|---|---|---|
| `sched_500ms_cnt` 到期 | 500ms | 9 帧状态报文（0x1860~0x187F） |
| `sched_250ms_cnt` 到期 且 `CellBalSwitch==1` | 250ms | 3 帧均衡状态（0x1864~0x1866） |

**9 帧状态报文**（按发送顺序）：
1. `0x186050F4` - 电池总览
2. `0x186750F4` - 单体电压累加值（同函数内）
3. `0x186850F4` - IMD 诊断信息
4. `0x186150F4` - 单体电压极值
5. `0x186250F4` - 温度极值
6. `0x186350F4` - 继电器/充电状态
7. `0x187650F4` - 告警状态
8. `0x187750F4` - 告警阈值
9. `0x187F50F4` - 告警开关

### 3.2 CAN2 发送时序

**充电模式**（`CurrentChargeMode != 0`）：
- `sched_1s_cnt` 到期（1s 周期）：发送 `0x1806E5F4` 充电请求

**放电模式**（`CurrentChargeMode == 0`）：
- `sched_500ms_cnt` 到期（500ms 周期）：发送 `0x401` + `0x402`

### 3.3 事件触发发送

以下报文不按周期发送，而是由事件触发：

| 事件 | 触发报文 |
|---|---|
| 收到电流方向设置命令 | `0x18A250F4` |
| 收到 ADC 校准命令 | `0x18A150F4` |
| 收到 RTC 校时命令 | `0x18A450F4` |
| HV_ACC 上高压完成 | CAN1 状态帧 |
| HV_ACC 下高压请求 | 停止充电命令 |

---

## 4. 字节序与单位约定

### 4.1 字节序规则

| 报文类别 | 字节序 | 说明 |
|---|---|---|
| CAN1 主控发送多字节整数 | **大端**（Big-Endian） | 高字节在前，如 `0x1234` → `[0x12, 0x34]` |
| CAN1 从控电压/温度 | **小端**（Little-Endian） | 低字节在前，如 `0x1234` → `[0x34, 0x12]` |
| 霍尔电流传感器 32 位值 | **大端** | 4 字节，高字节在前 |
| CAN2 `0x401/0x402` | **小端** | Intel 格式 |
| CAN2 充电请求/反馈 | **大端** | `0x1806E5F4` 和 `0x18FF50E5` |

> **重要**：字节序错误是对接失败的常见原因，务必严格遵守。

### 4.2 数据单位与编码

| 物理量 | 变量名示例 | 单位 | 编码方式 | 示例 |
|---|---|---|---|---|
| 总电压 | `BatVoltage` | 0.1V | 直接值 | `5700` → 570.0V |
| 预充电压 | `PreBatVoltage` | 0.1V | 直接值 | `5650` → 565.0V |
| 单体电压 | `CellVolt[]` | mV | 直接值 | `3650` → 3.650V |
| 单体电压累加 | `CellVoltCal` | 0.1V | 直接值 | `5700` → 570.0V |
| 总电流 | `BatCurrent` | 0.1A | 偏移 10000 | `10500` → +50.0A（充电）<br>`9500` → -50.0A（放电） |
| 功率 | `BatPower` | W | 偏移 1000 | `1500` → 放电 500W<br>`800` → 充电 200W |
| 温度 | `CellTemp[]` | °C | 偏移 30 | `55` → 25°C<br>`10` → -20°C |
| SOC | `BatSoc` | % | 直接值 | `80` → 80% |
| 充电请求电压 | `ChrReqVolt` | 0.1V | 直接值 | `5800` → 580.0V |
| 充电请求电流 | `ChrReqCurr` | 0.1A | 直接值 | `100` → 10.0A |

### 4.3 编码转换公式

**电流转换**：
- 原始值 → 物理值：(BatCurrent - 10000) / 10，单位A
- 物理值 → 原始值：current_A * 10 + 10000

**功率转换**：
- 原始值 → 物理值：BatPower - 1000，>0放电,<0充电
- 物理值 → 原始值：power_W + 1000

**温度转换**：
- 原始值 → 物理值：CellTemp - 30
- 物理值 → 原始值：temp_C + 30

---

## 5. CAN1 接收详解

### 5.1 从控电压帧（兼容历史映射）

**帧类型**：扩展帧，DLC=8
**数据格式**：小端（Little-Endian）

#### 5.1.1 ID 规则

**基址**：`BASE_V = 0x180050F3`

**从控 ID 段分配**（`i = 0..5`）：
- 从控 0：`0x180050F3` ~ `0x180550F3`（帧 0~5）
- 从控 1：`0x180650F3` ~ `0x180B50F3`（帧 6~11）
- 从控 2：`0x180C50F3` ~ `0x181150F3`（帧 12~17）
- 从控 3：`0x181250F3` ~ `0x181750F3`（帧 18~23）
- 从控 4：`0x181850F3` ~ `0x181D50F3`（帧 24~29）
- 从控 5：`0x181E50F3` ~ `0x182350F3`（帧 30~35）

**ID 计算公式**：id = 0x180050F3 + ((i * 6 + frame_num) << 16)，其中 i=0..5, frame_num=0..5

**全局帧号**：`n = ((ID - 0x180050F3) >> 16)`，范围 `0..35`

#### 5.1.2 数据落表规则

**数组起始索引**：`vol_sum = {0, 23, 46, 69, 92, 115, 138}`

**特殊映射规则**：
- **帧 0**（每个从控的第一帧）：取 `Byte2..Byte7`，解析 3 个电压
- **帧 1~5**：取 `Byte0..Byte7`，解析 4 个电压

**结果**：每个从控 `3 + 5×4 = 23` 串电压

### 5.2 从控温度帧

**帧类型**：扩展帧，DLC=8
**数据格式**：直接值（无字节序问题，单字节）

#### 5.2.1 ID 规则

**基址**：`BASE_T = 0x184050F3`
**ID 计算**：`ID = BASE_T + (i << 16)`，其中 `i = 0..5`

**从控 ID 分配**：
- 从控 0：`0x184050F3`
- 从控 1：`0x184150F3`
- 从控 2：`0x184250F3`
- 从控 3：`0x184350F3`
- 从控 4：`0x184450F3`
- 从控 5：`0x184550F3`

#### 5.2.2 数据格式

**数组起始索引**：`tem_sum = {0, 8, 16, 24, 32, 40, 48}`

**数据内容**：`Byte0..Byte7` 为 8 路温度偏移值
- 单位：°C（偏移 30）
- 示例：`Byte0 = 55` → 实际温度 = 55 - 30 = 25°C

### 5.3 霍尔电流传感器帧 `0x03C0`

**传感器型号**：CS09HV-C20 闭环霍尔电流传感器
**帧类型**：标准帧，DLC=8
**数据格式**：大端（Big-Endian）

#### 5.3.1 数据格式

| Byte | 位 | 含义 | 说明 |
|---|---|---|---|
| 0..3 | - | `raw_ip`（32位，大端） | 原始电流值 |
| 4 | 0 | `is_error` | 0=正常，1=故障 |
| 4 | 1..7 | `error_code` | 故障代码（7位） |
| 5..6 | - | 传感器名称（16位，大端） | 设备标识 |
| 7 | - | 软件版本 | 传感器固件版本 |

#### 5.3.2 电流计算

**原始值转换**：
```
// 解析 raw_ip（大端）
raw_ip = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];

// 转换为有符号电流（mA）
current_ma = raw_ip - 0x80000000;
```

**方向约定**（可通过 CAN 命令配置）：
- 默认：正值 = 充电，负值 = 放电
- 反转：正值 = 放电，负值 = 充电

#### 5.3.3 故障检测

**故障标志**：`data[4] & 0x01`
**故障代码**：(data[4] >> 1) & 0x7F

### 5.4 参数修改命令

**帧类型**：扩展帧
**数据格式**：大端（Big-Endian）

#### 5.4.1 充电参数修改 `0x188050F5`

**最小长度**：DLC ≥ 5

| Byte | 含义 | 单位 | 说明 |
|---|---|---|---|
| 0..1 | `ChrReqVolt`（大端） | 0.1V | 充电请求电压 |
| 2..3 | `ChrReqCurr`（大端） | 0.1A | 充电请求电流 |
| 4 | 控制字段 | - | bit5=1 清除故障日志 |
| 5..7 | 保留 | - | 未使用 |

**注意**：
- 更新 ChrReqVolt 和 ChrReqCurr
- 如果 Byte4 & 0x20，则清除故障日志

#### 5.4.2 告警阈值修改 `0x188150F5`

**最小长度**：DLC ≥ 6

| Byte | 含义 | 单位 | 说明 |
|---|---|---|---|
| 0..1 | `ALM_CELL_OV_VALUE`（大端） | mV | 单体过压阈值 |
| 2..3 | `ALM_CELL_UV_VALUE`（大端） | mV | 单体欠压阈值 |
| 4 | `ALM_CELL_OT_VALUE` | 偏移30 | 高温阈值 |
| 5 | `ALM_CELL_UT_VALUE` | 偏移30 | 低温阈值 |
| 6..7 | 保留 | - | 未使用 |

#### 5.4.3 告警开关修改 `0x188250F5`

**最小长度**：DLC ≥ 2

| Byte | 位 | 含义 | 说明 |
|---|---|---|---|
| 0 | 7 | `ALM_CELL_OV_SWITCH` | 单体过压告警开关 |
| 0 | 6 | `ALM_CELL_UV_SWITCH` | 单体欠压告警开关 |
| 0 | 5 | `ALM_CELL_OT_SWITCH` | 高温告警开关 |
| 0 | 4 | `ALM_CELL_UT_SWITCH` | 低温告警开关 |
| 0 | 3 | `ALM_BATT_DV_SWITCH` | 压差告警开关 |
| 0 | 2 | `ALM_BATT_DT_SWITCH` | 温差告警开关 |
| 0 | 1 | `ALM_CHRG_OCS_SWITCH` | 充电过流告警开关 |
| 0 | 0 | `ALM_DSCH_OCS_SWITCH` | 放电过流告警开关 |
| 1 | 7 | `ALM_BSU_OFFLINE_SWITCH` | 从控离线告警开关 |
| 1 | 6 | 保留（兼容） | 预充失败固定硬保护，不可关闭 |
| 1 | 5 | `ALM_HVREL_FAIL_SWITCH` | 高压异常告警开关 |
| 1 | 4 | `ALM_HALL_BREAK_SWITCH` | 霍尔断线告警开关 |
| 1 | 3 | `ALM_BATT_OV_SWITCH` | 电池总压过压告警开关 |
| 1 | 2 | `ALM_BATT_UV_SWITCH` | 电池总压欠压告警开关 |
| 1 | 1 | `BeepSwitch` | 蜂鸣器总开关（1=启用，0=禁用） |
| 1 | 0 | 扩展开关有效标记 | 1=Byte1 bit3/bit2/bit1 有效，0=兼容旧格式 |

> **注意 1**：Byte1 bit6（历史预充失败开关位）输入兼容但忽略，预充失败固定硬保护。
> **注意 2**：Byte1 bit0 需置 1，主控才会按 bit3/bit2/bit1 更新 `ALM_BATT_OV_SWITCH/ALM_BATT_UV_SWITCH/BeepSwitch`（兼容旧上位机）。

#### 5.4.4 故障复位专用命令 `0x188350F5`

**最小长度**：DLC ≥ 4
**防误触机制**：固定命令字 + 异或校验

| Byte | 值 | 说明 |
|---|---|---|
| 0 | `0xA5` | 固定命令字 0 |
| 1 | `0x5A` | 固定命令字 1 |
| 2 | `0x3C` | 二次确认字段 |
| 3 | `Byte0 ^ Byte1 ^ Byte2` | 异或校验（应为 `0x99`） |
| 4..7 | 保留 | 未使用 |

**执行条件**：
1. `BatState == 7`（故障态）
2. 上述 4 字节校验通过

> **安全说明**：故障复位仅在 `BatState==7` 时有效，且需满足安全条件（如故障已消除）。

### 5.5 ADC 电压校准命令 `0x18A050F5`

**最小长度**：DLC ≥ 4
**帧类型**：扩展帧
**数据格式**：小端（Little-Endian）

#### 5.5.1 命令格式

| Byte | 含义 | 说明 |
|---|---|---|
| 0 | 命令码 | 1=记录低点, 2=记录高点, 3=计算保存, 4=恢复默认, 5=查询 |
| 1 | 通道选择 | 0=BAT 端, 1=PRE 端 |
| 2..3 | 真实电压（小端） | 单位 0.1V，仅命令 1/2 使用 |
| 4..7 | 保留 | 未使用 |

#### 5.5.2 命令详解

| 命令码 | 功能 |
|---|---|
| 1 | 记录低点 |
| 2 | 记录高点 |
| 3 | 计算并保存 |
| 4 | 恢复默认 |
| 5 | 查询 |

#### 5.5.3 应答格式 `0x18A150F4`

**通用应答**（cmd=1/2/3/4/未知）：

| Byte | 含义 | 说明 |
|---|---|---|
| 0 | 命令码回显 | 原命令码 |
| 1 | 状态码 | 见下表 |
| 2..3 | `adc_now`（小端） | 当前 ADC 原始值 |
| 4..7 | 保留（0） | 未使用 |

**状态码**：
- `0x00`：成功
- `0x01`：校准点不合理，无法计算（如两点太接近）
- `0xFD`：Flash 保存失败
- `0xFE`：通道参数错误（仅支持 0/1）
- `0xFF`：未知命令

**查询应答**（cmd=5）：

| Byte | 含义 | 说明 |
|---|---|---|
| 0 | `0x05` | 命令码 |
| 1 | 校准状态 | 1=已校准, 0=未校准 |
| 2..3 | `adc_now`（小端） | 当前 ADC 原始值 |
| 4..5 | `v_now`（小端） | 当前电压（0.1V） |
| 6..7 | 保留（0） | 未使用 |

#### 5.5.4 注意事项

1. **两点法校准**：低点和高点应尽量覆盖实际使用范围（如 100V 和 600V）
2. **点间距要求**：两点电压差应 > 50V，否则返回 `0x01` 错误
3. **通道独立**：BAT 和 PRE 通道独立校准，互不影响

### 5.6 电流方向设置命令 `0x18A150F5`

**最小长度**：DLC ≥ 1
**帧类型**：扩展帧

#### 5.6.1 命令格式

| Byte | 含义 | 说明 |
|---|---|---|
| 0 | 方向设置 | 0=正常（正值=充电），1=反转（正值=放电） |
| 1..7 | 保留 | 未使用 |

#### 5.6.2 方向约定

**正常模式**（`Byte0 = 0`）：
- 霍尔传感器正值 → 充电电流（`BatCurrent > 10000`）
- 霍尔传感器负值 → 放电电流（`BatCurrent < 10000`）

**反转模式**（`Byte0 = 1`）：
- 霍尔传感器正值 → 放电电流（`BatCurrent < 10000`）
- 霍尔传感器负值 → 充电电流（`BatCurrent > 10000`）

> **用途**：适配不同的霍尔传感器安装方向，无需重新接线。

#### 5.6.3 应答格式 `0x18A250F4`

**应答内容**：

| Byte | 含义 | 说明 |
|---|---|---|
| 0 | 当前方向值 | 0=正常, 1=反转（规范化后的值） |

> **注意**：应答返回规范化后的当前方向值（仅 0/1），避免上位机收到非法回显值。

#### 5.6.4 注意事项

1. **立即生效**：设置后立即影响电流方向解析
2. **非法值处理**：非 0/1 值会被规范化为 0 或 1
3. **应答确认**：建议等待应答确认设置成功

### 5.7 RTC 校时命令 `0x18A350F5`

**长度要求**：DLC 必须为 8
**帧类型**：扩展帧

#### 5.7.1 命令格式

| Byte | 含义 | 说明 |
|---|---|---|
| 0..1 | 年（大端） | 合法范围 1970..2099 |
| 2 | 月 | 1..12 |
| 3 | 日 | 按月份和闰年检查 |
| 4 | 时 | 0..23 |
| 5 | 分 | 0..59 |
| 6 | 秒 | 0..59 |
| 7 | 校验和 | `Byte0 ^ Byte1 ^ ... ^ Byte6` |

#### 5.7.2 应答格式 `0x18A450F4`

| Byte | 含义 | 说明 |
|---|---|---|
| 0 | 状态码 | 见下表 |
| 1..2 | 当前年（大端） | 成功时为新时间；失败时为当前 RTC 快照（不可用则为 0） |
| 3 | 当前月 | 同上 |
| 4 | 当前日 | 同上 |
| 5 | 当前时 | 同上 |
| 6 | 当前分 | 同上 |
| 7 | 当前秒 | 同上 |

状态码：
- `0x00`：校时成功
- `0x01`：长度错误（DLC < 8）
- `0x02`：校验和错误
- `0x03`：参数越界（年月日时分秒非法）
- `0x04`：RTC 未就绪（初始化失败）
- `0x05`：RTC 设置超时（写完成等待超时或同步等待超时）
- `0x06`：RTC 未知内部错误

#### 5.7.4 注意事项

1. 当短时间收到多帧校时命令时，系统按”最后一帧”执行。
2. 校时成功后会写入”已校时”标志。

---

## 6. CAN1 发送详解

**数据格式**：大端（Big-Endian）
**帧类型**：扩展帧
**发送周期**：见第 3 节

### 6.1 `0x186050F4` 电池总览（DLC=7）

**发送周期**：500ms

| Byte | 含义 | 单位 | 说明 |
|---|---|---|---|
| 0..1 | `BatVoltage`（大端） | 0.1V | 总电压 |
| 2..3 | 电流（大端） | 偏移 10000, 0.1A | 充电模式发 `ChrBatCurrent`，放电模式发 `BatCurrent` |
| 4 | `BatSoc` | % | SOC |
| 5 | `IMDSIG_TRANS_RAW` | - | IMD 数字信号原始电平（0=正常，1=故障） |
| 6 | 状态/告警 | - | 高 4 位 `BatState`，低 4 位 `BatAlmLv` |

**解析示例**：
```c
void parse_bat_total(const uint8_t data[7]) {
    uint16_t voltage = ((uint16_t)data[0] << 8) | data[1];  // 0.1V
    uint16_t current = ((uint16_t)data[2] << 8) | data[3];
    uint8_t soc = data[4];
    uint8_t imd_raw = data[5];  // 0=IMD正常, 1=IMD故障
    uint8_t state = (data[6] >> 4) & 0x0F;
    uint8_t alm_lv = data[6] & 0x0F;

    float voltage_V = voltage / 10.0f;
    float current_A = ((int16_t)current - 10000) / 10.0f;

    printf("Voltage: %.1fV, Current: %.1fA, SOC: %d%%, State: %d, AlmLv: %d\n",
           voltage_V, current_A, soc, state, alm_lv);
}
```

**BatState 状态码**：
- `2`：自检中
- `3`：待机
- `4`：预充中
- `5`：正常运行
- `7`：故障

**BatAlmLv 告警级别**：
- `0`：无告警
- `1`：一级告警（警告）
- `2`：二级告警（严重）

### 6.1.1 附加帧：`0x186750F4` 单体电压累加值（DLC=2）

**发送周期**：500ms

| Byte | 含义 | 单位 | 说明 |
|---|---|---|---|
| 0..1 | `CellVoltCal`（大端） | 0.1V | 所有单体电压累加值 |

**用途**：用于高压异常检测，对比总电压与单体累加值

**解析示例**：
```c
uint16_t cell_volt_cal = ((uint16_t)data[0] << 8) | data[1];
float cell_sum_V = cell_volt_cal / 10.0f;
printf("Cell Voltage Sum: %.1fV\n", cell_sum_V);
```

> **兼容性说明**：`0x186050F4` 的既有 7 字节格式保持不变；`CellVoltCal` 通过新增独立帧发送。

### 6.1.2 附加帧：`0x186850F4` IMD 诊断信息（DLC=8）

**发送周期**：500ms

| Byte | 含义 | 单位 | 说明 |
|---|---|---|---|
| 0 | `freq_class` | - | 0/10/20/30/40/50/255（未知） |
| 1 | 状态位图 | bit | b7=digital_ok, b6=pwm_signal_ok, b5=insulation_valid, b4=insulation_pass, b3=sst_good, b2=sst_bad, b1=duty_integrity_ok, b0=pa8_level |
| 2..3 | `duty_permille`（大端） | 0.1% | 占空比千分比（0..1000） |
| 4..5 | `rf_kohm`（大端） | kOhm | 绝缘电阻（10/20Hz 有效；`0xFFFF` 可表示“上限/近似无穷大”） |
| 6..7 | `freq_0p01hz`（大端） | 0.01Hz | 频率编码（如 10.00Hz 对应 1000） |

### 6.2 `0x186150F4` 单体电压极值（DLC=6）

**发送周期**：500ms

| Byte | 含义 | 单位 | 说明 |
|---|---|---|---|
| 0..1 | `MaxCellVolt`（大端） | mV | 最高单体电压 |
| 2..3 | `MinCellVolt`（大端） | mV | 最低单体电压 |
| 4 | `MaxCellVoltNo` | - | 最高电压序号（0~137） |
| 5 | `MinCellVoltNo` | - | 最低电压序号（0~137） |

**解析示例**：
```c
uint16_t max_v = ((uint16_t)data[0] << 8) | data[1];
uint16_t min_v = ((uint16_t)data[2] << 8) | data[3];
uint8_t max_no = data[4];
uint8_t min_no = data[5];

printf("Max: %dmV (Cell %d), Min: %dmV (Cell %d), Delta: %dmV\n",
       max_v, max_no, min_v, min_no, max_v - min_v);
```

### 6.3 `0x186250F4` 温度极值（DLC=5）

**发送周期**：500ms

| Byte | 含义 | 单位 | 说明 |
|---|---|---|---|
| 0 | `MaxTemp` | 偏移 30 | 最高温度 |
| 1 | `MinTemp` | 偏移 30 | 最低温度 |
| 2 | `MaxTempNo` | - | 最高温度序号（0~47） |
| 3 | `MinTempNo` | - | 最低温度序号（0~47） |
| 4 | `CoolingCtl` | - | 风冷控制状态 |

**解析示例**：
```c
int8_t max_t = (int8_t)data[0] - 30;
int8_t min_t = (int8_t)data[1] - 30;
uint8_t max_no = data[2];
uint8_t min_no = data[3];
uint8_t cooling = data[4];

printf("Max: %d°C (Sensor %d), Min: %d°C (Sensor %d), Cooling: %d\n",
       max_t, max_no, min_t, min_no, cooling);
```

### 6.4 `0x186350F4` 继电器/充电状态（DLC=8）

**发送周期**：500ms

| Byte | 位 | 含义 | 说明 |
|---|---|---|---|
| 0 | 7..6 | `PosRlyStr` | 正继电器状态（0=断开, 1=闭合） |
| 0 | 5..4 | `NegRlyStr` | 负继电器状态 |
| 0 | 3..2 | `PreRlyStr` | 预充继电器状态 |
| 0 | 1..0 | 保留 | 未使用 |
| 1 | 7..4 | `ChrState` | 充电状态 |
| 1 | 3 | `ChrCommunication` | 充电机通信状态 |
| 1 | 2..0 | 保留 | 未使用 |
| 2..3 | - | `ChrReqVolt`（大端） | 充电请求电压（0.1V） |
| 4..5 | - | `ChrReqCurr`（大端） | 充电请求电流（0.1A） |
| 6..7 | - | `PreBatVoltage`（大端） | 预充电压（0.1V） |

**解析示例**：
```c
uint8_t pos_rly = (data[0] >> 6) & 0x03;
uint8_t neg_rly = (data[0] >> 4) & 0x03;
uint8_t pre_rly = (data[0] >> 2) & 0x03;
uint8_t chr_state = (data[1] >> 4) & 0x0F;
uint8_t chr_comm = (data[1] >> 3) & 0x01;
uint16_t chr_v = ((uint16_t)data[2] << 8) | data[3];
uint16_t chr_i = ((uint16_t)data[4] << 8) | data[5];
uint16_t pre_v = ((uint16_t)data[6] << 8) | data[7];

printf("Relay: Pos=%d Neg=%d Pre=%d, Charge: State=%d Comm=%d Req=%.1fV/%.1fA, PreV=%.1fV\n",
       pos_rly, neg_rly, pre_rly, chr_state, chr_comm,
       chr_v/10.0f, chr_i/10.0f, pre_v/10.0f);
```

### 6.5 均衡状态（3 帧）

**发送周期**：250ms（条件：`CellBalSwitch==1`）

| ID | DLC | 内容 | 说明 |
|---|---:|---|---|
| `0x186450F4` | 6 | `CellBalanceN[0..5]` | 均衡状态分片 0 |
| `0x186550F4` | 6 | `CellBalanceN[6..11]` | 均衡状态分片 1 |
| `0x186650F4` | 6 | `CellBalanceN[12..17]` | 均衡状态分片 2 |

**数据格式**：每字节的每个 bit 表示一个单体的均衡状态（1=均衡中，0=未均衡）

**解析示例**：
```c
// 解析第一帧（Cell 0~47）
void parse_balance_frame0(const uint8_t data[6]) {
    for (uint8_t byte_idx = 0; byte_idx < 6; byte_idx++) {
        for (uint8_t bit_idx = 0; bit_idx < 8; bit_idx++) {
            uint8_t cell_no = byte_idx * 8 + bit_idx;
            uint8_t is_balancing = (data[byte_idx] >> bit_idx) & 0x01;
            if (is_balancing) {
                printf("Cell %d is balancing\n", cell_no);
            }
        }
    }
}
```

### 6.6 `0x187650F4` 告警状态（DLC=6）

**发送周期**：500ms

**数据格式**：每个告警占 2 位（0=无告警, 1=一级告警, 2=二级告警）

| Byte | 位 | 含义 | 说明 |
|---|---|---|---|
| 0 | 7..6 | `ALM_CELL_OV` | 单体过压告警 |
| 0 | 5..4 | `ALM_CELL_UV` | 单体欠压告警 |
| 0 | 3..2 | `ALM_CELL_OT` | 高温告警 |
| 0 | 1..0 | `ALM_CELL_UT` | 低温告警 |
| 1 | 7 | `ALM_CELL_LBK` | 单体电压断线告警（1 位） |
| 1 | 6 | `ALM_CELL_TBK` | 温度断线告警（1 位） |
| 1 | 5..4 | `ALM_BATT_DV` | 压差告警 |
| 1 | 3..2 | `ALM_BATT_DT` | 温差告警 |
| 1 | 1..0 | `ALM_BATT_OV` | 总压过压告警 |
| 2 | 7..6 | `ALM_BATT_UV` | 总压欠压告警 |
| 2 | 5..4 | `ALM_BATT_OC` | 总流过流告警 |
| 2 | 3..2 | `ALM_BATT_UC` | 总流欠流告警 |
| 2 | 1..0 | `ALM_CHRG_OCS` | 充电过流告警 |
| 3 | 7..6 | `ALM_DSCH_OCS` | 放电过流告警 |
| 3 | 5..4 | `ALM_CHRG_OCT` | 充电过温告警 |
| 3 | 3..2 | `ALM_DSCH_OCT` | 放电过温告警 |
| 3 | 1..0 | 保留 | 未使用 |
| 4 | 7..5 | 保留 | 未使用 |
| 4 | 4 | `ALM_PRECHRG_FAIL` | 预充失败告警（1 位） |
| 4 | 3 | `ALM_AUX_FAIL` | 辅助电源故障（1 位） |
| 4 | 2 | `BMSSlaveLife[4]` | 从控 4 离线（1 位） |
| 4 | 1 | `BMSSlaveLife[5]` | 从控 5 离线（1 位） |
| 4 | 0 | 保留 | 未使用 |
| 5 | 7 | `ALM_HVREL_FAIL` | 高压异常告警（1 位） |
| 5 | 6 | `ALM_HALL_BREAK` | 霍尔断线告警（1 位） |
| 5 | 5 | `ALM_BSU_OFFLINE` | 从控离线告警（1 位） |
| 5 | 4 | `BMSSlaveLife[0]` | 从控 0 离线（1 位） |
| 5 | 3 | `BMSSlaveLife[1]` | 从控 1 离线（1 位） |
| 5 | 2 | `BMSSlaveLife[2]` | 从控 2 离线（1 位） |
| 5 | 1 | `BMSSlaveLife[3]` | 从控 3 离线（1 位） |
| 5 | 0 | 保留 | 未使用 |

**解析示例**：
```c
void parse_alarm(const uint8_t data[6]) {
    // 2 位告警
    uint8_t cell_ov = (data[0] >> 6) & 0x03;
    uint8_t cell_uv = (data[0] >> 4) & 0x03;
    uint8_t cell_ot = (data[0] >> 2) & 0x03;
    uint8_t cell_ut = data[0] & 0x03;

    // 1 位告警
    uint8_t cell_lbk = (data[1] >> 7) & 0x01;
    uint8_t cell_tbk = (data[1] >> 6) & 0x01;
    uint8_t prechrg_fail = (data[4] >> 4) & 0x01;
    uint8_t hvrel_fail = (data[5] >> 7) & 0x01;
    uint8_t hall_break = (data[5] >> 6) & 0x01;

    // 从控在线状态
    uint8_t slave_online[6];
    slave_online[0] = !((data[5] >> 4) & 0x01);
    slave_online[1] = !((data[5] >> 3) & 0x01);
    slave_online[2] = !((data[5] >> 2) & 0x01);
    slave_online[3] = !((data[5] >> 1) & 0x01);
    slave_online[4] = !((data[4] >> 2) & 0x01);
    slave_online[5] = !((data[4] >> 1) & 0x01);

    printf("Alarms: OV=%d UV=%d OT=%d UT=%d PreChgFail=%d HVFail=%d HallBreak=%d\n",
           cell_ov, cell_uv, cell_ot, cell_ut, prechrg_fail, hvrel_fail, hall_break);
}
```

### 6.7 `0x187750F4` 告警阈值（DLC=6）

**发送周期**：500ms

| Byte | 含义 | 单位 | 说明 |
|---|---|---|---|
| 0..1 | `ALM_CELL_OV_VALUE`（大端） | mV | 单体过压阈值 |
| 2..3 | `ALM_CELL_UV_VALUE`（大端） | mV | 单体欠压阈值 |
| 4 | `ALM_CELL_OT_VALUE` | 偏移 30 | 高温阈值 |
| 5 | `ALM_CELL_UT_VALUE` | 偏移 30 | 低温阈值 |

**解析示例**：
```c
uint16_t ov_th = ((uint16_t)data[0] << 8) | data[1];
uint16_t uv_th = ((uint16_t)data[2] << 8) | data[3];
int8_t ot_th = (int8_t)data[4] - 30;
int8_t ut_th = (int8_t)data[5] - 30;

printf("Thresholds: OV=%dmV UV=%dmV OT=%d°C UT=%d°C\n",
       ov_th, uv_th, ot_th, ut_th);
```

### 6.8 `0x187F50F4` 告警开关（DLC=2）

**发送周期**：500ms

**数据格式**：与 `0x188250F5` 接收命令格式相同

| Byte | 位 | 含义 | 说明 |
|---|---|---|---|
| 0 | 7 | `ALM_CELL_OV_SWITCH` | 单体过压告警开关 |
| 0 | 6 | `ALM_CELL_UV_SWITCH` | 单体欠压告警开关 |
| 0 | 5 | `ALM_CELL_OT_SWITCH` | 高温告警开关 |
| 0 | 4 | `ALM_CELL_UT_SWITCH` | 低温告警开关 |
| 0 | 3 | `ALM_BATT_DV_SWITCH` | 压差告警开关 |
| 0 | 2 | `ALM_BATT_DT_SWITCH` | 温差告警开关 |
| 0 | 1 | `ALM_CHRG_OCS_SWITCH` | 充电过流告警开关 |
| 0 | 0 | `ALM_DSCH_OCS_SWITCH` | 放电过流告警开关 |
| 1 | 7 | `ALM_BSU_OFFLINE_SWITCH` | 从控离线告警开关 |
| 1 | 6 | 固定为 1 | 预充失败固定硬保护（兼容保留） |
| 1 | 5 | `ALM_HVREL_FAIL_SWITCH` | 高压异常告警开关 |
| 1 | 4 | `ALM_HALL_BREAK_SWITCH` | 霍尔断线告警开关 |
| 1 | 3 | `ALM_BATT_OV_SWITCH` | 电池总压过压告警开关 |
| 1 | 2 | `ALM_BATT_UV_SWITCH` | 电池总压欠压告警开关 |
| 1 | 1 | `BeepSwitch` | 蜂鸣器总开关 |
| 1 | 0 | `ALM_IMD_FAIL_SWITCH` | IMD 数字故障主控冗余开关 |

> **注意**：Byte1 bit6 固定上报为 1，用于预充失败硬保护兼容保留；bit0 现在用于 `ALM_IMD_FAIL_SWITCH`。该开关只控制主控侧 IMD 冗余掉 AIR 与故障锁存，不关闭上高压前/过程中的 IMD 硬保护检查。

### 6.9 `0x18A150F4` ADC 校准应答（DLC=8）

**触发条件**：收到 ADC 校准命令后

**通用应答**（cmd=1/2/3/4/未知）：

| Byte | 含义 | 说明 |
|---|---|---|
| 0 | 命令码回显 | 原命令码 |
| 1 | 状态码 | 0x00=成功, 0x01=校准点不合理, 0xFD=Flash 失败, 0xFE=通道错误, 0xFF=未知命令 |
| 2..3 | `adc_now`（小端） | 当前 ADC 原始值 |
| 4..7 | 保留（0） | 未使用 |

**查询应答**（cmd=5）：

| Byte | 含义 | 说明 |
|---|---|---|
| 0 | `0x05` | 命令码 |
| 1 | 校准状态 | 1=已校准, 0=未校准 |
| 2..3 | `adc_now`（小端） | 当前 ADC 原始值 |
| 4..5 | `v_now`（小端） | 当前电压（0.1V） |
| 6..7 | 保留（0） | 未使用 |

**解析示例**：
```c
void parse_calib_reply(const uint8_t data[8]) {
    uint8_t cmd = data[0];
    uint8_t status = data[1];
    uint16_t adc_now = data[2] | ((uint16_t)data[3] << 8);  // 小端

    if (cmd == 0x05) {  // 查询应答
        uint16_t v_now = data[4] | ((uint16_t)data[5] << 8);
        printf("Query: Calibrated=%d, ADC=%d, Voltage=%.1fV\n",
               status, adc_now, v_now / 10.0f);
    } else {  // 通用应答
        const char *status_str[] = {"Success", "Invalid Points", "Unknown", "Flash Error", "Channel Error", "Unknown Cmd"};
        uint8_t idx = (status == 0x00) ? 0 : (status == 0x01) ? 1 : (status == 0xFD) ? 3 : (status == 0xFE) ? 4 : 5;
        printf("Cmd %d: %s, ADC=%d\n", cmd, status_str[idx], adc_now);
    }
}
```

---

## 7. CAN2 详解

**用途**：与车辆控制器（VCU）或充电机通信
**模式切换**：根据 `CurrentChargeMode` 自动切换波特率和过滤器

### 7.1 `0x1806E5F4` 充电请求（扩展帧，DLC=5）

**发送周期**：1s（充电模式）
**数据格式**：大端（Big-Endian）

| Byte | 含义 | 单位 | 说明 |
|---|---|---|---|
| 0..1 | `ChrReqVolt`（大端） | 0.1V | 请求充电电压 |
| 2..3 | `ChrReqCurr`（大端） | 0.1A | 请求充电电流（直接值，无换算；示例：`100` 表示 `10.0A`） |
| 4 | `ChrCtl` | - | 控制：`0=启动充电`，`1=停止充电` |

> **重要**：Byte4 含义与主流程一致：启动充电时发送 0，停机/下高压时发送 1。

**发送示例**：
```c
// 请求充电：580.0V, 10.0A
uint8_t data[5];
data[0] = (5800 >> 8) & 0xFF;
data[1] = 5800 & 0xFF;
data[2] = (100 >> 8) & 0xFF;
data[3] = 100 & 0xFF;
data[4] = 0x00;  // 启动充电
CAN_Send_ExtId(0x1806E5F4, data, 5);
```

**解析示例**：
```c
uint16_t req_v = ((uint16_t)data[0] << 8) | data[1];
uint16_t req_i = ((uint16_t)data[2] << 8) | data[3];
uint8_t ctl = data[4];

printf("Charge Request: %.1fV, %.1fA, Control=%s\n",
       req_v / 10.0f, req_i / 10.0f, ctl ? "Stop" : "Start");
```

### 7.2 `0x401` 功率状态（标准帧，DLC=8）

**发送周期**：500ms（放电模式）
**数据格式**：小端（Little-Endian，Intel 格式）

| Byte | 含义 | 单位 | 说明 |
|---|---|---|---|
| 0..1 | `BatVoltage`（小端） | 0.1V | 总电压 |
| 2..3 | `BatPower`（小端） | W | 功率（偏移 1000） |
| 4..5 | `BatCurrent`（小端） | 0.1A | 电流（偏移 10000） |
| 6 | `BatSoc` | % | SOC |
| 7 | `MaxTemp` | 偏移 30 | 最高温度 |

**解析示例**：
```c
uint16_t voltage = data[0] | ((uint16_t)data[1] << 8);
uint16_t power = data[2] | ((uint16_t)data[3] << 8);
uint16_t current = data[4] | ((uint16_t)data[5] << 8);
uint8_t soc = data[6];
int8_t max_temp = (int8_t)data[7] - 30;

float voltage_V = voltage / 10.0f;
int16_t power_W = (int16_t)power - 1000;  // >0 放电, <0 充电
float current_A = ((int16_t)current - 10000) / 10.0f;

printf("Power: %.1fV, %dW, %.1fA, SOC=%d%%, MaxTemp=%d°C\n",
       voltage_V, power_W, current_A, soc, max_temp);
```

### 7.3 `0x402` 故障诊断（标准帧，DLC=8）

**发送周期**：500ms（放电模式）
**数据格式**：小端（Little-Endian）

| Byte | 位 | 含义 | 说明 |
|---|---|---|---|
| 0 | 7..4 | `BatState` | 状态码（2/3/4/5/7） |
| 0 | 3..0 | `BatAlmLv` | 告警级别（0/1/2） |
| 1 | 7..6 | `ALM_CELL_OV` | 单体过压告警 |
| 1 | 5..4 | `ALM_CELL_UV` | 单体欠压告警 |
| 1 | 3..2 | `ALM_CELL_OT` | 高温告警 |
| 1 | 1..0 | `ALM_CELL_UT` | 低温告警 |
| 2 | 7 | `ALM_CELL_LBK` | 单体电压断线 |
| 2 | 6 | `ALM_CELL_TBK` | 温度断线 |
| 2 | 5..4 | `ALM_BATT_DV` | 压差告警 |
| 2 | 3..2 | `ALM_BATT_DT` | 温差告警 |
| 2 | 1..0 | `ALM_BSU_OFFLINE` | 从控离线告警 |
| 3 | 7..6 | `ALM_CHRG_OCS` | 充电过流告警 |
| 3 | 5..4 | `ALM_DSCH_OCS` | 放电过流告警 |
| 3 | 3..2 | `ALM_CHRG_OCT` | 充电过温告警 |
| 3 | 1..0 | `ALM_DSCH_OCT` | 放电过温告警 |
| 4 | 7 | `ALM_PRECHRG_FAIL` | 预充失败 |
| 4 | 6 | `ALM_AUX_FAIL` | 辅助电源故障 |
| 4 | 5 | `ALM_HVREL_FAIL` | 高压异常 |
| 4 | 4 | `ALM_HALL_BREAK` | 霍尔断线 |
| 4 | 3 | `ALM_SAFETY_CIRCUIT_BREAK` | 安全回路断开 |
| 4 | 2 | `ALM_IMD_FAIL` | IMD 数字故障（主控冗余链） |
| 4 | 1..0 | 保留 | 未使用 |
| 5..6 | - | `ErrorRom` 低16位（小端） | 故障码位图低16位 |
| 7 | 5..0 | `BMSSlaveLife[0..5]` | 从控离线状态（1=离线） |

**解析示例**：
```c
uint8_t state = (data[0] >> 4) & 0x0F;
uint8_t alm_lv = data[0] & 0x0F;
uint8_t cell_ov = (data[1] >> 6) & 0x03;
uint8_t prechrg_fail = (data[4] >> 7) & 0x01;
uint8_t hvrel_fail = (data[4] >> 5) & 0x01;
uint16_t error_rom_lo16 = data[5] | ((uint16_t)data[6] << 8);
uint8_t imd_fail = (data[4] >> 2) & 0x01;
uint8_t slave_offline = data[7] & 0x3F;

printf("State=%d AlmLv=%d CellOV=%d PreChgFail=%d HVFail=%d IMDFail=%d ErrorRomLo16=0x%04X SlaveOffline=0x%02X\n",
       state, alm_lv, cell_ov, prechrg_fail, hvrel_fail, imd_fail, error_rom_lo16, slave_offline);
```

### 7.4 `0x18FF50E5` 充电机反馈（扩展帧，DLC≥5）

**接收条件**：充电模式
**数据格式**：大端（Big-Endian）

| Byte | 含义 | 单位 | 说明 |
|---|---|---|---|
| 0..1 | `ChrBatVoltage`（大端） | 0.1V | 充电机测量电压 |
| 2..3 | `ChrBatCurrent`（大端） | 0.1A | 充电机测量电流 |
| 4 | `ChrBatState` | - | 充电机状态（见下表） |

**ChrBatState 状态码**：
- bit0：硬件故障
- bit1：过温
- bit2：输入错误
- bit3：电池错误
- bit4：通讯超时

**发送示例**（充电机端）：
```c
// 充电机反馈：实测 570.0V, 9.5A, 正常
uint8_t data[5];
data[0] = (5700 >> 8) & 0xFF;
data[1] = 5700 & 0xFF;
data[2] = (95 >> 8) & 0xFF;
data[3] = 95 & 0xFF;
data[4] = 0x00;  // 正常
CAN_Send_ExtId(0x18FF50E5, data, 5);
```

**解析示例**（主控端）：
```c
uint16_t chr_v = ((uint16_t)data[0] << 8) | data[1];
uint16_t chr_i = ((uint16_t)data[2] << 8) | data[3];
uint8_t chr_state = data[4];

printf("Charger Feedback: %.1fV, %.1fA, State=0x%02X\n",
       chr_v / 10.0f, chr_i / 10.0f, chr_state);

if (chr_state & 0x01) printf("  - Hardware Fault\n");
if (chr_state & 0x02) printf("  - Over Temperature\n");
if (chr_state & 0x04) printf("  - Input Error\n");
if (chr_state & 0x08) printf("  - Battery Error\n");
if (chr_state & 0x10) printf("  - Communication Timeout\n");
```

---

## 8. 收发职责与状态关联

### 8.1 主控接收责任

**CAN1 接收**：处理从控电压、从控温度、霍尔电流传感器数据及参数修改命令

**CAN2 接收**：
- 充电模式：接收充电机反馈（`0x18FF50E5`）
- 放电模式：不接收

### 8.2 主控发送责任

**周期发送**：
- CAN1：状态报文（500ms）+ 均衡状态（250ms，条件）
- CAN2 充电模式：充电请求（1s）
- CAN2 放电模式：功率状态 + 故障诊断（500ms）

**事件触发发送**：
- HV_ACC 上高压完成：发送 CAN1 状态帧
- HV_ACC 下高压请求：发送停止充电命令
- ADC 校准应答：处理校准命令后发送应答
- 电流方向应答：立即应答

---

## 9. 常见问题与调试

### 9.1 字节序错误

**症状**：数值异常，如电压显示为 6553.5V 而非 655.35V

**原因**：字节序混淆（大端/小端）

**解决**：
- CAN1 主控发送：大端
- CAN1 从控发送：小端
- CAN2 `0x401/0x402`：小端
- CAN2 充电请求/反馈：大端

**验证方法**：
```c
// 大端：高字节在前
uint16_t value_be = ((uint16_t)data[0] << 8) | data[1];

// 小端：低字节在前
uint16_t value_le = data[0] | ((uint16_t)data[1] << 8);
```

### 9.2 从控电压帧映射错误

**症状**：电压数据错位或缺失

**原因**：未正确处理帧 0 的特殊映射（Byte2..7）

**解决**：严格按照 5.1.2 节的映射规则实现

**调试方法**：
- 打印接收到的 ID 和数据
- 验证 `volnum_set` 计算是否正确
- 检查是否正确判断帧 0

### 9.3 CAN 总线 Bus-Off

**症状**：CAN 通信突然中断，无法收发

**原因**：
- 波特率不匹配
- 终端电阻缺失或错误
- 总线短路或断路
- 发送失败次数过多

**解决**：
- 检查波特率配置（CAN1=250kbps, CAN2=250/500kbps）
- 确认终端电阻（120Ω）
- 启用自动离线恢复（ABOM=ENABLE）
- 检查硬件连接

### 9.4 从控离线误报

**症状**：从控实际在线但被标记为离线

**原因**：
- 从控发送周期过长
- 主控离线检测阈值过小
- CAN 总线负载过高导致丢帧

**解决**：
- 确认从控发送周期 < 1s
- 调整 `BMSSlaveLifeCnt` 阈值
- 降低总线负载

### 9.5 ADC 校准失败

**症状**：校准命令返回 `0x01`（校准点不合理）

**原因**：
- 两点电压差过小（< 50V）
- 低点电压 > 高点电压
- ADC 原始值相同

**解决**：
- 确保两点电压差 > 50V
- 低点建议 100V，高点建议 600V
- 等待电压稳定后再记录

### 9.6 充电机通信超时

**症状**：充电机报告通讯超时（`ChrBatState` bit4=1）

**原因**：
- 主控未按 1s 周期发送充电请求
- CAN2 波特率不匹配
- 充电机超时阈值过短

**解决**：
- 确认 TIM4 中断正常运行
- 检查 `TIM4_Task_Service()` 的 1s 软计数调度是否运行
- 确认 CAN2 波特率为 250kbps

**文档版本**：v2.0
**最后更新**：2026-02-27
**维护者**：武理博
