# Grafana 使用指南

本文说明 FSAE 遥测 Grafana 的常用配置流程。数据链路为：

`STM32/DTU -> MQTT fsae/telemetry -> Telegraf -> InfluxDB(fsae_db) -> Grafana`

当前推荐入口：`https://bitfsae.com/monitor/`。旧入口：`https://bitfsae.xin/monitor/`。

## 1. 数据表

Telegraf 将同一个 `TelemetryFrame` 拆成多个 InfluxDB measurement：

| measurement | 用途 | 关键字段 |
| --- | --- | --- |
| `telemetry` | 基础遥测和 BMS 摘要 | `hv_voltage_dv`, `hv_current_ma`, `speed_kmh`, `battery_soc`, `max_cell_voltage`, `max_temp` |
| `bms_data` | 电池模组明细 | `module_id`, `v_01`~`v_23`, `t_1`~`t_8` |
| `motor_state` | 电机状态 | `position`, `rpm`, `torque_nm`, `power_w`, `motor_temp_dc`, `inverter_temp_dc` |
| `thermal_summary` | 热成像摘要 | `position`, `min_temp_centi_c`, `max_temp_centi_c`, `avg_temp_centi_c` |
| `alarm_state` | 告警 | `alarm_id`, `severity`, `message` |

常用单位约定：

| 字段 | 实际单位 |
| --- | --- |
| `hv_voltage_dv` | 0.1 V |
| `hv_current_ma` | mA |
| `battery_temp_max_dc`, `max_temp`, `min_temp`, `motor_temp_dc` | 0.1 degC |
| `v_01`~`v_23` | mV |
| `t_1`~`t_8` | 0.1 degC |
| `*_centi_c` | 0.01 degC |

## 2. 新建仪表盘

1. 左侧菜单进入 `Dashboards`。
2. 点 `New` -> `New dashboard`。
3. 点 `Add visualization`。
4. 选择 InfluxDB 数据源，通常数据库为 `fsae_db`。
5. 写查询，选择右侧可视化类型并配置字段、单位、阈值。
6. 点右上角 `Apply` 保存面板，再保存 Dashboard。

建议按页面分组：

| Dashboard/Row | 面板内容 |
| --- | --- |
| `Overview` | 高压电压、电流、SOC、车速、最高温、告警 |
| `BMS` | 6 个模组的电芯电压、温度、极值 |
| `Motor` | 电机转速、扭矩、功率、电机/控制器温度 |
| `Thermal` | 四轮热成像 min/max/avg |
| `Debug` | 原始字段、最新帧时间、seq、source_id |

## 3. 查询模式

Grafana 面板的查询编辑器可使用图形模式，也可以切到 `Raw query` 写 InfluxQL。遥测项目建议复杂面板直接使用 `Raw query`，字段和筛选条件更清晰。

基础格式：

```sql
SELECT <field>
FROM "<measurement>"
WHERE $timeFilter
GROUP BY time($__interval) fill(null)
```

最新一条数据：

```sql
SELECT <field>
FROM "<measurement>"
ORDER BY time DESC
LIMIT 1
```

按 tag 筛选，例如 BMS 模组：

```sql
SELECT v_01
FROM "bms_data"
WHERE "module_id" = '1' AND $timeFilter
```

注意：`module_id` 在 Telegraf 里作为 tag 写入，所以查询时用字符串：`'1'`。

## 4. 常用查询

### 4.1 基础遥测时序

```sql
SELECT "hv_voltage_dv" / 10
FROM "telemetry"
WHERE $timeFilter
```

```sql
SELECT "hv_current_ma" / 1000
FROM "telemetry"
WHERE $timeFilter
```

```sql
SELECT "speed_kmh"
FROM "telemetry"
WHERE $timeFilter
```

### 4.2 最新状态值

```sql
SELECT "battery_soc", "max_cell_voltage", "min_cell_voltage", "max_temp", "min_temp"
FROM "telemetry"
ORDER BY time DESC
LIMIT 1
```

### 4.3 单个模组 23 节电芯最新电压

```sql
SELECT v_01, v_02, v_03, v_04, v_05, v_06, v_07, v_08, v_09, v_10, v_11, v_12, v_13, v_14, v_15, v_16, v_17, v_18, v_19, v_20, v_21, v_22, v_23
FROM "bms_data"
WHERE "module_id" = '1'
ORDER BY time DESC
LIMIT 1
```

### 4.4 单个模组温度

```sql
SELECT t_1, t_2, t_3, t_4, t_5, t_6, t_7, t_8
FROM "bms_data"
WHERE "module_id" = '1'
ORDER BY time DESC
LIMIT 1
```

### 4.5 所有模组某一节电芯趋势

```sql
SELECT "v_01"
FROM "bms_data"
WHERE $timeFilter
GROUP BY "module_id"
```

### 4.6 电机状态

```sql
SELECT "rpm"
FROM "motor_state"
WHERE $timeFilter
GROUP BY "position"
```

```sql
SELECT "motor_temp_dc" / 10, "inverter_temp_dc" / 10
FROM "motor_state"
WHERE $timeFilter
GROUP BY "position"
```

### 4.7 告警

```sql
SELECT "severity", "message"
FROM "alarm_state"
WHERE $timeFilter
ORDER BY time DESC
LIMIT 50
```

## 5. 面板类型

右侧 `Visualization` 常用选择：

| 类型 | 适合内容 |
| --- | --- |
| `Time series` | 电压、电流、温度、车速、电机转速趋势 |
| `Stat` | 最新 SOC、最高温、当前车速、RTD 状态 |
| `Gauge` | SOC、高压电压、电池最高温 |
| `Bar chart` | 23 节电芯电压、8 路温度的最新值 |
| `Table` | 告警列表、调试字段、最新一帧原始值 |
| `State timeline` | 驾驶模式、VCU 状态、告警等级 |

## 6. 转换 Transform

进入面板编辑页下方 `Transformations`：

| 转换 | 用途 |
| --- | --- |
| `Reduce` | 把时序压成最新值、最大值、最小值、平均值 |
| `Organize fields` | 重命名字段、隐藏 `time`、调整列顺序 |
| `Labels to fields` | 将 `module_id`、`position` 这类 tag 展开成列 |
| `Merge series/tables` | 合并多个查询结果 |
| `Add field from calculation` | 做差值、比例换算、平均值 |

常用做法：

- 最新值面板：查询多字段 -> `Reduce` -> Calculation 选 `Last not null`。
- BMS 柱状图：查询 `v_01`~`v_23` 最新值 -> `Organize fields` 重命名为 `Cell 01`~`Cell 23`。
- 多电机趋势：`GROUP BY "position"` 后用 `Labels to fields` 或直接保留多 series。

## 7. 右侧字段配置

### 7.1 单位 Unit

在 `Field` -> `Standard options` -> `Unit` 设置：

| 显示值 | 查询建议 | Unit |
| --- | --- | --- |
| 高压电压 | `hv_voltage_dv / 10` | `volt` |
| 高压电流 | `hv_current_ma / 1000` | `ampere` |
| 电芯电压 | `v_01` 等 | `millivolt` |
| 温度 | `*_dc / 10` 或 `*_centi_c / 100` | `celsius` |
| 车速 | `speed_kmh` | `km/h` |
| SOC | `battery_soc` | `percent (0-100)` |

如果 Grafana 没有合适单位，可选 `Custom unit`，例如 `mV`、`km/h`、`degC`。

### 7.2 Calculation

Stat/Gauge 常用：

| Calculation | 用途 |
| --- | --- |
| `Last not null` | 最新状态，最常用 |
| `Max` | 最高温、最高电压 |
| `Min` | 最低电压 |
| `Mean` | 一段时间平均值 |
| `Range` | 电芯压差、温差 |

### 7.3 字段名 Display name

在 `Field` -> `Display name` 设置显示名。建议统一：

- `hv_voltage_dv / 10` -> `HV Voltage`
- `hv_current_ma / 1000` -> `HV Current`
- `v_01` -> `Cell 01`
- `t_1` -> `Temp 1`
- `motor_temp_dc / 10` -> `Motor Temp`

### 7.4 阈值 Thresholds

在 `Field` -> `Thresholds` 设置颜色。建议先用保守显示阈值，不替代 BMS/VCU 的真实故障判定：

| 面板 | Green | Yellow | Red |
| --- | --- | --- | --- |
| 单体电压 mV | `>= 3300` | `< 3300` | `< 3000` |
| 电池温度 degC | `< 50` | `50~60` | `>= 60` |
| 高压电压 V | 正常工作范围 | 偏低/偏高 | 异常 |
| SOC % | `> 30` | `15~30` | `< 15` |
| 告警 severity | `0~1` | `2` | `>= 3` |

## 8. 变量 Variables

Dashboard 变量适合做模组/电机筛选。

创建位置：`Dashboard settings` -> `Variables` -> `New variable`。

模组变量：

```sql
SHOW TAG VALUES FROM "bms_data" WITH KEY = "module_id"
```

变量名设为 `module_id`，查询中使用：

```sql
SELECT v_01, v_02, v_03, v_04, v_05, v_06, v_07, v_08, v_09, v_10, v_11, v_12, v_13, v_14, v_15, v_16, v_17, v_18, v_19, v_20, v_21, v_22, v_23
FROM "bms_data"
WHERE "module_id" = '$module_id'
ORDER BY time DESC
LIMIT 1
```

电机位置变量：

```sql
SHOW TAG VALUES FROM "motor_state" WITH KEY = "position"
```

## 9. 推荐面板配置

### BMS 单模组电芯电压柱状图

- Query：使用第 4.3 节查询。
- Visualization：`Bar chart`。
- Transform：`Organize fields`，隐藏 `time`，字段改名为 `Cell 01`~`Cell 23`。
- Unit：`millivolt`。
- Calculation：`Last not null`。
- Thresholds：`3000` 红，`3300` 黄，正常绿。

### 基础遥测总览

- Query：分别查询 `hv_voltage_dv / 10`、`hv_current_ma / 1000`、`speed_kmh`、`battery_soc`。
- Visualization：`Stat` 或 `Gauge`。
- Calculation：`Last not null`。
- Unit：按第 7.1 节设置。

### 告警表

- Query：使用第 4.7 节查询。
- Visualization：`Table`。
- Transform：`Organize fields`，保留 `time`, `alarm_id`, `severity`, `message`。
- Thresholds：对 `severity` 设置 `2` 黄、`3` 红。

## 10. 排查

| 现象 | 检查点 |
| --- | --- |
| Grafana 无数据 | 时间范围是否太短；InfluxDB 数据源是否指向 `fsae_db`；Telegraf 是否运行 |
| 查询能出数据但面板空白 | 可视化类型是否合适；字段是否被 Transform 隐藏；单位/计算是否误设 |
| BMS 查询为空 | `bms_data` 只在 `TelemetryFrame.modules` 上送时产生，频率低于基础遥测 |
| 按 `module_id` 查不到 | `module_id` 是 tag，条件写 `"module_id" = '1'`，不要写成数字 |
| 字段显示为多个 series | 使用 `Organize fields` 或 `Labels to fields` 整理 |
| 数值大 10 或 100 倍 | 检查字段单位，`*_dc` 需 `/ 10`，`*_centi_c` 需 `/ 100` |

服务器侧快速验证：

```bash
docker exec -it fsae_influxdb influx -database fsae_db -execute 'SHOW MEASUREMENTS'
docker exec -it fsae_influxdb influx -database fsae_db -execute 'SHOW FIELD KEYS FROM "telemetry"'
docker exec -it fsae_influxdb influx -database fsae_db -execute 'SELECT * FROM "bms_data" LIMIT 5'
```
