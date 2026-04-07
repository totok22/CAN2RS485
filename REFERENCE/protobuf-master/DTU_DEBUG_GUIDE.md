# DTU 联调指南

本文用于完成 `进展.md` 中新增的 USB 转 485 + `local_sim2.py` 联调任务。

## 1. 目标

在没有车端 `CAN -> 485` 板子的情况下，用电脑完成以下链路验证：

`local_sim2.py` -> `USB 转 485` -> `DTU 串口` -> `MQTT` -> `服务器` -> `Grafana`

## 2. 接线

典型接法：

*   USB 转 485 `A` -> DTU `485A`
*   USB 转 485 `B` -> DTU `485B`
*   如果模块标识是 `D+ / D-`，按设备说明书对应到 `A / B`
*   DTU 单独供电，电脑 USB 只给 USB 转 485 模块供电

如果收不到数据，优先尝试对调 `A/B`。

## 3. DTU 建议配置

结合现有项目，建议先用下面这组参数联调：

*   MQTT 模式
*   Broker: `123.57.174.98:1883`
*   Topic: `fsae/telemetry`
*   MQTT 版本: `3.1.1`
*   用户名/密码: 空
*   串口波特率: `115200`
*   数据位: `8`
*   停止位: `1`
*   校验位: `None`

串口打包建议：

*   优先使用“空闲时间切包”或“超时切包”
*   如果 DTU 必须依赖结束符切包，则在脚本里添加 `--packet-suffix-hex 0D0A`

## 4. 运行方法

只走 DTU：

```bash
python local_sim2.py --mode serial --serial-port COM3 --baudrate 115200
```

串口和 MQTT 同时发，便于对照：

```bash
python local_sim2.py --mode both --serial-port COM3 --baudrate 115200
```

如果 DTU 依赖回车换行切包：

```bash
python local_sim2.py --mode serial --serial-port COM3 --baudrate 115200 --packet-suffix-hex 0D0A
```

## 5. 推荐联调顺序

1. 先用 `--mode both` 跑起来，确认脚本自身还在稳定出数据
2. 再看网站/Grafana 是否持续有数据
3. 如果直连 MQTT 有数据，但 DTU 路径没有数据，优先排查 DTU 串口参数和切包方式
4. 如果 DTU 路径偶尔有数据但不稳定，优先降低 BMS 发送频率，或者给 DTU 增加明确包尾

## 6. 排查重点

### 现象：DTU 完全无上报

检查：

*   `A/B` 是否接反
*   DTU 串口波特率、校验位、停止位是否一致
*   DTU 是否真的工作在 MQTT 模式
*   Topic 是否还是 `fsae/telemetry`

### 现象：服务器有 `invalid wire-format`

说明 DTU 上送的数据边界不对，常见原因：

*   DTU 把两帧 Protobuf 粘在一起发
*   DTU 把一帧拆开了
*   DTU 增加了额外头尾字节

优先处理：

*   调整 DTU 的串口打包超时/长度参数
*   或在脚本中增加 `--packet-suffix-hex`

### 现象：MQTT 有数据，但 `bms_data` 很少

这是正常的，`modules` 目前按 2Hz 刷新，不是每帧都带。
