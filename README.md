# CAN2RS485

基于 `STM32F405RGT6` 的车载遥测转接板。

当前主目标是稳定完成：

`CAN -> 状态聚合 -> nanopb/Protobuf(TelemetryFrame) -> USART2/RS485 -> DTU`

## 当前能力

- 已支持旧主控 `CAN1` 关键报文采集：
  - 单体电压 `0x180050F3 + (n<<16)`
  - 单体温度 `0x184050F3 + (i<<16)`
  - 总览 `0x186050F4`
  - 电压极值 `0x186150F4`
  - 温度极值 `0x186250F4`
  - 状态 `0x186350F4`
  - 告警 `0x187650F4`
  - 霍尔电流 `0x03C0`
- 已补充 `CAN2` 被动解析：
  - 充电机反馈 `0x18FF50E5`
  - 放电功率状态 `0x401`
  - 新主控故障诊断 `0x402`
- 遥测发送策略：
  - 基础 `TelemetryFrame`：100 ms
  - 携带 `modules`：500 ms
- `USART2` 保持 RS485 严格时序：
  - 发前拉高 `RS485_DIR`
  - `HAL_UART_Transmit()` 返回后继续等待 `UART_FLAG_TC`
  - 最后一个停止位发完再拉低方向脚

## 旧主控 / 新主控兼容策略

- `app.c` 当前按“旧主控优先”实现，默认采用旧主控温度映射。
- 看到新主控专用帧后会切换到新协议分支：
  - `0x186750F4`
  - `0x186850F4`
  - `0x402`
  - `0x188350F5`
  - `0x18A050F5`
  - `0x18A150F5`
  - `0x18A350F5`
- 这样可以兼容当前旧主控车，又为后续切到新主控保留入口，不需要再把 `app.c` 整体推倒重来。

## 已修复的关键问题

- 发送路径不再在栈上构造整份 `TelemetryFrame`，避免主栈 `0x400` 下的大对象风险。
- 旧主控温度帧中从控 3、4 的 7 路特殊映射已恢复。
- 告警故障字与霍尔/IMD 附加故障位改为组合更新，不再被后续帧直接覆盖。
- CAN1/CAN2 接收入口已按总线分流，避免把 CAN2 数据误按 CAN1 协议解析。
- 电压帧、温度帧、CAN2 状态帧增加最小 DLC 防御。

## 当前未完成项

- 未实现 CAN 参数工具命令的完整业务闭环：
  - 阈值保存
  - RTC 校时
  - ADC 校准
  - 应答帧发送
- 未实现 CAN2 主动转发/发送，仅做被动监听与上云聚合。
- `TelemetryFrame` 结构仍以当前云端消费字段为核心，未扩展新主控全部诊断细节。

## 构建

```bash
cmake --preset Debug
cmake --build --preset Debug
```

## 参考文档

- [`DOC/todo.md`](./DOC/todo.md)
- [`DOC/其他信息.md`](./DOC/其他信息.md)
- [`REFERENCE/旧主控 CAN 通讯协议.md`](./REFERENCE/%E6%97%A7%E4%B8%BB%E6%8E%A7%20CAN%20%E9%80%9A%E8%AE%AF%E5%8D%8F%E8%AE%AE.md)
- [`REFERENCE/新主控 CAN 通讯协议.md`](./REFERENCE/%E6%96%B0%E4%B8%BB%E6%8E%A7%20CAN%20%E9%80%9A%E8%AE%AF%E5%8D%8F%E8%AE%AE.md)
- [`REFERENCE/protobuf-master/STM32_GUIDE.md`](./REFERENCE/protobuf-master/STM32_GUIDE.md)
