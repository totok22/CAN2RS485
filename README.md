# CAN2RS485

基于 `STM32F405RGT6` 的车载遥测转接板。

主链路：

`CAN -> TelemetryFrame -> USART2/RS485 -> DTU`

## 当前状态

- 已实现旧主控 `CAN1` 主链路。
- 已补 `CAN2` 被动监听：`0x18FF50E5`、`0x401`、`0x402`。
- 当前收到 `CAN1` 或 `CAN2` 任一总线的有效业务数据后，开始周期上报。
- 基础帧 100 ms，带 `modules` 的帧 500 ms。
- `USART2` 严格按 `RS485_DIR -> UART_Transmit -> 等待 TC -> 拉低 DIR` 时序发送。
- 代码默认兼容旧主控；检测到新主控专用帧后切换到新协议分支。
- 状态灯：未完成有效上报时慢闪，成功接收 CAN 并发出遥测后常亮，故障快闪。
- USART1 调试 CLI 默认关闭；打开 `APP_DEBUG_CLI_ENABLE` 后可通过串口打印最近 CAN 帧。
- CAN RX 中断单次最多处理 8 帧，避免长时间占用中断。
- 电池模组数量、电芯数、温度点数和 protobuf `modules` 容量有编译期断言保护。

## 状态灯

- 引脚：`PC13`，高边驱动，高电平点亮；初始化默认低电平熄灭。
- 未完成有效上报：每 `500 ms` 翻转一次。
- 已收到有效 CAN 且成功发出遥测：常亮。
- 故障：每 `100 ms` 翻转一次。
- 故障来源：当前有效鲜度窗口内的 BMS 报警、霍尔错误、CAN2 诊断故障。

## USART1 调试 CLI

- 默认关闭：`APP_DEBUG_CLI_ENABLE = 0U`，不启动 USART1 接收中断。
- 启用方式：改为 `APP_DEBUG_CLI_ENABLE = 1U` 后重新编译烧录。
- 串口：`USART1`，`PA9/PA10`，`115200 8N1`。
- 输入：发送任意非空 ASCII 命令并以 `\r` 或 `\n` 结束，例如 `dump` 后回车。
- 行长：最多 `31` 字节；超长输入会丢弃本行。
- 行为：当前不解析具体命令内容，任意命令都会打印最近 CAN 帧。
- 输出：最多打印最近 `3` 帧；缓存深度为 `8` 帧。
- 注意：CLI 输出使用阻塞式 `HAL_UART_Transmit()`，只用于现场调试，不建议常开。

## 当前未完成项

- 未实现 `0x18A*`、`0x188350F5` 等新主控工具命令闭环。
- 未实现 CAN2 主动转发/发送，仅做被动监听与上云聚合。
- 未扩展新主控全部诊断字段到 `TelemetryFrame`。

## 构建

```bash
cmake --preset Debug
cmake --build --preset Debug
```

## 参考文档

- [`DOC/电路信息.md`](./DOC/%E7%94%B5%E8%B7%AF%E4%BF%A1%E6%81%AF.md)
- [`DOC/todo.md`](./DOC/todo.md)
- [`REFERENCE/旧主控 CAN 通讯协议.md`](./REFERENCE/%E6%97%A7%E4%B8%BB%E6%8E%A7%20CAN%20%E9%80%9A%E8%AE%AF%E5%8D%8F%E8%AE%AE.md)
- [`REFERENCE/新主控 CAN 通讯协议.md`](./REFERENCE/%E6%96%B0%E4%B8%BB%E6%8E%A7%20CAN%20%E9%80%9A%E8%AE%AF%E5%8D%8F%E8%AE%AE.md)
- [`REFERENCE/protobuf-master/STM32_GUIDE.md`](./REFERENCE/protobuf-master/STM32_GUIDE.md)
