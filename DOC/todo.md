## todo 
- [x] 使用 STM32CUBEMX 自动生成代码。
- [x] CAN1 和 CAN2 接收过滤器初始化。
- [x] 依据DOC/其他信息.md，REFERENCE/protobuf-master和REFERENCE/旧主控 CAN 通讯协议.md撰写修改相应代码，目前仅仅使用旧主控接受其 CAN1 发的信息并串口（485）发给 DTU。


## changelog
- 新增 `Core/Src/app.c` 业务层，完成 CAN 启动、过滤器配置、CAN1 接收解析、TelemetryFrame 编码和 RS485 发送。
- 接入 `Middlewares/Third_Party/nanopb`，将 `fsae_telemetry.pb.c/.h` 与 `pb_encode/pb_common` 纳入工程。
- `USART2` 发送 RS485 数据时，发送后等待 `TC` 置位再拉低 `RS485_DIR`。
- 当前只桥接旧主控 `CAN1` 数据到 DTU，未实现车上双 CAN 全量业务转发。
- 重构 `Core/Src/app.c` 的接收路径，按 `CAN1/CAN2` 分流解析，并补充 `CAN2` 被动监听：
  - `0x18FF50E5`
  - `0x401`
  - `0x402`
- 修复发送路径的大对象栈占用问题，改为静态发送快照和静态 `TelemetryFrame` 编码缓冲。
- 修复旧主控温度帧兼容性：从控 3、4 按旧协议只取 7 路有效温度；检测到新主控专用帧后切换到新协议映射。
- 修复故障字覆盖问题：告警故障、霍尔故障、IMD 故障改为组合更新。
- 新增协议兼容框架，为未来切换新主控保留入口，但当前仍以旧主控链路稳定性优先。

## remaining
- [ ] 若未来要完整支持新主控工具链，还需实现 `0x18A*` / `0x188350F5` 等命令的业务语义和应答帧。
- [ ] 若未来要做 CAN2 主动对接 VCU/充电机，还需明确本板在车上是“旁路监听”还是“参与总线发送”。
- [ ] 若未来需要把 IMD 诊断、CellVoltCal、更多 CAN2 故障细节上云，需要同步修改 `.proto`、nanopb 生成代码和云端 `telegraf.conf`。
