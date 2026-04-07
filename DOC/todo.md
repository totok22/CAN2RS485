## todo 
- [x] 使用 STM32CUBEMX 自动生成代码。
- [x] CAN1 和 CAN2 接收过滤器初始化。
- [x] 依据DOC/其他信息.md，REFERENCE/protobuf-master和REFERENCE/旧主控 CAN 通讯协议.md撰写修改相应代码，目前仅仅使用旧主控接受其 CAN1 发的信息并串口（485）发给 DTU。


## changelog
- 新增 `Core/Src/app.c` 业务层，完成 CAN 启动、过滤器配置、CAN1 接收解析、TelemetryFrame 编码和 RS485 发送。
- 接入 `Middlewares/Third_Party/nanopb`，将 `fsae_telemetry.pb.c/.h` 与 `pb_encode/pb_common` 纳入工程。
- `USART2` 发送 RS485 数据时，发送后等待 `TC` 置位再拉低 `RS485_DIR`。
- 当前只桥接旧主控 `CAN1` 数据到 DTU，未实现车上双 CAN 全量业务转发。
