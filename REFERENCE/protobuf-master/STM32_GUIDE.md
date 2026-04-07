# STM32 车载发送端开发指南



我们的遥测系统使用 **Google Protobuf** 协议对数据进行序列化，但在单片机（STM32）上，我们使用 **Nanopb** 这个轻量级库。

## 1. 文件位置

已将需要的所有文件整理在 `stm32_code` 目录下。

*   `fsae_telemetry.proto`: 数据定义源文件
*   `fsae_telemetry.options`: Nanopb 配置文件
*   `fsae_telemetry.pb.c` / `.h`: C 代码（更改数据定义`.proto`或`.options`后需要重新生成）
*   `nanopb_lib/`:包含了 Nanopb 的核心库文件 (`pb_encode.c`, `pb_common.c` 等)

## 2. 如何生成 STM32 代码 (如果需要更新)

如果你修改了 `.proto` 文件，你需要使用 Nanopb 提供的生成器脚本重新生成 C 代码。

### 准备环境
确保你下载了 [Nanopb](https://jpa.kapsi.fi/nanopb/)，将生成器目录加入 PATH，或者直接调用 `nanopb_generator.py`。

### 生成命令
在 `protos` 目录下运行命令行：

```bash
# 假设你已经安装了 nanopb_generator
nanopb_generator fsae_telemetry.proto
```

或者使用 Python 运行（如果你有 Python 环境和 protobuf 库）：

```bash
python path/to/nanopb/generator/nanopb_generator.py fsae_telemetry.proto
```

当前机器目录（macOS）：

```bash
python /Users/poli/nanopb-0.4.9.1-macosx-x86/generator/nanopb_generator.py fsae_telemetry.proto
```





### 产物

执行成功后，你会得到：
*   `fsae_telemetry.pb.c`
*   `fsae_telemetry.pb.h`

当前 `CAN2RS485` 工程已将生成后的文件放在 `Middlewares/Third_Party/nanopb/` 下参与构建；如果重新生成，请同步替换工程中的对应文件。

## 3. 工程依赖

除了上述生成的两个文件，你的 STM32 工程还需要 Nanopb 的核心库文件（这些文件一般不需要更新，除非升级 Nanopb 版本）：
*   `pb.h`
*   `pb_common.c`
*   `pb_common.h`
*   `pb_encode.c`
*   `pb_encode.h`
*   `pb_decode.c` (如果车上也需要接收指令，则需要这个；如果只发送，可以不需要)
*   `pb_decode.h`

当前工程只发送遥测，已接入：
*   `pb_common.c/.h`
*   `pb_encode.c/.h`
*   `pb.h`
*   `fsae_telemetry.pb.c/.h`

## 4. 编程注意事项 (Critical)

1. **数组长度**:
   在 `.proto` 中定义的 `repeated` 字段（如 `modules`），在 C 语言中会被生成为结构体数组。
   Nanopb 会使用 `.options` 文件中的 `max_count` 来静态分配内存。
   *   例如：`fsae.TelemetryFrame.modules max_count:6`
   *   代码中必须设置 `frame.modules_count` 来告诉编码器实际有多少个有效数据。
   ```c
   TelemetryFrame frame = TelemetryFrame_init_zero;
   frame.modules_count = 6; // 必须设置！不能超过 6
   ```
   当前协议为**单 Topic**设计，建议将基础遥测和 `modules` 一起编码进同一个 `TelemetryFrame` 后发送到 `fsae/telemetry`。

2. **字符串**:
   如果有 string 类型，Nanopb 也会生成 `char array[SIZE]`。同样需要在 `.options` 中指定 `max_size`。目前我们的定义里应该没有 string，主要是数值。

> ​	注： 推荐不要新加数据 或者字符串型或者是布尔类型的，可能会导致服务器端报错，普通的数值型即可。

1.  **不要手动修改生成的 .c/.h**:
    每次 `.proto` 更新，都应该重新生成，而不是手动去改 C 代码，否则下次更新会被覆盖。

## 5. 当前 CAN2RS485 工程做法

*   输入：旧主控 `CAN1`
*   输出：`USART2` 半双工 RS485 到 DTU
*   数据源：
    *   从控电压帧 `0x180050F3 + (n<<16)`
    *   从控温度帧 `0x184050F3 + (i<<16)`
    *   旧主控摘要帧 `0x186050F4 / 0x186150F4 / 0x186250F4 / 0x186350F4 / 0x187650F4`
    *   霍尔电流帧 `0x03C0`
*   发送策略：
    *   基础 `TelemetryFrame` 10Hz
    *   带 `modules` 的完整帧 2Hz
*   485 方向控制：
    *   发送前拉高 `RS485_DIR`
    *   `HAL_UART_Transmit()` 返回后继续等待 `UART_FLAG_TC`
    *   最后一个停止位完成后再拉低 `RS485_DIR`

## 6. 示例代码片段

```c
#include "pb_encode.h"
#include "fsae_telemetry.pb.h"

void send_telemetry() {
    uint8_t buffer[512];
    TelemetryFrame message = TelemetryFrame_init_zero;

    // 1. 填充数据
    message.timestamp_ms = HAL_GetTick();
    message.frame_id = frame_counter++;
    message.apps_position = get_apps_pedal(); // float
    message.motor_rpm = get_motor_rpm();      // int32
    
    // ... 填充其他 ...

    // 2. 序列化
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    bool status = pb_encode(&stream, TelemetryFrame_fields, &message);

    if (!status) {
        // encoding failed
        printf("Encoding failed: %s\n", PB_GET_ERROR(&stream));
        return;
    }

    // 3. 发送 (buffer, stream.bytes_written)
    mqtt_publish("fsae/telemetry", buffer, stream.bytes_written);
}

```





如果未来发现流量过大，可以将 `.proto` 中的 `float` 改为 `int32`，单位使用**毫伏(mV)** 和 **0.1摄氏度**。这样配合 Protobuf 的 Varint 编码（小整数占用字节少），可以压缩一半左右的体积。但目前的 `float` 方案实现最简单，建议先用着。

### 下一步建议

为了确保你的 STM32 不会因为栈溢出（Stack Overflow）而崩溃，建议你检查 `send_telemetry` 函数中 `uint8_t buffer[1024]` 的定义位置。如果这是在一个任务栈很小的 FreeRTOS 任务中定义的局部变量，**请将其改为全局变量 (`static uint8_t buffer[1024]`)** 或者增大该任务的栈空间。

另外，考虑 DTU 带宽限制，当前 `TelemetryFrame` 的 BMS 摘要字段只保留：

*   `battery_soc`
*   `max_cell_voltage` / `min_cell_voltage` 及其编号
*   `max_temp` / `min_temp` 及其编号
*   `battery_fault_code`
