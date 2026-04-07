# FSAE 遥测系统

BITFSAE 赛车遥测项目。车载 STM32 读取 CAN 数据，用 **Nanopb**（Protobuf 轻量实现）序列化后经 RS485 送入 DTU，由 DTU 通过 **MQTT** 无线上传至阿里云服务器，Grafana 实时可视化。

```
STM32(CAN) --RS485--> DTU --MQTT/4G--> 阿里云
                                          |
                               Mosquitto -> Telegraf -> InfluxDB -> Grafana
```

> 当前状态（2025-12）：本地模拟脚本已验证服务器链路，Grafana 可见数据流；车端 CAN→485 板尚未就绪，用 USB 转 485 + `local_sim2.py` 替代测试 DTU。

---

## 文件/目录说明

### 根目录（主要文件）

| 文件 | 作用 |
|------|------|
| `fsae_telemetry_pb2.py` | 由 `protoc` 自动生成的 Python Protobuf 绑定，供 `local_sim2.py` 调用，**勿手动修改** |
| `local_sim2.py` | 本地模拟脚本，模拟赛车数据并支持三种输出模式：`mqtt`（直发服务器）、`serial`（经 USB→485 喂 DTU）、`both`（两路同时，便于对照） |

### 文档

| 文件 | 内容摘要 |
|------|----------|
| `README.md` | 本文件 |
| `进展.md` | 开发日志与 TODO 看板 |
| `其他信息.md` | 项目背景、DTU 型号（USR-DR154）与配置参数（IP、端口、Topic 等） |
| `服务器文件.md` | 服务器端部署说明，含 `docker-compose.yml` 全文及 Telegraf XPath 解析说明；**以 GitHub 仓库 `bitfsae_web/legacy-aliyun-xin` 分支为准** |
| `PROTO_GUIDE.md` | Protobuf 协议维护规范：禁止改已有字段名/ID、新增字段方法、Telegraf 同步更新要求 |
| `STM32_GUIDE.md` | STM32 端集成指南：Nanopb 库使用、`.pb.c/.h` 重新生成命令、工程依赖说明 |
| `DTU_DEBUG_GUIDE.md` | USB 转 485 + DTU 联调指南：接线、串口参数配置、推荐调试顺序、常见故障排查 |
| `README_local_sim.md` | `local_sim2.py` 详细使用说明：依赖安装、配置区域、各运行模式命令示例 |
| `AGENTS.md` | AI Agent 行为约束（当前为空，预留） |

---

### `stm32_code/` — 车载端代码

STM32 工程中需要引用的全部文件，直接集成进 Keil/CubeIDE 工程即可。

```
stm32_code/
├── fsae_telemetry.proto      # 数据结构定义源文件
├── fsae_telemetry.options    # Nanopb 配置（repeated 字段 max_count 等）
├── fsae_telemetry.pb.c/.h   # 由 proto+options 自动生成，修改 proto 后需重新生成
└── nanopb_lib/               # Nanopb 核心库（pb.h / pb_encode / pb_decode / pb_common）
```

> 重新生成命令（参见 `STM32_GUIDE.md`）：
> ```bash
> python "D:\EdgeDo\nanopb-0.4.9.1-windows-x86\...\nanopb_generator.py" fsae_telemetry.proto
> ```

---

### `server_config/` — 服务器端配置

对应阿里云服务器 `/home/admin/fsae_project` 目录内容，**以远程仓库为最新权威**，本地副本可能过时。

```
server_config/
├── docker-compose.yml        # 五个容器：Mosquitto / InfluxDB / Telegraf / Grafana / Nginx
├── protos/
│   ├── fsae_telemetry.proto  # 服务器端 proto（Telegraf 解码用，需与车端保持一致）
│   └── fsae_telemetry.options
└── telegraf/
    └── telegraf.conf         # MQTT 订阅 + xpath_protobuf 解析 + InfluxDB 写入配置
```

关键约束：`telegraf.conf` 中的 XPath 映射与 `.proto` 字段名强绑定，修改字段名后**必须**同步更新并重启 Telegraf 容器。

---

### `REFERENCE/` — 参考资料

旧版电箱主控（BMS Master）的 STM32F10x 完整工程备份，用于参考原有 CAN 报文结构和 BMS 数据字段定义。包含原理图 PCB、Keil 工程源码及编译产物，**不参与当前遥测系统构建**。

---

## 快速上手

**测试服务器链路（纯 MQTT）：**
```bash
pip install paho-mqtt protobuf pyserial
python local_sim2.py
```

**联调 DTU（USB 转 485）：**
```bash
python local_sim2.py --mode serial --serial-port COM3 --baudrate 115200
```

**同时验证两条链路：**
```bash
python local_sim2.py --mode both --serial-port COM3 --baudrate 115200
```

服务器地址：`123.57.174.98:1883`，Topic：`fsae/telemetry`，Grafana：`https://bitfsae.xin/monitor/`
