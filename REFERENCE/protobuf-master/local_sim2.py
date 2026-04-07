import argparse
import random
import time

import paho.mqtt.client as mqtt

import fsae_telemetry_pb2 as pb  # 导入刚才生成的库

try:
    import serial
except ImportError:
    serial = None

# ================= 配置区域 =================
# 阿里云服务器的公网 IP
SERVER_IP = "123.57.174.98" 
SERVER_PORT = 1883
TOPIC_TELEMETRY = "fsae/telemetry"
SERIAL_PORT = "COM3"
SERIAL_BAUDRATE = 115200
SERIAL_BYTESIZE = 8
SERIAL_STOPBITS = 1
SERIAL_PARITY = "N"
SERIAL_TIMEOUT = 1.0
SERIAL_SUFFIX_HEX = ""

# 发送频率设置
BASE_FREQ = 10.0           # 基础频率 10Hz
LOOP_INTERVAL = 1.0 / BASE_FREQ 
BMS_DIVIDER = 5            # 10Hz / 5 = 2Hz

# ===========================================

START_TIMESTAMP = time.time()

def get_current_time_ms():
    # 2. 修改这里：计算当前时间与启动时间的差值 (模拟单片机的 HAL_GetTick)
    diff = time.time() - START_TIMESTAMP
    return int(diff * 1000)

class CarSimulator:
    def __init__(self):
        # 初始化车辆状态 (用于模拟连续变化的数值)
        self.rpm = 0
        self.speed = 0
        self.apps = 0 # 油门
        self.brake = 0 # 刹车
        self.hv_voltage = 380.0
        self.hv_current = 0
        self.motor_temp = 40.0
        self.state = "IDLE" # IDLE, ACCEL, BRAKE, COAST
        self.frame_count = 0
        self.module_snapshots = []

    def _refresh_bms_cache(self):
        modules = []
        for i in range(6):
            base_vol = 4000 + (i * 10)
            voltages = [int(base_vol + random.randint(-15, 15)) for _ in range(23)]
            temps = [350 + i * 5 + random.randint(-5, 5) for _ in range(8)]
            modules.append({
                "module_id": i + 1,
                "voltages": voltages,
                "temps": temps,
            })
        self.module_snapshots = modules

    def update_physics(self):
        """模拟物理变化，让曲线看起来真实"""
        # 1. 随机切换驾驶状态
        if random.random() < 0.05: # 5%概率改变状态
            self.state = random.choice(["ACCEL", "BRAKE", "COAST", "ACCEL"])
        
        # 2. 根据状态更新数据
        if self.state == "ACCEL":
            self.apps = min(self.apps + 5, 100)
            self.brake = max(self.brake - 10, 0)
            self.rpm = min(self.rpm + 200 + random.randint(-50, 50), 12000)
            self.hv_current = (self.apps / 100) * 200 # 电流跟油门走
        
        elif self.state == "BRAKE":
            self.apps = max(self.apps - 10, 0)
            self.brake = min(self.brake + 10, 80)
            self.rpm = max(self.rpm - 400, 0)
            self.hv_current = -20 # 动能回收模拟
            
        elif self.state == "COAST":
            self.apps = max(self.apps - 5, 0)
            self.brake = 0
            self.rpm = max(self.rpm - 100, 0)
            self.hv_current = 5 # 待机电流

        # 3. 模拟温度缓慢上升
        if self.rpm > 5000:
            self.motor_temp += 0.05
        else:
            self.motor_temp = max(self.motor_temp - 0.02, 30)

        # 4. 电压随负载波动
        self.hv_voltage = 380.0 - (self.hv_current * 0.05) + random.uniform(-0.1, 0.1)

    def generate_frame(self):
        """生成单 Topic 遥测帧；基础信息 10Hz，BMS 详细数据 2Hz 刷新一次"""
        self.update_physics()
        self.frame_count += 1
        include_bms = (self.frame_count % BMS_DIVIDER == 0)
        if include_bms or not self.module_snapshots:
            self._refresh_bms_cache()

        frame = pb.TelemetryFrame()
        
        # --- 10Hz 数据填充 ---
        frame.timestamp_ms = get_current_time_ms()
        frame.frame_id = self.frame_count
        frame.apps_position = self.apps
        frame.brake_pressure = self.brake
        frame.steering_angle = random.uniform(-45, 45) # 转向角随机摆动
        frame.motor_rpm = int(self.rpm)
        frame.hv_voltage = self.hv_voltage
        frame.hv_current = self.hv_current
        frame.motor_temp = self.motor_temp
        frame.inverter_temp = self.motor_temp - 5
        frame.battery_temp_max = max(module["temps"][0] for module in self.module_snapshots) / 10.0
        frame.ready_to_drive = 1  
        frame.vcu_status = 1 if self.rpm > 0 else 0 # 模拟一下：有转速时 VCU 才算开启
        all_voltages = []
        all_temps = []
        for module_data in self.module_snapshots:
            all_voltages.extend(module_data["voltages"])
            all_temps.extend(module_data["temps"])

        max_voltage = max(all_voltages)
        min_voltage = min(all_voltages)
        max_voltage_index = all_voltages.index(max_voltage) + 1
        min_voltage_index = all_voltages.index(min_voltage) + 1
        max_temp = max(all_temps)
        min_temp = min(all_temps)
        max_temp_index = all_temps.index(max_temp) + 1
        min_temp_index = all_temps.index(min_temp) + 1

        frame.battery_soc = max(0, min(100, int((self.hv_voltage - 320.0) / 0.7)))
        frame.max_cell_voltage = max_voltage
        frame.min_cell_voltage = min_voltage
        frame.max_cell_voltage_no = max_voltage_index
        frame.min_cell_voltage_no = min_voltage_index
        frame.max_temp = max_temp
        frame.min_temp = min_temp
        frame.max_temp_no = max_temp_index
        frame.min_temp_no = min_temp_index
        frame.battery_fault_code = 0
        if min_voltage < 3900:
            frame.battery_fault_code |= 0x01
        if max_temp > 500:
            frame.battery_fault_code |= 0x02
        if self.hv_current < -10:
            frame.battery_fault_code |= 0x04

        if include_bms:
            for module_data in self.module_snapshots:
                module = frame.modules.add()
                module.module_id = module_data["module_id"]
                for j, val in enumerate(module_data["voltages"], start=1):
                    setattr(module, f"v{j:02d}", val)
                for k, temp in enumerate(module_data["temps"], start=1):
                    setattr(module, f"t{k}", temp)

        return frame


def parse_args():
    parser = argparse.ArgumentParser(
        description="FSAE telemetry simulator: send protobuf frames via MQTT, RS-485 serial, or both."
    )
    parser.add_argument(
        "--mode",
        choices=["mqtt", "serial", "both"],
        default="mqtt",
        help="mqtt: direct to broker; serial: write protobuf to USB-RS485; both: send to both paths.",
    )
    parser.add_argument("--server-ip", default=SERVER_IP)
    parser.add_argument("--server-port", type=int, default=SERVER_PORT)
    parser.add_argument("--topic", default=TOPIC_TELEMETRY)
    parser.add_argument("--serial-port", default=SERIAL_PORT)
    parser.add_argument("--baudrate", type=int, default=SERIAL_BAUDRATE)
    parser.add_argument("--bytesize", type=int, default=SERIAL_BYTESIZE)
    parser.add_argument("--stopbits", type=float, default=SERIAL_STOPBITS)
    parser.add_argument("--parity", default=SERIAL_PARITY, choices=["N", "E", "O", "M", "S"])
    parser.add_argument("--serial-timeout", type=float, default=SERIAL_TIMEOUT)
    parser.add_argument(
        "--packet-suffix-hex",
        default=SERIAL_SUFFIX_HEX,
        help="Optional hex suffix appended to every serial packet, e.g. 0A or 0D0A.",
    )
    return parser.parse_args()


def open_mqtt_client(args):
    print(f"Connecting to MQTT Broker: {args.server_ip}:{args.server_port}...")
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.connect(args.server_ip, args.server_port, 60)
    print(f"MQTT connected. Topic: {args.topic}")
    return client


def open_serial_port(args):
    if serial is None:
        raise RuntimeError("pyserial is not installed. Run: pip install pyserial")
    print(
        f"Opening serial port: {args.serial_port} | {args.baudrate},{args.bytesize},{args.parity},{args.stopbits}"
    )
    ser = serial.Serial(
        port=args.serial_port,
        baudrate=args.baudrate,
        bytesize=args.bytesize,
        parity=args.parity,
        stopbits=args.stopbits,
        timeout=args.serial_timeout,
    )
    print("Serial connected.")
    return ser


def build_serial_packet(frame, packet_suffix_hex):
    payload = frame.SerializeToString()
    if packet_suffix_hex:
        payload += bytes.fromhex(packet_suffix_hex)
    return payload


def main():
    args = parse_args()
    client = None
    ser = None

    try:
        if args.mode in ("mqtt", "both"):
            client = open_mqtt_client(args)
        if args.mode in ("serial", "both"):
            ser = open_serial_port(args)
    except Exception as e:
        print(f"Initialization failed: {e}")
        return

    print("Starting simulation...")
    print(f"Mode: {args.mode} | Base Freq: {BASE_FREQ}Hz | BMS Freq: {BASE_FREQ/BMS_DIVIDER}Hz")

    sim = CarSimulator()

    try:
        while True:
            start_time = time.time()

            frame = sim.generate_frame()

            if client is not None:
                client.publish(args.topic, frame.SerializeToString())

            if ser is not None:
                serial_payload = build_serial_packet(frame, args.packet_suffix_hex)
                ser.write(serial_payload)
                ser.flush()

            if len(frame.modules) > 0:
                extra = ""
                if ser is not None:
                    extra = f" | serial_bytes: {len(serial_payload)}"
                print(f"Sent merged telemetry+BMS frame @ {frame.timestamp_ms}{extra}")

            # 打印日志 (每 10 帧打印一次，避免刷屏)
            if sim.frame_count % 10 == 0:
                print(f"ID: {frame.frame_id:05d} | State: {sim.state:5s} | RPM: {frame.motor_rpm:5d} | SOC: {frame.battery_soc:3d}%")

            # 精确控制频率
            elapsed = time.time() - start_time
            sleep_time = max(0, LOOP_INTERVAL - elapsed)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nSimulation stopped.")
        if client is not None:
            client.disconnect()
        if ser is not None and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
