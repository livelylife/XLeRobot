import serial
import time

PORT = "/dev/ttyACM0"
BAUD = 1000000
# 我们关注的疑似轮子 ID
TARGET_IDS = [7, 8, 9, 12, 13]


def read_register(ser, motor_id, address, length):
    # Header(2) + ID(1) + Len(2) + Inst(2=Read) + Addr(1) + ReadLen(1) + Sum(1)
    checksum = (~(motor_id + 4 + 2 + address + length)) & 0xFF
    packet = bytes([0xFF, 0xFF, motor_id, 0x04, 0x02, address, length, checksum])

    ser.reset_input_buffer()
    ser.write(packet)
    time.sleep(0.01)  # 等待回包

    # 尝试读取头部 + 数据
    # 回包: Header(2) + ID(1) + Len(1) + Err(1) + Param(n) + Sum(1)
    response = ser.read(5 + length + 1)

    if len(response) < (5 + length + 1):
        return None

    # 解析参数部分
    params = response[5:-1]

    # 简单的转换逻辑
    if length == 1:
        return params[0]
    elif length == 2:
        return params[0] | (params[1] << 8)
    else:
        return params


def check_motor_health(ser, mid):
    print(f"\n====== 诊断 ID {mid} ======")

    # 1. 读取当前电压 (Address 46, 1 byte, unit=0.1V)
    voltage_raw = read_register(ser, mid, 46, 1)
    if voltage_raw is None:
        print("  ❌ 无法读取 (通信超时)")
        return

    voltage = voltage_raw / 10.0
    print(f"  🔋 当前电压: {voltage} V")

    # 2. 读取硬件错误状态 (Address 50, 1 byte)
    error_status = read_register(ser, mid, 50, 1)
    print(f"  ⚠️ 错误状态: {bin(error_status) if error_status is not None else 'N/A'}")
    if error_status != 0:
        print("     -> 警告：电机处于报错保护状态！")

    # 3. 读取工作模式 (Address 11, 1 byte)
    # 0=Position(关节), 1=Velocity(轮子), 3=Step
    mode = read_register(ser, mid, 11, 1)
    mode_str = "未知"
    if mode == 0:
        mode_str = "位置模式 (机械臂)"
    elif mode == 1:
        mode_str = "速度模式 (轮子)"
    elif mode == 3:
        mode_str = "步进模式"
    print(f"  ⚙️ 工作模式: {mode} ({mode_str})")

    # 4. 读取扭矩开关 (Address 40, 1 byte)
    torque = read_register(ser, mid, 40, 1)
    print(f"  🔒 扭矩状态: {'开启 (硬)' if torque == 1 else '关闭 (软)'}")


if __name__ == "__main__":
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        print(f"正在诊断端口 {PORT} 上的电机状态...")

        for i in TARGET_IDS:
            check_motor_health(ser, i)

        ser.close()
        print("\n诊断结束。")
    except Exception as e:
        print(f"串口错误: {e}")