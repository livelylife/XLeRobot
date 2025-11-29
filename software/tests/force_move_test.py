import serial
import time

PORT = "/dev/ttyACM1"  # 你的右手+底盘端口
BAUD = 1000000

# 我们只测这些“嫌疑”ID，跳过手臂 1-6
TARGET_IDS = [7, 8, 9, 12, 13]


def write_packet(ser, motor_id, address, value_bytes):
    # Feetech 协议写指令
    # Header(2)+ID(1)+Len(2+len+1)+Inst(3)+Addr(1)+Data(len)+Sum(1)
    length = 2 + len(value_bytes) + 1
    instruction = 3

    payload = [address] + list(value_bytes)
    checksum = (~(motor_id + length + instruction + sum(payload))) & 0xFF
    packet = bytes([0xFF, 0xFF, motor_id, length, instruction]) + bytes(payload) + bytes([checksum])
    ser.write(packet)


def test_motor(ser, mid):
    print(f"\n====== 测试 ID {mid} ======")

    # 1. 强制开启扭矩 (Address 40, Value 1)
    print("  -> 开启扭矩 (Torque On)...")
    write_packet(ser, mid, 40, bytes([1]))
    time.sleep(0.1)

    # 2. 尝试往两个方向动 (Goal Position, Address 42)
    # 假设中心是 2048
    print("  -> 动作 A (2200)")
    # 2200 = 0x0898 -> Low: 0x98, High: 0x08
    write_packet(ser, mid, 42, bytes([0x98, 0x08]))
    time.sleep(0.5)

    print("  -> 动作 B (1900)")
    # 1900 = 0x076C -> Low: 0x6C, High: 0x07
    write_packet(ser, mid, 42, bytes([0x6C, 0x07]))
    time.sleep(0.5)

    print("  -> 回中 (2048)")
    write_packet(ser, mid, 42, bytes([0x00, 0x08]))

    # 3. 再次尝试：如果它是轮子模式，可能需要发速度指令
    # 如果上面没动，试试发速度 (Velocity Mode)
    # 先把扭矩关了切模式
    # write_packet(ser, mid, 40, bytes([0]))
    # write_packet(ser, mid, 11, bytes([1])) # Operating Mode = 1 (Velocity)
    # write_packet(ser, mid, 40, bytes([1]))
    # write_packet(ser, mid, 44, bytes([0xF4, 0x01])) # Speed 500
    # time.sleep(1)
    # write_packet(ser, mid, 44, bytes([0, 0])) # Stop


if __name__ == "__main__":
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        print("请架空机器人，注意观察！")

        for i in TARGET_IDS:
            cmd = input(f"按回车测试 ID {i} (按 s 跳过): ")
            if cmd != 's':
                test_motor(ser, i)

        ser.close()
    except Exception as e:
        print(f"出错: {e}")