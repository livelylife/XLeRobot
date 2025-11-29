import serial
import time

# 我们要扫描的两个端口
PORTS = ["/dev/ttyACM0", "/dev/ttyACM1"]
# 我们要扫描的潜在 ID (包含轮子可能的 ID)
TARGET_IDS = [7, 8, 9, 12, 13]
BAUD = 1000000


def wiggle_motor(ser, motor_id):
    # 构造写位置指令 (Goal_Position, Address 42)
    def send_pos(pos):
        pos_L = pos & 0xFF
        pos_H = (pos >> 8) & 0xFF
        # Header(2)+ID(1)+Len(5)+Inst(3)+Addr(1)+Data(2)+Sum(1)
        length = 5
        instruction = 3
        address = 42
        checksum = (~(motor_id + length + instruction + address + pos_L + pos_H)) & 0xFF
        packet = bytes([0xFF, 0xFF, motor_id, length, instruction, address, pos_L, pos_H, checksum])
        ser.write(packet)

    # 1. 开启扭矩 (Address 40 = 1)
    checksum_torque = (~(motor_id + 4 + 3 + 40 + 1)) & 0xFF
    ser.write(bytes([0xFF, 0xFF, motor_id, 0x04, 0x03, 0x28, 0x01, checksum_torque]))
    time.sleep(0.05)

    print(f"      -> 发送抖动指令...")
    # 2. 抖动
    send_pos(2150)
    time.sleep(0.4)
    send_pos(1950)
    time.sleep(0.4)
    send_pos(2048)  # 回中
    time.sleep(0.1)


def scan_port(port_name):
    print(f"\n====== 正在扫描端口: {port_name} ======")
    try:
        ser = serial.Serial(port_name, BAUD, timeout=0.1)
    except Exception as e:
        print(f"  ❌ 无法打开端口 (可能没插或被占用): {e}")
        return

    print("  警告：请架空机器人，电机即将转动！")

    for mid in TARGET_IDS:
        print(f"\n  [?] 正在测试 ID {mid} ...")

        # 先尝试读取一下，看看在不在，不在就别浪费时间抖动了
        checksum_read = (~(mid + 4 + 2 + 57 + 2)) & 0xFF  # Read present position
        ser.reset_input_buffer()
        ser.write(bytes([0xFF, 0xFF, mid, 0x04, 0x02, 0x39, 0x02, checksum_read]))
        time.sleep(0.02)
        response = ser.read(10)

        if len(response) > 0:
            print(f"    ✅ ID {mid} 在线！正在抖动它...")
            wiggle_motor(ser, mid)
            user_input = input(f"    >>> 请看一眼：ID {mid} 是哪个部位？(回车继续，输入记录): ")
            if user_input:
                print(f"    *** 记录: ID {mid} = {user_input}")
        else:
            print(f"    ❌ ID {mid} 无响应 (可能不在此端口，或没电)")

    ser.close()


if __name__ == "__main__":
    # 记得先授权
    import os

    os.system("sudo chmod 666 /dev/ttyACM0 /dev/ttyACM1")

    for port in PORTS:
        scan_port(port)
        print("\n------------------------------------------------")
