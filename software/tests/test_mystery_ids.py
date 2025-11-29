import serial
import time
import struct

PORT = "/dev/ttyACM0"
BAUD = 1000000


# Feetech 协议写指令
def move_motor(ser, motor_id, position):
    # 位置范围 0-4096. 2048 是中间.
    # 这里我们只微调一点点，防止打坏东西
    # 转换为字节
    pos_L = position & 0xFF
    pos_H = (position >> 8) & 0xFF

    # 写入 Goal_Position (地址 42)
    # Header(2) + ID(1) + Len(5) + Inst(3=Write) + Addr(1) + Val(2) + Checksum
    length = 5
    instruction = 3
    address = 42

    checksum = (~(motor_id + length + instruction + address + pos_L + pos_H)) & 0xFF
    packet = bytes([0xFF, 0xFF, motor_id, length, instruction, address, pos_L, pos_H, checksum])

    ser.write(packet)
    print(f"发送指令：让 ID {motor_id} 移动到 {position}")


def check_id(target_id):
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print(f"\n>>> 正在测试 ID {target_id} (小心！观察哪个电机在动)")

        # 1. 扭矩开启 (防止电机无力)
        # 略过扭矩开启步骤，直接发位置，通常默认是有力的

        # 2. 读当前位置
        # (简化代码，直接让他动一下)

        # 让他往两个方向微微抖动
        current_pos = 2048  # 假设中间

        print("动作 1...")
        move_motor(ser, target_id, 2100)
        time.sleep(0.5)

        print("动作 2...")
        move_motor(ser, target_id, 2000)
        time.sleep(0.5)

        print("回中...")
        move_motor(ser, target_id, 2048)

        ser.close()
    except Exception as e:
        print(f"出错: {e}")


if __name__ == "__main__":
    print("请盯着机器人看...")
    print("我们要测试 ID 7 和 ID 8 到底是谁！")

    cmd = input("按回车测试 ID 7，按 's' 跳过: ")
    if cmd != 's':
        check_id(7)

    cmd = input("按回车测试 ID 8，按 's' 跳过: ")
    if cmd != 's':
        check_id(8)