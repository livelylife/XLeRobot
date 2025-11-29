import serial
import time

PORT = "/dev/ttyACM0"  # 你的右手+底盘端口
BAUD = 1000000
IDS = [7, 8, 9]


def move_motor_small_step(ser, motor_id):
    print(f"\n>>> 正在测试 ID {motor_id} ...")

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

    # 1. 往前动一点 (假设当前在 2048 附近)
    print(f"   ID {motor_id} -> 顺时针抖动")
    send_pos(2200)
    time.sleep(0.5)

    # 2. 往后动一点
    print(f"   ID {motor_id} -> 逆时针抖动")
    send_pos(1900)
    time.sleep(0.5)

    # 3. 回中
    print(f"   ID {motor_id} -> 回中")
    send_pos(2048)
    time.sleep(0.5)


if __name__ == "__main__":
    try:
        print("打开串口...")
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        print("请架空机器人，观察哪个轮子在抖动！")

        for i in IDS:
            input(f"\n按回车键开始测试 ID {i} ...")
            move_motor_small_step(ser, i)

        ser.close()
        print("\n测试结束。")
    except Exception as e:
        print(f"出错: {e}")