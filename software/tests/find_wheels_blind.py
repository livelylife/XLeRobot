import serial
import time

# 只测试轮子所在的端口
PORT = "/dev/ttyACM1"
BAUD = 1000000

# 盲测范围：1 到 20
TEST_RANGE = range(1, 21)


def force_spin_id(ser, mid):
    print(f"  ⚡️ 正在尝试激活 ID {mid} ...", end="")

    try:
        # 1. 强制解锁扭矩 (Address 40 = 1)
        # Header(2)+ID(1)+Len(4)+Inst(3)+Addr(1)+Data(1)+Sum(1)
        checksum_torque = (~(mid + 4 + 3 + 40 + 1)) & 0xFF
        ser.write(bytes([0xFF, 0xFF, mid, 0x04, 0x03, 0x28, 0x01, checksum_torque]))
        time.sleep(0.02)

        # 2. 切换到轮子模式 (Mode 1) - 以防它是轮子
        # 先关扭矩才能切模式，为了安全我们跳过这步，直接当它是普通电机转

        # 3. 发送速度指令 (让它转起来!)
        # 如果是位置模式，这会没用；如果是轮子模式，这会转起来。
        # 我们发送一个“位置指令”，让它转到 3000 (大约大半圈)
        # Goal Position (42)
        val_l = 3000 & 0xFF
        val_h = (3000 >> 8) & 0xFF
        checksum_pos = (~(mid + 5 + 3 + 42 + val_l + val_h)) & 0xFF
        ser.write(bytes([0xFF, 0xFF, mid, 0x05, 0x03, 0x2A, val_l, val_h, checksum_pos]))

        # 同时发送一个“速度指令” (以防它是轮子模式)
        # Goal Velocity (44), Speed 800
        # checksum_vel = (~(mid + 5 + 3 + 44 + 0x20 + 0x03)) & 0xFF
        # ser.write(bytes([0xFF, 0xFF, mid, 0x05, 0x03, 0x2C, 0x20, 0x03, checksum_vel]))

        print(" 指令已发送")
        time.sleep(1)  # 观察1秒

        # 4. 回中/停止
        # 位置回中
        checksum_home = (~(mid + 5 + 3 + 42 + 0x00 + 0x08)) & 0xFF
        ser.write(bytes([0xFF, 0xFF, mid, 0x05, 0x03, 0x2A, 0x00, 0x08, checksum_home]))

    except Exception as e:
        print(f" (发送失败)")


if __name__ == "__main__":
    print(f"正在盲测端口 {PORT} ... 请架空机器人！")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)

        for i in TEST_RANGE:
            print(f"\n[测试 ID {i}]")
            force_spin_id(ser, i)

            user = input(f"ID {i} 刚才动了吗？是哪个轮子？(动了输位置，没动直接回车): ")
            if user:
                print(f"✅✅✅ 找到轮子！ID {i} = {user}")

        ser.close()
        print("\n测试结束。")
    except Exception as e:
        print(f"串口错误: {e}")