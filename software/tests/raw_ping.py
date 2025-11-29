import serial
import time
import sys

# 目标端口 (你现在的端口)
PORT = "/dev/ttyACM0"

# Feetech 协议指令
CMD_PING_ID1 = bytes([0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB])
CMD_PING_ID9 = bytes([0xFF, 0xFF, 0x09, 0x02, 0x01, 0xF3])
CMD_PING_ID10 = bytes([0xFF, 0xFF, 0x0A, 0x02, 0x01, 0xF2])

BAUDRATES = [1000000, 500000, 57600, 115200]


# 修改了函数名，防止 PyCharm 把它当成 pytest 跑
def check_baudrate(baud):
    print(f"Testing Baudrate: {baud} ... ", end="")
    try:
        ser = serial.Serial(PORT, baud, timeout=0.1)

        # 1. 测试 ID 1 (手臂)
        ser.reset_input_buffer()
        ser.write(CMD_PING_ID1)
        time.sleep(0.02)
        resp1 = ser.read(10)

        # 2. 测试 ID 9 (轮子)
        ser.reset_input_buffer()
        ser.write(CMD_PING_ID9)
        time.sleep(0.02)
        resp9 = ser.read(10)

        # 3. 测试 ID 10 (轮子)
        ser.reset_input_buffer()
        ser.write(CMD_PING_ID10)
        time.sleep(0.02)
        resp10 = ser.read(10)

        ser.close()

        found = []
        if len(resp1) > 0: found.append("ID_1(Arm)")
        if len(resp9) > 0: found.append("ID_9(Wheel)")
        if len(resp10) > 0: found.append("ID_10(Wheel)")

        if found:
            print(f"✅ 成功! 收到回应: {found}")
            return True
        else:
            print("❌ 无回应")
            return False

    except Exception as e:
        print(f"❌ 串口打开失败: {e}")
        return False


if __name__ == "__main__":
    print(f"========= 开始底层测试: {PORT} =========")
    success = False
    for b in BAUDRATES:
        if check_baudrate(b):
            success = True
            break

    if not success:
        print("\n[最终结论] 硬件通信完全失败。")
        print("可能原因：")
        print("1. 电池没电/开关没开 (最可能)")
        print("2. 权限不足 (sudo chmod 666)")
    else:
        print("\n[最终结论] 硬件通信正常！")