import serial
import time

# 定义要检测的端口
PORTS = ["/dev/ttyACM0", "/dev/ttyACM1"]
BAUD = 1000000  # 我们已经确认是 1M 波特率

# 指令: Ping ID 9 (这是底盘左轮的ID，只有右手板子有)
# Checksum = ~(9 + 2 + 1) = F3
CMD_PING_ID9 = bytes([0xFF, 0xFF, 0x09, 0x02, 0x01, 0xF3])

# 指令: Ping ID 1 (这是手臂肩部ID，两个板子都有)
CMD_PING_ID1 = bytes([0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB])


def check_port(port):
    print(f"正在检测 {port} ... ", end="")
    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)

        # 1. 检查有没有 ID 9 (底盘)
        ser.reset_input_buffer()
        ser.write(CMD_PING_ID9)
        time.sleep(0.02)
        resp9 = ser.read(10)

        # 2. 检查有没有 ID 1 (手臂) -以此确认端口是否连接了电机板
        ser.reset_input_buffer()
        ser.write(CMD_PING_ID1)
        time.sleep(0.02)
        resp1 = ser.read(10)

        ser.close()

        if len(resp9) > 0:
            print(f"✅ 发现底盘轮子 (ID 9)")
            return "RIGHT_AND_WHEELS"  # 这是一个右手+底盘板
        elif len(resp1) > 0:
            print(f"✅ 发现手臂电机，但无轮子")
            return "LEFT_AND_HEAD"  # 这是一个左手+头部板
        else:
            print("❌ 打开了但无电机响应 (可能是摄像头或其他设备)")
            return None

    except serial.SerialException:
        print("❌ 无法打开 (可能未连接或权限不足)")
        return None


print("========== 最终端口识别 ==========")
right_port = None
left_port = None

for p in PORTS:
    role = check_port(p)
    if role == "RIGHT_AND_WHEELS":
        right_port = p
    elif role == "LEFT_AND_HEAD":
        left_port = p

print("\n========== 📝 你的最终配置单 ==========")
if right_port and left_port:
    print(f"port1 (左手+头)   = '{left_port}'")
    print(f"port2 (右手+底盘) = '{right_port}'")
    print("\n请直接把这两行填入你的 Python 代码配置中！")
else:
    print("未能完全识别，请检查是否两个板子都插好了且有电。")