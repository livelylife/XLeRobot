import serial
import sys

PORTS = ["/dev/ttyACM0", "/dev/ttyACM1"]
BAUD = 1000000

# Ping ID 9 (只有右手+底盘板子有这个ID)
# Checksum = ~(9 + 2 + 1) = F3
CMD_PING_ID9 = bytes([0xFF, 0xFF, 0x09, 0x02, 0x01, 0xF3])

print("正在检测端口归属...")

for port in PORTS:
    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
        ser.reset_input_buffer()
        ser.write(CMD_PING_ID9)
        response = ser.read(10)
        ser.close()

        if len(response) > 0:
            print(f"✅ {port} 是 【右手 + 底盘】 (发现了 ID 9)")
        else:
            print(f"ℹ️ {port} 是 【左手 + 头部】 (没发现 ID 9)")

    except Exception as e:
        print(f"❌ 无法打开 {port}: {e}")