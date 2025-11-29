import serial
import time

PORT = "/dev/ttyACM0"
BAUD = 1000000


def scan_all_ids():
    print(f"正在 {PORT} 上地毯式搜索所有 ID (0-254)...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.05)
        found = []

        for motor_id in range(0, 254):
            # 构造 Ping 指令
            checksum = (~(motor_id + 2 + 1)) & 0xFF
            packet = bytes([0xFF, 0xFF, motor_id, 0x02, 0x01, checksum])

            ser.reset_input_buffer()
            ser.write(packet)
            time.sleep(0.002)  # 极短等待
            resp = ser.read(10)

            if len(resp) > 0:
                print(f"✅ 发现 ID: {motor_id}")
                found.append(motor_id)

        ser.close()
        print(f"\n搜索结束。找到的所有 ID: {found}")

        if 9 in found and 10 not in found:
            print("结论: ID 10 确实丢了，请检查连接线。")
        elif 10 in found:
            print("结论: ID 10 找到了！可能只是刚才接触不良。")
        else:
            print("结论: ID 10 变成了别的数字？请核对上面的列表。")

    except Exception as e:
        print(f"打开串口失败: {e}")


if __name__ == "__main__":
    scan_all_ids()