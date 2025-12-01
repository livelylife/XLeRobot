import serial
import time

# === 修改这里为你设备管理器里看到的端口 ===
PORT = "COM3"
BAUD = 1000000


def scan_leader():
    print(f"正在尝试打开 {PORT} ...")
    try:
        # 打开串口
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        print("✅ 串口打开成功！正在扫描 ID 1-6...")

        found = []
        for mid in range(1, 7):
            # Ping 指令: Header(2)+ID(1)+Len(2)+Inst(1)+Sum(1)
            # Checksum = ~(ID + 2 + 1)
            checksum = (~(mid + 2 + 1)) & 0xFF
            packet = bytes([0xFF, 0xFF, mid, 0x02, 0x01, checksum])

            ser.reset_input_buffer()
            ser.write(packet)
            time.sleep(0.01)
            response = ser.read(10)

            if len(response) > 0:
                print(f"  -> 发现电机 ID {mid}")
                found.append(mid)

        if len(found) > 0:
            print(f"🎉 测试通过！发现主臂电机: {found}")
        else:
            print("❌ 串口能打开，但没扫到电机。")
            print("可能原因：1. 主臂没电？ 2. 也是 3 轮底盘那种波特率问题？")

        ser.close()

    except Exception as e:
        print(f"❌ 无法打开串口 {PORT}")
        print(f"错误信息: {e}")
        print("请检查：设备管理器里 COM 口号是否正确？是否被其他程序占用了？")


if __name__ == "__main__":
    scan_leader()