import serial
import time
import sys

# 扫描两个端口
PORTS = ["/dev/ttyACM0", "/dev/ttyACM1"]
BAUD = 1000000
# 扫描范围：只扫 1-20，通常够了
SCAN_RANGE = range(1, 21)


def read_info(ser, mid):
    # 构造读取指令: Read 5 bytes starting from Address 0 (Model L, Model H, Version, ID, Baud)
    # Header(2)+ID(1)+Len(4)+Inst(2)+Addr(1)+Len(1)+Sum(1)
    checksum = (~(mid + 4 + 2 + 0 + 6)) & 0xFF
    packet = bytes([0xFF, 0xFF, mid, 0x04, 0x02, 0x00, 0x06, checksum])

    ser.reset_input_buffer()
    ser.write(packet)
    time.sleep(0.005)

    # Header(2)+ID(1)+Len(1)+Err(1)+Data(6)+Sum(1) = 12 bytes
    response = ser.read(12)

    if len(response) < 12:
        return None

    # 尝试读取电压 (Address 46, 1 byte)
    checksum_volt = (~(mid + 4 + 2 + 46 + 1)) & 0xFF
    packet_volt = bytes([0xFF, 0xFF, mid, 0x04, 0x02, 0x2E, 0x01, checksum_volt])
    ser.write(packet_volt)
    time.sleep(0.005)
    resp_volt = ser.read(7)

    voltage = 0.0
    if len(resp_volt) == 7:
        voltage = resp_volt[5] / 10.0

    # 解析型号
    model = response[5] | (response[6] << 8)

    return {"model": model, "voltage": voltage}


def scan_all():
    print("============================================")
    print("      🤖 机器人全系统硬件体检报告")
    print("============================================")

    for port in PORTS:
        print(f"\n>>>> 正在扫描端口: {port}")
        try:
            ser = serial.Serial(port, BAUD, timeout=0.05)
        except Exception as e:
            print(f"❌ 无法打开端口: {e}")
            continue

        found_count = 0
        print(f"{'ID':<5} | {'型号':<10} | {'电压 (V)':<10} | {'状态判定'}")
        print("-" * 45)

        for mid in SCAN_RANGE:
            info = read_info(ser, mid)
            if info:
                found_count += 1
                v = info['voltage']
                status = "✅ 正常"

                # 智能判定状态
                if v < 5.0:
                    status = "🔴 严重欠压 (没接动力电!)"
                elif v < 9.0:
                    status = "⚠️ 电压过低 (电池没电)"
                elif v > 13.0:
                    status = "⚠️ 电压过高"

                print(f"{mid:<5} | {info['model']:<10} | {v:<10.1f} | {status}")

        if found_count == 0:
            print("(在此端口未发现任何电机，请检查接线)")

        ser.close()

    print("\n============================================")
    print("诊断建议：")
    print("1. 如果电压显示 < 5V，说明【动力线】没插好或断了。")
    print("2. 仅仅 USB 供电不足以驱动电机，只能读取 ID。")
    print("============================================")


if __name__ == "__main__":
    scan_all()