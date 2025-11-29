import sys

# 确保能找到 lerobot 库 (根据你之前的路径配置)
sys.path.insert(0, "/home/joyandai/workspace/lerobot/src/")

from lerobot.motors.feetech import FeetechMotorsBus

# 扫描所有出现的 ACM 端口
PORTS = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2"]


def scan():
    print("========== 开始修正版扫描 ==========")

    for port in PORTS:
        print(f"\n>> 正在检查端口: {port}")

        try:
            # 修正：不传 baudrate 参数，使用默认值
            bus = FeetechMotorsBus(port=port, motors={})
            bus.connect()

            found_ids = []
            # 扩大扫描范围：扫描 1 到 20 (涵盖手臂、头部、轮子)
            for i in range(1, 21):
                try:
                    val = bus.read("Present_Position", i)
                    if val is not None:
                        found_ids.append(i)
                except:
                    pass

            bus.disconnect()

            if found_ids:
                print(f"✅ 成功! 端口 {port} 发现电机 ID: {found_ids}")

                # 智能识别
                if 9 in found_ids or 10 in found_ids:
                    print(f"   -> 包含 ID 9/10，这应该是：[右手 + 底盘轮子] 的板子")
                else:
                    print(f"   -> 不含轮子，这应该是：[左手 + 头部] 的板子")
            else:
                print(f"❌ 端口 {port} 可打开，但没有电机回应 (可能是空闲端口或供电不足)。")

        except Exception as e:
            print(f"❌ 无法连接 {port}: {e}")

    print("\n========== 扫描结束 ==========")


if __name__ == "__main__":
    scan()