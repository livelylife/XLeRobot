import time
import json
import zmq
import sys
import os

try:
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.motors.motors_bus import Motor, MotorNormMode, MotorCalibration
    from lerobot.robots.xlerobot.xlerobot import XLerobot
    from lerobot.robots.xlerobot.config_xlerobot import XLerobotConfig
except ImportError:
    print("❌ 严重错误: 找不到 lerobot 库。")
    sys.exit(1)

# === 配置区域 ===
JETSON_IP = "10.16.27.206"
JETSON_PORT = 5555
LEADER_PORT = "COM3"

# 主臂电机 ID (1-6)
leader_motor_ids = [1, 2, 3, 4, 5, 6]


def main():
    # 1. 建立 ZMQ 网络连接
    print(f"📡 正在连接远程机器人 {JETSON_IP}:{JETSON_PORT} ...")
    context = zmq.Context()
    socket = context.socket(zmq.PUSH)
    socket.setsockopt(zmq.SNDTIMEO, 1000)
    socket.connect(f"tcp://{JETSON_IP}:{JETSON_PORT}")
    print("✅ 网络通道已建立")

    # 2. 连接本地主臂
    print(f"🦾 正在连接本地主臂 ({LEADER_PORT})...")
    try:
        # === 关键修复：统一电机命名 ===
        print("🔧 正在加载校准文件并初始化主臂...")
        leader_config = XLerobotConfig(id="my_xlerobot_lab")
        temp_robot = XLerobot(leader_config)
        calibration_data = temp_robot.calibration

        # 定义主臂电机的名字，使其与校准文件中的 key 一致
        leader_motor_names = {
            1: "right_arm_shoulder_pan",
            2: "right_arm_shoulder_lift",
            3: "right_arm_elbow_flex",
            4: "right_arm_wrist_flex",
            5: "right_arm_wrist_roll",
            6: "right_arm_gripper",
        }

        leader_bus = FeetechMotorsBus(
            port=LEADER_PORT,
            motors={
                # 使用校准文件里的名字作为 key
                name: Motor(mid, "sts3215", MotorNormMode.RANGE_M100_100)
                for mid, name in leader_motor_names.items()
            },
            calibration=calibration_data  # 注入校准数据
        )

        leader_bus.connect()
        leader_bus.disable_torque()
        print("✅ 本地主臂已连接 (校准已加载)")
        # ================================

    except FileNotFoundError:
        print(f"❌ 校准文件未找到！请确认 my_xlerobot_lab.json 已被复制。")
        return
    except Exception as e:
        print(f"❌ 主臂连接失败: {e}")
        return

    print("\n🚀 开始主从控制！按 Ctrl+C 退出。")
    print("-----------------------------------")

    try:
        while True:
            start_time = time.time()
            # 读取主臂数据，现在名字匹配了，不会报错
            leader_pos = leader_bus.sync_read("Present_Position", list(leader_bus.motors.keys()))
            if not leader_pos: continue

            # 直接把主臂数据发给从臂 (因为名字已经一样了)
            action = {f"{name}.pos": pos for name, pos in leader_pos.items()}

            # 附加底盘锁定
            action["x.vel"] = 0.0;
            action["y.vel"] = 0.0;
            action["theta.vel"] = 0.0

            try:
                shoulder_pan_pos = action.get("right_arm_shoulder_pan.pos")
                if shoulder_pan_pos is not None:
                    print(f"\r[正在发送] Shoulder Pan: {shoulder_pan_pos:.2f}", end="")

                socket.send_string(json.dumps(action), flags=zmq.NOBLOCK)
            except Exception as e:
                print(f"⚠️ 发送错误: {e}")

            time.sleep(max(0, 1 / 30 - (time.time() - start_time)))

    except KeyboardInterrupt:
        print("\n🛑 停止中...")
    finally:
        try:
            leader_bus.disconnect()
        except:
            pass
        socket.close()
        context.term()
        print("已断开连接。")


if __name__ == "__main__":
    main()