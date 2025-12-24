import sys
import logging
import time
import zmq
import json
import base64
import cv2

# 确保能找到 lerobot 库
sys.path.insert(0, "/home/joyandai/workspace/lerobot/src")

from lerobot.robots.xlerobot.xlerobot import XLerobot
from lerobot.robots.xlerobot.config_xlerobot import XLerobotConfig, XLerobotHostConfig


class CustomRobotHost:
    def __init__(self, config: XLerobotHostConfig, robot_instance):
        self.robot = robot_instance
        self.config = config
        self.zmq_context = zmq.Context()

        print(f"📡 绑定端口 - CMD: {config.port_zmq_cmd}, DATA: {config.port_zmq_observations}")
        self.zmq_cmd_socket = self.zmq_context.socket(zmq.PULL)
        self.zmq_cmd_socket.bind(f"tcp://*:{config.port_zmq_cmd}")
        self.zmq_observation_socket = self.zmq_context.socket(zmq.PUSH)
        self.zmq_observation_socket.bind(f"tcp://*:{config.port_zmq_observations}")
        self.running = True

    def run(self):
        print(f"🚀 服务器已启动! 正在等待 PC 指令...")
        last_cmd_time = time.time()

        while self.running:
            loop_start = time.time()

            try:
                msg = self.zmq_cmd_socket.recv_string(zmq.NOBLOCK)
                data = json.loads(msg)

                # === 调试日志 (新增) ===
                # 打印收到的指令，特别是第一个关节的角度
                shoulder_pan_pos = data.get("right_arm_shoulder_pan.pos")
                if shoulder_pan_pos is not None:
                    print(f"\r[接收到指令] Shoulder Pan: {shoulder_pan_pos:.2f}", end="")
                # ==========================

                self.robot.send_action(data)
                last_cmd_time = time.time()

            except zmq.Again:
                pass
            except Exception as e:
                print(f"指令处理错误: {e}")

            # ... (省略了 observation 和 watchdog 代码) ...

            elapsed = time.time() - loop_start
            time.sleep(max(0, 1 / self.config.max_loop_freq_hz - elapsed))

    def stop(self):
        print("\n正在关闭服务器...")
        self.running = False
        self.zmq_cmd_socket.close()
        self.zmq_observation_socket.close()
        self.zmq_context.term()


def main():
    logging.basicConfig(level=logging.INFO)
    print("=== 初始化机器人服务器 ===")

    # === 1. 硬件配置 ===
    robot_config = XLerobotConfig(
        id="my_xlerobot_lab",
        port1='/dev/ttyACM2',
        port2='/dev/ttyACM0',
        disable_torque_on_disconnect=True
    )

    # === 2. 网络配置 ===
    host_config = XLerobotHostConfig(
        port_zmq_cmd=5555,
        port_zmq_observations=5556,
        watchdog_timeout_ms=1000,
        max_loop_freq_hz=30
    )

    # === 3. 连接硬件 ===
    try:
        print("正在连接硬件...")
        robot = XLerobot(robot_config)
        robot.connect(calibrate=False)
        print("✅ 硬件连接成功！")
    except Exception as e:
        print(f"❌ 硬件连接失败: {e}")
        return

    # === 4. 启动服务器 ===
    server = CustomRobotHost(host_config, robot)
    try:
        server.run()
    except KeyboardInterrupt:
        print("\n用户停止。")
    finally:
        server.stop()
        robot.disconnect()
        print("已断开连接。")


if __name__ == "__main__":
    main()