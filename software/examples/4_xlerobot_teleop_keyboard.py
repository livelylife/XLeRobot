import time
import sys
import threading
import termios
import tty
import select

from lerobot.robots.xlerobot.xlerobot import XLerobot
from lerobot.robots.xlerobot.config_xlerobot import XLerobotConfig


# === SSH 键盘监听类 (必须用这个，否则 SSH 无法控制) ===
class SSHKeyboard:
    def __init__(self):
        self.keys = {}
        self.running = False
        self.thread = None
        self.settings = termios.tcgetattr(sys.stdin)

    def connect(self):
        self.running = True
        self.thread = threading.Thread(target=self._listen, daemon=True)
        self.thread.start()
        print("\n[SSHKeyboard] 监听启动。")
        print("🎮 控制键位:")
        print("   i: 前进   k: 后退")
        print("   j: 左移   l: 右移")
        print("   u: 左转   o: 右转")
        print("   q: 退出")

    def disconnect(self):
        self.running = False
        if self.thread: self.thread.join(timeout=1.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def get_action(self):
        active_keys = self.keys.copy()
        self.keys.clear()
        return active_keys

    def _listen(self):
        try:
            tty.setcbreak(sys.stdin.fileno())
            while self.running:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key: self.keys[key] = True
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main():
    # === 1. 强制端口配置 (根据之前的物理测试) ===
    # Port 1: Left + Head (ACM2)
    # Port 2: Right + Base (ACM0)
    robot_config = XLerobotConfig(
        id="my_xlerobot_lab",
        port1='/dev/ttyACM2',
        port2='/dev/ttyACM0',
    )

    print("正在连接机器人 (三轮全向模式)...")
    try:
        # calibrate=False 表示跳过物理校准过程，直接加载文件
        robot = XLerobot(robot_config)
        robot.connect(calibrate=False)
        print("✅ 连接成功！")
    except Exception as e:
        print(f"❌ 连接失败: {e}")
        return

    keyboard = SSHKeyboard()
    keyboard.connect()

    try:
        while True:
            keys = keyboard.get_action()

            if 'q' in keys:
                print("退出程序...")
                break

            # === 2. 运动控制逻辑 ===
            vx, vy, omega = 0.0, 0.0, 0.0

            # 速度参数
            speed = 0.3
            rot_speed = 60

            # X轴 (前后)
            if 'i' in keys: vx += speed
            if 'k' in keys: vx -= speed

            # Y轴 (左右平移)
            if 'j' in keys: vy += speed
            if 'l' in keys: vy -= speed

            # 旋转
            if 'u' in keys: omega += rot_speed
            if 'o' in keys: omega -= rot_speed

            # 构建 Action
            action = {
                "x.vel": vx,
                "y.vel": vy,
                "theta.vel": omega
            }

            # 发送指令
            robot.send_action(action)

            # 频率控制 (20Hz)
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("停止中...")
    finally:
        robot.disconnect()
        keyboard.disconnect()


if __name__ == "__main__":
    main()