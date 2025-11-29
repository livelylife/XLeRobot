# To Run on the host
'''python
PYTHONPATH=src python -m lerobot.robots.xlerobot.xlerobot_host --robot.id=my_xlerobot
'''

# To Run the teleop:
'''python
PYTHONPATH=src python -m examples.xlerobot.teleoperate_Keyboard
'''

import time
import numpy as np
import math
import argparse
import sys
import threading
import termios
import tty
import select

# 确保能找到 lerobot 库
# sys.path.insert(0, "/home/joyandai/workspace/lerobot/src") # 如果需要，取消注释这行

# Comment the following line when used locally
# from lerobot.robots.xlerobot import XLerobotClient, XLerobotConfigClient
from lerobot.robots.xlerobot import XLerobotConfig, XLerobot
from lerobot.utils.robot_utils import busy_wait
# 禁用 Rerun 初始化，防止 SSH 报错，只导入 log_rerun_data
from lerobot.utils.visualization_utils import log_rerun_data
from lerobot.model.SO101Robot import SO101Kinematics


# 替换掉原有的 KeyboardTeleop，使用我们要定义的 SSHKeyboard
# from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig

# === SSH 键盘监听类 (新增) ===
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
        print("\n[SSHKeyboard] 监听已启动。")

    def disconnect(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def get_action(self):
        # 返回当前按下的键
        active_keys = self.keys.copy()
        self.keys.clear()
        return active_keys

    def _listen(self):
        try:
            tty.setcbreak(sys.stdin.fileno())
            while self.running:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key:
                        self.keys[key] = True
                        if key == '\x03':  # Ctrl+C
                            self.running = False
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


# Keymaps (semantic action: key)
LEFT_KEYMAP = {
    'shoulder_pan+': 'e', 'shoulder_pan-': 'q',
    'wrist_roll+': 'r', 'wrist_roll-': 'f',
    'gripper+': 't', 'gripper-': 'g',
    'x+': 'w', 'x-': 's', 'y+': 'a', 'y-': 'd',
    'pitch+': 'z', 'pitch-': 'x',
    'reset': 'c',
    # For head motors
    "head_motor_1+": "<", "head_motor_1-": ">",
    "head_motor_2+": ",", "head_motor_2-": ".",

    'triangle': 'y',  # Rectangle trajectory key
}
RIGHT_KEYMAP = {
    'shoulder_pan+': '9', 'shoulder_pan-': '7',
    'wrist_roll+': '/', 'wrist_roll-': '*',
    'gripper+': '+', 'gripper-': '-',
    'x+': '8', 'x-': '2', 'y+': '4', 'y-': '6',
    'pitch+': '1', 'pitch-': '3',
    'reset': '0',

    'triangle': 'Y',  # Rectangle trajectory key
}

LEFT_JOINT_MAP = {
    "shoulder_pan": "left_arm_shoulder_pan",
    "shoulder_lift": "left_arm_shoulder_lift",
    "elbow_flex": "left_arm_elbow_flex",
    "wrist_flex": "left_arm_wrist_flex",
    "wrist_roll": "left_arm_wrist_roll",
    "gripper": "left_arm_gripper",
}
RIGHT_JOINT_MAP = {
    "shoulder_pan": "right_arm_shoulder_pan",
    "shoulder_lift": "right_arm_shoulder_lift",
    "elbow_flex": "right_arm_elbow_flex",
    "wrist_flex": "right_arm_wrist_flex",
    "wrist_roll": "right_arm_wrist_roll",
    "gripper": "right_arm_gripper",
}

# Head motor mapping
HEAD_MOTOR_MAP = {
    "head_motor_1": "head_motor_1",
    "head_motor_2": "head_motor_2",
}


class RectangularTrajectory:
    def __init__(self, width=0.06, height=0.06, segment_duration=0.91):
        self.width = width
        self.height = height
        self.segment_duration = segment_duration
        self.total_duration = 4 * segment_duration

    def get_trajectory_point(self, current_x, current_y, t):
        segment = int(t / self.segment_duration)
        segment_t = t % self.segment_duration
        normalized_t = segment_t / self.segment_duration
        smooth_t = 0.5 * (1 - math.cos(math.pi * normalized_t))

        corners = [
            (current_x, current_y),  # Start (bottom-left)
            (current_x + self.width, current_y),  # Bottom-right
            (current_x + self.width, current_y + self.height),  # Top-right
            (current_x, current_y + self.height),  # Top-left
            (current_x, current_y)  # Back to start
        ]

        segment = max(0, min(3, segment))
        start_corner = corners[segment]
        end_corner = corners[segment + 1]

        target_x = start_corner[0] + smooth_t * (end_corner[0] - start_corner[0])
        target_y = start_corner[1] + smooth_t * (end_corner[1] - start_corner[1])

        return target_x, target_y


class SimpleHeadControl:
    def __init__(self, initial_obs, kp=0.81):
        self.kp = kp
        self.degree_step = 1
        self.target_positions = {
            "head_motor_1": initial_obs.get("head_motor_1.pos", 0.0),
            "head_motor_2": initial_obs.get("head_motor_2.pos", 0.0),
        }
        self.zero_pos = {"head_motor_1": 0.0, "head_motor_2": 0.0}

    def move_to_zero_position(self, robot):
        self.target_positions = self.zero_pos.copy()
        action = self.p_control_action(robot)
        robot.send_action(action)

    def handle_keys(self, key_state):
        if key_state.get('head_motor_1+'): self.target_positions["head_motor_1"] += self.degree_step
        if key_state.get('head_motor_1-'): self.target_positions["head_motor_1"] -= self.degree_step
        if key_state.get('head_motor_2+'): self.target_positions["head_motor_2"] += self.degree_step
        if key_state.get('head_motor_2-'): self.target_positions["head_motor_2"] -= self.degree_step

    def p_control_action(self, robot):
        obs = robot.get_observation()
        action = {}
        for motor in self.target_positions:
            current = obs.get(f"{HEAD_MOTOR_MAP[motor]}.pos", 0.0)
            error = self.target_positions[motor] - current
            control = self.kp * error
            action[f"{HEAD_MOTOR_MAP[motor]}.pos"] = current + control
        return action


class SimpleTeleopArm:
    def __init__(self, kinematics, joint_map, initial_obs, prefix="left", kp=0.81):
        self.kinematics = kinematics
        self.joint_map = joint_map
        self.prefix = prefix
        self.kp = kp
        self.joint_positions = {
            "shoulder_pan": initial_obs[f"{prefix}_arm_shoulder_pan.pos"],
            "shoulder_lift": initial_obs[f"{prefix}_arm_shoulder_lift.pos"],
            "elbow_flex": initial_obs[f"{prefix}_arm_elbow_flex.pos"],
            "wrist_flex": initial_obs[f"{prefix}_arm_wrist_flex.pos"],
            "wrist_roll": initial_obs[f"{prefix}_arm_wrist_roll.pos"],
            "gripper": initial_obs[f"{prefix}_arm_gripper.pos"],
        }
        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0
        self.degree_step = 1
        self.xy_step = 0.0021
        self.target_positions = {k: 0.0 for k in self.joint_positions}
        self.zero_pos = {k: 0.0 for k in self.joint_positions}

        self.rectangular_trajectory = RectangularTrajectory(
            width=0.06, height=0.06, segment_duration=1.01
        )

    def move_to_zero_position(self, robot):
        self.target_positions = self.zero_pos.copy()
        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0
        self.target_positions["wrist_flex"] = 0.0
        action = self.p_control_action(robot)
        robot.send_action(action)

    def execute_rectangular_trajectory(self, robot, fps=30):
        print(f"[{self.prefix}] Starting rectangular trajectory...")
        start_x = self.current_x
        start_y = self.current_y
        start_time = time.time()

        while True:
            elapsed_time = time.time() - start_time
            if elapsed_time >= self.rectangular_trajectory.total_duration:
                break

            target_x, target_y = self.rectangular_trajectory.get_trajectory_point(
                start_x, start_y, elapsed_time
            )
            self.current_x = target_x
            self.current_y = target_y

            try:
                joint2, joint3 = self.kinematics.inverse_kinematics(self.current_x, self.current_y)
                self.target_positions["shoulder_lift"] = joint2
                self.target_positions["elbow_flex"] = joint3
                self.target_positions["wrist_flex"] = (
                        -self.target_positions["shoulder_lift"]
                        - self.target_positions["elbow_flex"]
                        + self.pitch
                )

                action = self.p_control_action(robot)
                if self.prefix == "left":
                    robot_action = {**action, **{}, **{}, **{}}
                elif self.prefix == "right":
                    robot_action = {**{}, **action, **{}, **{}}

                robot.send_action(robot_action)
                time.sleep(1.0 / fps)

            except Exception as e:
                print(f"[{self.prefix}] IK failed: {e}")
                break

    def handle_keys(self, key_state):
        if key_state.get('shoulder_pan+'): self.target_positions["shoulder_pan"] += self.degree_step
        if key_state.get('shoulder_pan-'): self.target_positions["shoulder_pan"] -= self.degree_step
        if key_state.get('wrist_roll+'): self.target_positions["wrist_roll"] += self.degree_step
        if key_state.get('wrist_roll-'): self.target_positions["wrist_roll"] -= self.degree_step
        if key_state.get('gripper+'): self.target_positions["gripper"] += self.degree_step
        if key_state.get('gripper-'): self.target_positions["gripper"] -= self.degree_step
        if key_state.get('pitch+'): self.pitch += self.degree_step
        if key_state.get('pitch-'): self.pitch -= self.degree_step

        moved = False
        if key_state.get('x+'): self.current_x += self.xy_step; moved = True
        if key_state.get('x-'): self.current_x -= self.xy_step; moved = True
        if key_state.get('y+'): self.current_y += self.xy_step; moved = True
        if key_state.get('y-'): self.current_y -= self.xy_step; moved = True

        if moved:
            try:
                joint2, joint3 = self.kinematics.inverse_kinematics(self.current_x, self.current_y)
                self.target_positions["shoulder_lift"] = joint2
                self.target_positions["elbow_flex"] = joint3
            except:
                pass

        self.target_positions["wrist_flex"] = (
                -self.target_positions["shoulder_lift"]
                - self.target_positions["elbow_flex"]
                + self.pitch
        )

    def p_control_action(self, robot):
        obs = robot.get_observation()
        current = {j: obs[f"{self.prefix}_arm_{j}.pos"] for j in self.joint_map}
        action = {}
        for j in self.target_positions:
            error = self.target_positions[j] - current[j]
            control = self.kp * error
            action[f"{self.joint_map[j]}.pos"] = current[j] + control
        return action


def main(robot_id=None):
    # Teleop parameters
    FPS = 30
    robot_name = "my_xlerobot_lab"

    # === 1. 强制端口配置 ===
    # Port 1: Left + Head (ACM2 - 拔插测试结果)
    # Port 2: Right + Base (ACM0 - 拔插测试结果)
    robot_config = XLerobotConfig(
        id=robot_name,
        port1='/dev/ttyACM2',
        port2='/dev/ttyACM0',
    )

    print("正在连接机器人 (三轮全向模式)...")
    try:
        # calibrate=False 跳过物理校准
        robot = XLerobot(robot_config)
        robot.connect(calibrate=False)
        print("✅ 连接成功！")
    except Exception as e:
        print(f"❌ 连接失败: {e}")
        return

    # init_rerun(session_name="xlerobot_teleop_v2") # 禁用 Rerun

    # 使用 SSH 键盘替代原有的 KeyboardTeleop
    keyboard = SSHKeyboard()
    keyboard.connect()

    # Init the arm and head instances
    obs = robot.get_observation()
    kin_left = SO101Kinematics()
    kin_right = SO101Kinematics()
    left_arm = SimpleTeleopArm(kin_left, LEFT_JOINT_MAP, obs, prefix="left")
    right_arm = SimpleTeleopArm(kin_right, RIGHT_JOINT_MAP, obs, prefix="right")
    head_control = SimpleHeadControl(obs)

    # Move both arms and head to zero position at start
    left_arm.move_to_zero_position(robot)
    right_arm.move_to_zero_position(robot)

    # Print comprehensive keymap information based on robot config
    print("\n" + "=" * 80)
    print("🤖 XLeRobot 2Wheels Keyboard Control Keymap")
    print("=" * 80)

    print("\n📱 Base Control (Differential Drive):")
    print(f"    {robot.teleop_keys['forward']}: Forward")
    print(f"    {robot.teleop_keys['backward']}: Backward")
    print(f"    {robot.teleop_keys['rotate_left']}: Rotate Left")
    print(f"    {robot.teleop_keys['rotate_right']}: Rotate Right")
    print(f"    {robot.teleop_keys['speed_up']}: Speed Up")
    print(f"    {robot.teleop_keys['speed_down']}: Speed Down")
    print(f"    {robot.teleop_keys['quit']}: Quit")
    print("    🚀 Smooth Control: Linear acceleration when holding, linear deceleration when released")

    print("\n🦾 Left Arm Control:")
    print("   Joint Control:")
    print(f"    Q/E: Shoulder Pan +/- (shoulder_pan)")
    print(f"    R/F: Wrist Roll +/- (wrist_roll)")
    print(f"    T/G: Gripper +/- (gripper)")
    print(f"    Z/X: Pitch +/- (pitch)")
    print("   Position Control:")
    print(f"    W/S: X-axis +/- (x movement)")
    print(f"    A/D: Y-axis +/- (y movement)")
    print("   Special Functions:")
    print(f"    C: Reset to zero position")
    print(f"    Y: Execute rectangular trajectory")

    print("\n🦾 Right Arm Control:")
    print("   Joint Control:")
    print(f"    7/9: Shoulder Pan +/- (shoulder_pan)")
    print(f"    /*: Wrist Roll +/- (wrist_roll)")
    print(f"    +/-: Gripper +/- (gripper)")
    print(f"    1/3: Pitch +/- (pitch)")
    print("   Position Control:")
    print(f"    8/2: X-axis +/- (x movement)")
    print(f"    4/6: Y-axis +/- (y movement)")
    print("   Special Functions:")
    print(f"    0: Reset to zero position")
    print(f"    Y: Execute rectangular trajectory")

    print("\n👁️ Head Control:")
    print(f"    </>: Head Motor 1 +/- (head_motor_1)")
    print(f"    ,/.: Head Motor 2 +/- (head_motor_2)")
    print(f"    ?: Head reset to zero position")

    print("\n" + "=" * 80)
    print("🎮 Control started! Use above keys to control robot")
    print("=" * 80 + "\n")

    try:
        while True:
            # 获取 SSH 键盘按键
            key_dict = keyboard.get_action()
            pressed_keys = set(key_dict.keys())

            # 手臂控制
            left_key_state = {action: (key in pressed_keys) for action, key in LEFT_KEYMAP.items()}
            right_key_state = {action: (key in pressed_keys) for action, key in RIGHT_KEYMAP.items()}

            if robot.teleop_keys['quit'] in pressed_keys: break  # 退出

            # Trajectory Handling
            if left_key_state.get('triangle'):
                left_arm.execute_rectangular_trajectory(robot, fps=FPS)
                continue
            if right_key_state.get('triangle'):
                right_arm.execute_rectangular_trajectory(robot, fps=FPS)
                continue

            # Reset Handling
            if left_key_state.get('reset'):
                left_arm.move_to_zero_position(robot)
                continue
            if right_key_state.get('reset'):
                right_arm.move_to_zero_position(robot)
                continue
            if '?' in pressed_keys:
                head_control.move_to_zero_position(robot)
                continue

            # Update Targets
            left_arm.handle_keys(left_key_state)
            right_arm.handle_keys(right_key_state)
            head_control.handle_keys(left_key_state)

            # Calculate Actions
            left_action = left_arm.p_control_action(robot)
            right_action = right_arm.p_control_action(robot)
            head_action = head_control.p_control_action(robot)

            # === Base Control (Omni) ===
            # 将按键集合转换为 numpy 数组供 _from_keyboard_to_base_action 使用
            # 注意：我们的 SSH 键盘返回的是单个字符，可能需要适配
            # 为了简单，我们手动构建 base action
            vx, vy, omega = 0.0, 0.0, 0.0
            speed = 0.2
            rot = 45

            if 'i' in pressed_keys: vx += speed
            if 'k' in pressed_keys: vx -= speed
            if 'j' in pressed_keys: vy += speed
            if 'l' in pressed_keys: vy -= speed
            if 'u' in pressed_keys: omega += rot
            if 'o' in pressed_keys: omega -= rot

            base_action = {"x.vel": vx, "y.vel": vy, "theta.vel": omega}

            # Combine and Send
            action = {**left_action, **right_action, **head_action, **base_action}
            robot.send_action(action)

            # obs = robot.get_observation()
            # log_rerun_data(obs, action) # 禁用

            time.sleep(1.0 / FPS)

    finally:
        robot.disconnect()
        keyboard.disconnect()
        print("Teleoperation ended.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot_id", type=str)
    args = parser.parse_args()
    main(args.robot_id)