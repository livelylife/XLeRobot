import time
import sys
import threading
import termios
import tty
import select
import math
import argparse
import numpy as np

sys.path.insert(0, "/home/wisx/workspace/lerobot/src")

from lerobot.robots.xlerobot.xlerobot import XLerobot
from lerobot.robots.xlerobot.config_xlerobot import XLerobotConfig
from lerobot.model.SO101Robot import SO101Kinematics


# === SSH 键盘监听类 (支持长按) ===
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

    def disconnect(self):
        self.running = False
        if self.thread: self.thread.join(timeout=1.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def get_pressed_keys(self):
        return set(self.keys.keys())

    def _listen(self):
        try:
            tty.setcbreak(sys.stdin.fileno())
            while self.running:
                # Read keys with a short timeout to detect key releases
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    key = sys.stdin.read(1)
                    if key: self.keys[key] = True
                else:  # No key pressed in the last 50ms, clear all keys
                    self.keys.clear()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


# === Keymaps, Arm and Head Control Classes ===
LEFT_KEYMAP = {'shoulder_pan+': 'e', 'shoulder_pan-': 'q', 'wrist_roll+': 'r', 'wrist_roll-': 'f', 'gripper+': 't',
               'gripper-': 'g', 'x+': 'w', 'x-': 's', 'y+': 'a', 'y-': 'd', 'pitch+': 'z', 'pitch-': 'x', 'reset': 'c',
               "head_motor_1+": "<", "head_motor_1-": ">", "head_motor_2+": ",", "head_motor_2-": ".", 'triangle': 'y'}
RIGHT_KEYMAP = {'shoulder_pan+': '9', 'shoulder_pan-': '7', 'wrist_roll+': '/', 'wrist_roll-': '*', 'gripper+': '+',
                'gripper-': '-', 'x+': '8', 'x-': '2', 'y+': '4', 'y-': '6', 'pitch+': '1', 'pitch-': '3', 'reset': '0',
                'triangle': 'Y'}
LEFT_JOINT_MAP = {"shoulder_pan": "left_arm_shoulder_pan", "shoulder_lift": "left_arm_shoulder_lift",
                  "elbow_flex": "left_arm_elbow_flex", "wrist_flex": "left_arm_wrist_flex",
                  "wrist_roll": "left_arm_wrist_roll", "gripper": "left_arm_gripper"}
RIGHT_JOINT_MAP = {"shoulder_pan": "right_arm_shoulder_pan", "shoulder_lift": "right_arm_shoulder_lift",
                   "elbow_flex": "right_arm_elbow_flex", "wrist_flex": "right_arm_wrist_flex",
                   "wrist_roll": "right_arm_wrist_roll", "gripper": "right_arm_gripper"}
HEAD_MOTOR_MAP = {"head_motor_1": "head_motor_1", "head_motor_2": "head_motor_2"}


class SimpleHeadControl:
    def __init__(self, initial_obs, kp=0.81):
        self.kp = kp;
        self.degree_step = 1
        self.target_positions = {"head_motor_1": initial_obs.get("head_motor_1.pos", 0.0),
                                 "head_motor_2": initial_obs.get("head_motor_2.pos", 0.0)}
        self.zero_pos = {"head_motor_1": 0.0, "head_motor_2": 0.0}

    def move_to_zero_position(self, robot):
        self.target_positions = self.zero_pos.copy(); robot.send_action(self.p_control_action(robot))

    def handle_keys(self, key_state):
        if key_state.get('head_motor_1+'): self.target_positions["head_motor_1"] += self.degree_step
        if key_state.get('head_motor_1-'): self.target_positions["head_motor_1"] -= self.degree_step
        if key_state.get('head_motor_2+'): self.target_positions["head_motor_2"] += self.degree_step
        if key_state.get('head_motor_2-'): self.target_positions["head_motor_2"] -= self.degree_step

    def p_control_action(self, robot):
        obs = robot.get_observation();
        action = {}
        for motor in self.target_positions:
            current = obs.get(f"{HEAD_MOTOR_MAP[motor]}.pos", 0.0);
            error = self.target_positions[motor] - current
            action[f"{HEAD_MOTOR_MAP[motor]}.pos"] = current + self.kp * error
        return action


class SimpleTeleopArm:
    def __init__(self, kinematics, joint_map, initial_obs, prefix="left", kp=0.81):
        self.kinematics, self.joint_map, self.prefix, self.kp = kinematics, joint_map, prefix, kp
        self.joint_positions = {j.replace(f"{prefix}_arm_", ""): initial_obs[f"{j}.pos"] for j in joint_map.values()}
        self.current_x, self.current_y, self.pitch = 0.1629, 0.1131, 0.0
        self.degree_step, self.xy_step = 1, 0.0021
        self.target_positions = {k: 0.0 for k in self.joint_positions}
        self.zero_pos = self.target_positions.copy()

    def move_to_zero_position(self, robot):
        self.target_positions = self.zero_pos.copy();
        self.current_x, self.current_y, self.pitch = 0.1629, 0.1131, 0.0
        self.target_positions["wrist_flex"] = 0.0;
        robot.send_action(self.p_control_action(robot))

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
                j2, j3 = self.kinematics.inverse_kinematics(self.current_x, self.current_y)
                self.target_positions["shoulder_lift"], self.target_positions["elbow_flex"] = j2, j3
            except:
                pass
        self.target_positions["wrist_flex"] = -self.target_positions["shoulder_lift"] - self.target_positions[
            "elbow_flex"] + self.pitch

    def p_control_action(self, robot):
        obs, action = robot.get_observation(), {}
        for j_name, j_map in self.joint_map.items():
            error = self.target_positions[j_name] - obs[f"{j_map}.pos"]
            action[f"{j_map}.pos"] = obs[f"{j_map}.pos"] + self.kp * error
        return action


def print_robot_status(robot):
    print("\n" + "=" * 50 + "\n      🤖 机器人硬件状态报告 🤖\n" + "=" * 50)
    config = robot.config
    print(f"\n--- Port {config.port1} (Bus 1) ---")
    if config.enable_left_arm:
        print(
            f"  - 左臂:  配置启用 -> {'✅ 已连接' if robot.left_arm_motors and robot.bus1 and robot.bus1.is_connected else '❌ 未检测到'}")
    else:
        print("  - 左臂:  配置禁用")
    if config.enable_head:
        print(
            f"  - 头部:  配置启用 -> {'✅ 已连接' if robot.head_motors and robot.bus1 and robot.bus1.is_connected else '❌ 未检测到'}")
    else:
        print("  - 头部:  配置禁用")
    print(f"\n--- Port {config.port2} (Bus 2) ---")
    if config.enable_right_arm:
        print(
            f"  - 右臂: 配置启用 -> {'✅ 已连接' if robot.right_arm_motors and robot.bus2 and robot.bus2.is_connected else '❌ 未检测到'}")
    else:
        print("  - 右臂: 配置禁用")
    if config.enable_base:
        print(
            f"  - 底盘:  配置启用 -> {'✅ 已连接' if robot.base_motors and robot.bus2 and robot.bus2.is_connected else '❌ 未检测到'}")
    else:
        print("  - 底盘:  配置禁用")
    print("=" * 50 + "\n")


    if config.enable_head and robot.head_motors:
        print("\n👁️ Head Control:")
        print(f"    </>: Head Motor 1 +/- (head_motor_1)")
        print(f"    ,/.: Head Motor 2 +/- (head_motor_2)")
        print(f"    ?: Head reset to zero position")

    if config.enable_left_arm and robot.left_arm_motors:
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

    if config.enable_right_arm and robot.right_arm_motors:
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

    if config.enable_base and robot.base_motors:
        print("\n🛞 Base Control (Omni-directional):")
        print("   Movement:")
        print(f"    i: 前进 (Forward)")
        print(f"    k: 后退 (Backward)")
        print(f"    j: 左移 (Strafe Left)")
        print(f"    l: 右移 (Strafe Right)")
        print("   Rotation:")
        print(f"    u: 逆时针旋转 (Rotate CCW)")
        print(f"    o: 顺时针旋转 (Rotate CW)")
        print("   Speed Control:")
        print(f"    n: 加速 (Speed Up)")
        print(f"    m: 减速 (Speed Down)")
        print("   Special Functions:")
        print(f"    b: 停止并退出 (Quit)")


    print("\n" + "=" * 50)
    print("🎮 Control started! Use above keys to control robot")
    print("=" * 80 + "\n")


def main(robot_id=None):
    FPS = 20
    robot_config = XLerobotConfig(
        id=robot_id or "my_xlerobot_lab",
        port1='/dev/ttyACM1',
        port2='/dev/ttyACM0',
        enable_left_arm=False,
        enable_right_arm=False,
        enable_head=False,
        enable_base=True,
    )

    print("正在连接机器人 (可选择性连接)...")
    try:
        robot = XLerobot(robot_config)
        robot.connect(calibrate=False)
        print(f"✅ 连接成功！port1={robot.config.port1} port2={robot.config.port2}")
        print_robot_status(robot)
    except Exception as e:
        print(f"❌ 连接失败: {e}")
        return

    keyboard = SSHKeyboard()
    keyboard.connect()

    obs = robot.get_observation()
    if robot_config.enable_left_arm: left_arm = SimpleTeleopArm(SO101Kinematics(), LEFT_JOINT_MAP, obs, "left")
    if robot_config.enable_right_arm: right_arm = SimpleTeleopArm(SO101Kinematics(), RIGHT_JOINT_MAP, obs, "right")
    if robot_config.enable_head: head_control = SimpleHeadControl(obs)

    print("🎮 控制已启动 (按 'b' 退出)...")
    try:
        while True:
            pressed_keys = keyboard.get_pressed_keys()

            if robot.teleop_keys['quit'] in pressed_keys:
                print("退出程序...")
                break

            action = {}
            if robot.config.enable_left_arm or robot.config.enable_head:
                left_key_state = {action: (key in pressed_keys) for action, key in LEFT_KEYMAP.items()}
                if robot.config.enable_left_arm:
                    if left_key_state.get('reset'): left_arm.move_to_zero_position(robot); continue
                    left_arm.handle_keys(left_key_state);
                    action.update(left_arm.p_control_action(robot))
                if robot.config.enable_head:
                    if '?' in pressed_keys: head_control.move_to_zero_position(robot); continue
                    head_control.handle_keys(left_key_state);
                    action.update(head_control.p_control_action(robot))
            if robot.config.enable_right_arm:
                right_key_state = {action: (key in pressed_keys) for action, key in RIGHT_KEYMAP.items()}
                if right_key_state.get('reset'): right_arm.move_to_zero_position(robot); continue
                right_arm.handle_keys(right_key_state);
                action.update(right_arm.p_control_action(robot))

            if robot.config.enable_base:
                vx, vy, omega = 0.0, 0.0, 0.0
                speed, rot_speed = 0.05, 20
                if 'i' in pressed_keys: vx += speed
                if 'k' in pressed_keys: vx -= speed
                if 'j' in pressed_keys: vy += speed
                if 'l' in pressed_keys: vy -= speed
                if 'u' in pressed_keys: omega += rot_speed
                if 'o' in pressed_keys: omega -= rot_speed
                action.update({"x.vel": vx, "y.vel": vy, "theta.vel": omega})

            robot.send_action(action)
            time.sleep(1.0 / FPS)

    finally:
        print("正在断开连接...")
        robot.disconnect()
        keyboard.disconnect()
        print("已安全断开。")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot_id", type=str)
    args = parser.parse_args()
    main(args.robot_id)