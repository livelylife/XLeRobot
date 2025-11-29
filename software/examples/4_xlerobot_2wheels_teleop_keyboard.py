# To Run on the host
'''python
PYTHONPATH=src python -m lerobot.robots.xlerobot_2wheels.xlerobot_2wheels_host --robot.id=my_xlerobot_2wheels
'''

# To Run the teleop:
'''python
PYTHONPATH=src python -m examples.xlerobot_2wheels.teleoperate_Keyboard
'''

import time
import numpy as np
import math

# import sys
# sys.path.append("/home/joyandai/workspace/lerobot/src/")
import sys
import select
import tty
import termios
import threading

class SSHKeyboard:
    """
    一个兼容 SSH 终端的键盘监听器，替换 LeRobot 的图形化 KeyboardTeleop。
    """
    def __init__(self):
        self.keys = {}
        self.running = False
        self.thread = None
        self.settings = termios.tcgetattr(sys.stdin)

    def connect(self):
        self.running = True
        self.thread = threading.Thread(target=self._listen, daemon=True)
        self.thread.start()
        print("[SSHKeyboard] Keyboard listener started. Control via SSH terminal.")

    def disconnect(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        print("[SSHKeyboard] Disconnected.")

    def get_action(self):
        # 返回当前按下的键，兼容 lerobot 的接口
        # 注意：终端模式下通常只能检测到“刚刚按下”，很难检测“一直按住”
        # 这里返回所有捕获到的键，读取后会清空（模拟按下事件）
        active_keys = self.keys.copy()
        self.keys.clear()  # 清除，防止一次按键被无限循环读取
        return active_keys

    def _listen(self):
        try:
            tty.setcbreak(sys.stdin.fileno())
            while self.running:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key:
                        self.keys[key] = True
                        # 特殊处理：如果是 ctrl+c，强制退出
                        if key == '\x03':
                            self.running = False
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

from lerobot.robots.xlerobot_2wheels import XLerobot2WheelsClient, XLerobot2WheelsClientConfig, XLerobot2WheelsConfig, XLerobot3Wheels
# from lerobot.utils.robot_utils import busy_wait
# from lerobot.utils.visualization_utils import init_rerun, log_rerun_data
import rerun as rr  # <--- 新增这行
from lerobot.utils.visualization_utils import log_rerun_data
from lerobot.model.SO101Robot import SO101Kinematics
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig

# Base speed control parameters - adjustable slopes
BASE_ACCELERATION_RATE = 10.0  # acceleration slope (speed/second)
BASE_DECELERATION_RATE = 10  # deceleration slope (speed/second) - very slow for noticeable deceleration
BASE_MAX_SPEED = 6.0          # maximum speed multiplier
MIN_VELOCITY_THRESHOLD = 0.02 # minimum velocity to send to motors during deceleration

# Keymaps (semantic action: key) - Updated for differential drive
LEFT_KEYMAP = {
    'shoulder_pan+': 'q', 'shoulder_pan-': 'e',
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
    'shoulder_pan+': '7', 'shoulder_pan-': '9',
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
    """
    Generates a rectangular trajectory on the x-y plane with sinusoidal velocity profiles.
    The rectangle is divided into 4 line segments, each with smooth acceleration/deceleration.
    """
    def __init__(self, width=0.06, height=0.06, segment_duration=0.91):
        """
        Initialize rectangular trajectory parameters.
        
        Args:
            width: Rectangle width in meters
            height: Rectangle height in meters  
            segment_duration: Time for each line segment in seconds
        """
        self.width = width
        self.height = height
        self.segment_duration = segment_duration
        self.total_duration = 4 * segment_duration
        
    def get_trajectory_point(self, current_x, current_y, t):
        """
        Get the target x, y position at time t for the rectangular trajectory.
        
        Args:
            current_x: Starting x position
            current_y: Starting y position
            t: Time since trajectory start (0 to total_duration)
            
        Returns:
            tuple: (target_x, target_y)
        """
        # Determine which segment we're in
        segment = int(t / self.segment_duration)
        segment_t = t % self.segment_duration
        
        # Normalize segment time (0 to 1)
        normalized_t = segment_t / self.segment_duration
        
        # Sinusoidal velocity profile: smooth acceleration and deceleration
        # s(t) = 0.5 * (1 - cos(π * t)) gives smooth 0 to 1 transition
        smooth_t = 0.5 * (1 - math.cos(math.pi * normalized_t))
        
        # Define rectangle corners relative to starting position
        corners = [
            (current_x, current_y),                           # Start (bottom-left)
            (current_x + self.width, current_y),              # Bottom-right
            (current_x + self.width, current_y + self.height), # Top-right  
            (current_x, current_y + self.height),             # Top-left
            (current_x, current_y)                            # Back to start
        ]
        
        # Clamp segment to valid range
        segment = max(0, min(3, segment))
        
        # Interpolate between current corner and next corner
        start_corner = corners[segment]
        end_corner = corners[segment + 1]
        
        target_x = start_corner[0] + smooth_t * (end_corner[0] - start_corner[0])
        target_y = start_corner[1] + smooth_t * (end_corner[1] - start_corner[1])
        
        return target_x, target_y

class SimpleHeadControl:
    def __init__(self, initial_obs, kp=0.81):
        self.kp = kp
        self.degree_step = 1
        # Initialize head motor positions
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
        if key_state.get('head_motor_1+'):
            self.target_positions["head_motor_1"] += self.degree_step
            print(f"[HEAD] head_motor_1: {self.target_positions['head_motor_1']}")
        if key_state.get('head_motor_1-'):
            self.target_positions["head_motor_1"] -= self.degree_step
            print(f"[HEAD] head_motor_1: {self.target_positions['head_motor_1']}")
        if key_state.get('head_motor_2+'):
            self.target_positions["head_motor_2"] += self.degree_step
            print(f"[HEAD] head_motor_2: {self.target_positions['head_motor_2']}")
        if key_state.get('head_motor_2-'):
            self.target_positions["head_motor_2"] -= self.degree_step
            print(f"[HEAD] head_motor_2: {self.target_positions['head_motor_2']}")

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
        self.prefix = prefix  # To distinguish left and right arm
        self.kp = kp
        # Initial joint positions
        self.joint_positions = {
            "shoulder_pan": initial_obs[f"{prefix}_arm_shoulder_pan.pos"],
            "shoulder_lift": initial_obs[f"{prefix}_arm_shoulder_lift.pos"],
            "elbow_flex": initial_obs[f"{prefix}_arm_elbow_flex.pos"],
            "wrist_flex": initial_obs[f"{prefix}_arm_wrist_flex.pos"],
            "wrist_roll": initial_obs[f"{prefix}_arm_wrist_roll.pos"],
            "gripper": initial_obs[f"{prefix}_arm_gripper.pos"],
        }
        # Set initial x/y to fixed values
        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0
        # Set the degree step and xy step
        self.degree_step = 3
        self.xy_step = 0.0081
        # Set target positions to zero for P control
        self.target_positions = {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 0.0,
        }
        self.zero_pos = {
            'shoulder_pan': 0.0,
            'shoulder_lift': 0.0,
            'elbow_flex': 0.0,
            'wrist_flex': 0.0,
            'wrist_roll': 0.0,
            'gripper': 0.0
        }
        
        # Rectangular trajectory instance
        self.rectangular_trajectory = RectangularTrajectory(
            width=0.06,          # 6cm wide rectangle
            height=0.06,         # 4cm tall rectangle  
            segment_duration=1.01 # 3 seconds per line segment
        )

    def move_to_zero_position(self, robot):
        print(f"[{self.prefix}] Moving to Zero Position: {self.zero_pos} ......")
        self.target_positions = self.zero_pos.copy()  # Use copy to avoid reference issues
        
        # Reset kinematic variables to their initial state
        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0
        
        # Don't let handle_keys recalculate wrist_flex - set it explicitly
        self.target_positions["wrist_flex"] = 0.0
        
        action = self.p_control_action(robot)
        robot.send_action(action)

    def execute_rectangular_trajectory(self, robot, fps=30):
        """
        Execute a blocking rectangular trajectory on the x-y plane.
        
        Args:
            robot: Robot instance to send actions to
            fps: Control loop frequency
        """
        print(f"[{self.prefix}] Starting rectangular trajectory...")
        print(f"[{self.prefix}] Rectangle: {self.rectangular_trajectory.width:.3f}m x {self.rectangular_trajectory.height:.3f}m")
        print(f"[{self.prefix}] Duration: {self.rectangular_trajectory.total_duration:.3f}s total")
        
        # Store starting position
        start_x = self.current_x
        start_y = self.current_y
        
        # Execute trajectory
        start_time = time.time()
        dt = 1.0 / fps
        
        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            # Check if trajectory is complete
            if elapsed_time >= self.rectangular_trajectory.total_duration:
                print(f"[{self.prefix}] Rectangular trajectory completed!")
                break
                
            # Get target position from trajectory
            target_x, target_y = self.rectangular_trajectory.get_trajectory_point(
                start_x, start_y, elapsed_time
            )
            
            # Update current position
            self.current_x = target_x
            self.current_y = target_y
            
            # Calculate inverse kinematics
            try:
                joint2, joint3 = self.kinematics.inverse_kinematics(self.current_x, self.current_y)
                self.target_positions["shoulder_lift"] = joint2
                self.target_positions["elbow_flex"] = joint3
                
                # Update wrist_flex coupling
                self.target_positions["wrist_flex"] = (
                    -self.target_positions["shoulder_lift"]
                    -self.target_positions["elbow_flex"]
                    + self.pitch
                )
                
                # Get action
                action = self.p_control_action(robot)
                
                # Determine which arm is executing and send appropriate action structure
                if self.prefix == "left":
                    # Send left arm action with empty actions for other components
                    robot_action = {**action, **{}, **{}, **{}}
                elif self.prefix == "right":
                    # Send right arm action with empty actions for other components
                    robot_action = {**{}, **action, **{}, **{}}
                
                # Send action to robot
                robot.send_action(robot_action)
                
                # Get observation and log data
                obs = robot.get_observation()
                log_rerun_data(obs, robot_action)
                
            except Exception as e:
                print(f"[{self.prefix}] IK failed at x={self.current_x:.4f}, y={self.current_y:.4f}: {e}")
                break
                
            # Maintain control frequency
            # busy_wait(dt)
        
        print(f"[{self.prefix}] Trajectory execution finished.")

    def handle_keys(self, key_state):
        # Joint increments
        if key_state.get('shoulder_pan+'):
            self.target_positions["shoulder_pan"] += self.degree_step
            print(f"[{self.prefix}] shoulder_pan: {self.target_positions['shoulder_pan']}")
        if key_state.get('shoulder_pan-'):
            self.target_positions["shoulder_pan"] -= self.degree_step
            print(f"[{self.prefix}] shoulder_pan: {self.target_positions['shoulder_pan']}")
        if key_state.get('wrist_roll+'):
            self.target_positions["wrist_roll"] += self.degree_step
            print(f"[{self.prefix}] wrist_roll: {self.target_positions['wrist_roll']}")
        if key_state.get('wrist_roll-'):
            self.target_positions["wrist_roll"] -= self.degree_step
            print(f"[{self.prefix}] wrist_roll: {self.target_positions['wrist_roll']}")
        if key_state.get('gripper+'):
            self.target_positions["gripper"] += self.degree_step
            print(f"[{self.prefix}] gripper: {self.target_positions['gripper']}")
        if key_state.get('gripper-'):
            self.target_positions["gripper"] -= self.degree_step
            print(f"[{self.prefix}] gripper: {self.target_positions['gripper']}")
        if key_state.get('pitch+'):
            self.pitch += self.degree_step
            print(f"[{self.prefix}] pitch: {self.pitch}")
        if key_state.get('pitch-'):
            self.pitch -= self.degree_step
            print(f"[{self.prefix}] pitch: {self.pitch}")

        # XY plane (IK)
        moved = False
        if key_state.get('x+'):
            self.current_x += self.xy_step
            moved = True
            print(f"[{self.prefix}] x+: {self.current_x:.4f}, y: {self.current_y:.4f}")
        if key_state.get('x-'):
            self.current_x -= self.xy_step
            moved = True
            print(f"[{self.prefix}] x-: {self.current_x:.4f}, y: {self.current_y:.4f}")
        if key_state.get('y+'):
            self.current_y += self.xy_step
            moved = True
            print(f"[{self.prefix}] x: {self.current_x:.4f}, y+: {self.current_y:.4f}")
        if key_state.get('y-'):
            self.current_y -= self.xy_step
            moved = True
            print(f"[{self.prefix}] x: {self.current_x:.4f}, y-: {self.current_y:.4f}")
        if moved:
            joint2, joint3 = self.kinematics.inverse_kinematics(self.current_x, self.current_y)
            self.target_positions["shoulder_lift"] = joint2
            self.target_positions["elbow_flex"] = joint3
            print(f"[{self.prefix}] shoulder_lift: {joint2}, elbow_flex: {joint3}")

        # Wrist flex is always coupled to pitch and the other two
        self.target_positions["wrist_flex"] = (
            -self.target_positions["shoulder_lift"]
            -self.target_positions["elbow_flex"]
            + self.pitch
        )
        # print(f"[{self.prefix}] wrist_flex: {self.target_positions['wrist_flex']}")

    def p_control_action(self, robot):
        obs = robot.get_observation()
        current = {j: obs[f"{self.prefix}_arm_{j}.pos"] for j in self.joint_map}
        action = {}
        for j in self.target_positions:
            error = self.target_positions[j] - current[j]
            control = self.kp * error
            action[f"{self.joint_map[j]}.pos"] = current[j] + control
        return action


class SmoothBaseController:
    """
    [修改版] 支持全向移动 (X, Y, Theta)
    j: 左移 (y+)
    l: 右移 (y-)
    i: 前进 (x+)
    k: 后退 (x-)
    u: 左转 (theta+)
    o: 右转 (theta-)
    """

    def __init__(self):
        self.current_speed = 0.0
        self.last_time = time.time()
        self.last_direction = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
        self.is_moving = False

    def update(self, pressed_keys, robot):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # 定义所有控制键
        # 注意：这里硬编码了按键，以确保你的需求生效
        # j/l 可能不在 robot.teleop_keys 配置里，所以我们手动添加
        move_keys = ['i', 'k', 'u', 'o', 'j', 'l']

        any_key_pressed = any(k in pressed_keys for k in move_keys)

        # 初始化动作
        base_action = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}

        if any_key_pressed:
            if not self.is_moving:
                self.is_moving = True

            # 获取速度档位
            speed_setting = robot.speed_levels[robot.speed_index]
            lin_speed = speed_setting["linear"]
            ang_speed = speed_setting["angular"]

            # === 核心按键映射 ===
            # X轴 (前后)
            if 'i' in pressed_keys: base_action["x.vel"] += lin_speed
            if 'k' in pressed_keys: base_action["x.vel"] -= lin_speed

            # Y轴 (左右横移)
            if 'j' in pressed_keys: base_action["y.vel"] += lin_speed  # 左
            if 'l' in pressed_keys: base_action["y.vel"] -= lin_speed  # 右

            # Theta轴 (旋转)
            if 'u' in pressed_keys: base_action["theta.vel"] += ang_speed
            if 'o' in pressed_keys: base_action["theta.vel"] -= ang_speed

            # 记录方向用于减速
            self.last_direction = base_action.copy()

            # 加速逻辑 (直接给满速，避免SSH延迟问题)
            self.current_speed = 1.0

        else:
            if self.is_moving:
                self.is_moving = False
            self.current_speed = 0.0

        # 应用速度系数
        final_action = {
            "x.vel": base_action["x.vel"] * self.current_speed,
            "y.vel": base_action["y.vel"] * self.current_speed,
            "theta.vel": base_action["theta.vel"] * self.current_speed
        }

        return final_action


# Global smooth controller instance
smooth_controller = SmoothBaseController()


def main():
    # Teleop parameters
    FPS = 50
    # ip = "192.168.1.123"  # This is for zmq connection
    ip = "localhost"  # This is for local/wired connection
    # robot_name = "my_xlerobot_2wheels_pc"
    robot_name = "my_xlerobot_2wheels_lab"

    # For zmq connection
    # robot_config = XLerobot2WheelsClientConfig(remote_ip=ip, id=robot_name)
    # robot = XLerobot2WheelsClient(robot_config)    

    # For local/wired connection
    # robot_config = XLerobot2WheelsConfig(id=robot_name)
    # robot = XLerobot2Wheels(robot_config)
    # For local/wired connection
    # 根据之前的测试结果：ACM0 是右手+轮子，ACM1 是左手+头
    robot_config = XLerobot2WheelsConfig(
        id=robot_name,
        # 左手 + 头 (刚才拔掉显示是 ACM2)
        port1='/dev/ttyACM2',

        # 右手 + 底盘 (刚才拔掉显示是 ACM0)
        port2='/dev/ttyACM0'
    )
    robot = XLerobot3Wheels(robot_config)
    
    try:
        robot.connect()
        print(f"[MAIN] Successfully connected to robot")
    except Exception as e:
        print(f"[MAIN] Failed to connect to robot: {e}")
        print(robot_config)
        print(robot)
        return
        
    # init_rerun(session_name="xlerobot_2wheels_teleop")
    # init_rerun(session_name="xlerobot_2wheels_teleop", spawn_local_viewer=False)

    # 使用原生 rerun 初始化，并启动 web 服务而不是本地窗口
    # rr.init("xlerobot_2wheels_teleop")
    # open_browser=False 防止它在 SSH 端尝试打开浏览器
    # 启动后，你可以在电脑浏览器访问 http://<Jetson的IP>:9090 来查看可视化
    # rr.serve(open_browser=False)

    #Init the keyboard instance
    # keyboard_config = KeyboardTeleopConfig()
    # keyboard = KeyboardTeleop(keyboard_config)
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
    print("\n" + "="*80)
    print("🤖 XLeRobot 2Wheels Keyboard Control Keymap")
    print("="*80)
    
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
    
    print(f"\n⚙️ Robot Configuration:")
    print(f"   Wheel Radius: {robot.config.wheel_radius:.3f}m")
    print(f"   Wheelbase: {robot.config.wheelbase:.3f}m")
    print(f"   Speed Levels: {len(robot.speed_levels)} levels")
    for i, level in enumerate(robot.speed_levels):
        print(f"      Level {i+1}: Linear {level['linear']:.1f}m/s, Angular {level['angular']:.0f}°/s")
    
    print(f"\n🚀 Smooth Control Parameters:")
    print(f"   Acceleration Rate: {BASE_ACCELERATION_RATE:.1f} speed/second")
    print(f"   Deceleration Rate: {BASE_DECELERATION_RATE:.1f} speed/second")
    print(f"   Max Speed Multiplier: {BASE_MAX_SPEED:.1f}x")
    
    print("\n" + "="*80)
    print("🎮 Control started! Use above keys to control robot")
    print("="*80 + "\n")

    try:
        while True:
            # pressed_keys = set(keyboard.get_action().keys())
            pressed_keys_dict = keyboard.get_action()
            pressed_keys = set(pressed_keys_dict.keys())
            if pressed_keys:
                print(f"Detected keys: {pressed_keys}")
            left_key_state = {action: (key in pressed_keys) for action, key in LEFT_KEYMAP.items()}
            right_key_state = {action: (key in pressed_keys) for action, key in RIGHT_KEYMAP.items()}

            # Handle rectangular trajectory for left arm (y key)
            if left_key_state.get('triangle'):
                print("[MAIN] Left arm rectangular trajectory triggered!")
                left_arm.execute_rectangular_trajectory(robot, fps=FPS)
                continue

            # Handle rectangular trajectory for right arm (Y key)  
            if right_key_state.get('triangle'):
                print("[MAIN] Right arm rectangular trajectory triggered!")
                right_arm.execute_rectangular_trajectory(robot, fps=FPS)
                continue

            # Handle reset for left arm
            if left_key_state.get('reset'):
                left_arm.move_to_zero_position(robot)
                continue  

            # Handle reset for right arm
            if right_key_state.get('reset'):
                right_arm.move_to_zero_position(robot)
                continue

            # Handle reset for head motors with '?'
            if '?' in pressed_keys:
                head_control.move_to_zero_position(robot)
                continue

            left_arm.handle_keys(left_key_state)
            right_arm.handle_keys(right_key_state)
            head_control.handle_keys(left_key_state)  # Head controlled by left arm keymap

            left_action = left_arm.p_control_action(robot)
            right_action = right_arm.p_control_action(robot)
            head_action = head_control.p_control_action(robot)

            # Get smooth base action with linear acceleration/deceleration
            base_action = smooth_controller.update(pressed_keys, robot)

            action = {**left_action, **right_action, **head_action, **base_action}
            robot.send_action(action)

            obs = robot.get_observation()
            # print(f"[MAIN] Observation: {obs}")
            # log_rerun_data(obs, action)
            # busy_wait(1.0 / FPS)
            time.sleep(1.0 / FPS)
    finally:
        robot.disconnect()
        keyboard.disconnect()
        print("Teleoperation ended.")

if __name__ == "__main__":
    main()
