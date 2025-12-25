#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time
from functools import cached_property
from itertools import chain
from typing import Any
import math
import numpy as np

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_xlerobot import XLerobotConfig

logger = logging.getLogger(__name__)


class XLerobot(Robot):
    """
    [MODIFIED for joyandai] 3-Wheel Omni-Directional Robot with selective hardware connection.
    """

    config_class = XLerobotConfig
    name = "xlerobot"

    def __init__(self, config: XLerobotConfig):
        super().__init__(config)
        self.config = config
        self.teleop_keys = config.teleop_keys

        self.speed_levels = [
            {"xy": 0.1, "theta": 45},  # slow
            {"xy": 0.25, "theta": 90},  # medium
            {"xy": 0.4, "theta": 135},  # fast
        ]
        self.speed_index = 0
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100

        # Initialize buses to None
        self.bus1 = None
        self.bus2 = None

        # --- Bus 1: Left Arm + Head (Conditional Initialization) ---
        motors1 = {}
        if self.config.enable_left_arm:
            motors1.update({f"left_arm_{j}": Motor(i + 1, "sts3215", norm_mode_body) for i, j in enumerate(
                ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"])})
        if self.config.enable_head:
            motors1.update({"head_motor_1": Motor(7, "sts3215", norm_mode_body),
                            "head_motor_2": Motor(8, "sts3215", norm_mode_body)})

        if motors1:
            calibration1 = {k: v for k, v in self.calibration.items() if k in motors1}
            self.bus1 = FeetechMotorsBus(port=self.config.port1, motors=motors1, calibration=calibration1)

        # --- Bus 2: Right Arm + Base (Conditional Initialization) ---
        motors2 = {}
        if self.config.enable_right_arm:
            motors2.update({f"right_arm_{j}": Motor(i + 1, "sts3215", norm_mode_body) for i, j in enumerate(
                ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"])})
        if self.config.enable_base:
            motors2.update(
                {f"base_wheel_{i + 1}": Motor(i + 7, "sts3215", MotorNormMode.RANGE_M100_100) for i in range(3)})

        if motors2:
            calibration2 = {k: v for k, v in self.calibration.items() if k in motors2}
            self.bus2 = FeetechMotorsBus(port=self.config.port2, motors=motors2, calibration=calibration2)

        self.left_arm_motors = [m for m in (self.bus1.motors if self.bus1 else []) if m.startswith("left_arm")]
        self.right_arm_motors = [m for m in (self.bus2.motors if self.bus2 else []) if m.startswith("right_arm")]
        self.head_motors = [m for m in (self.bus1.motors if self.bus1 else []) if m.startswith("head")]
        self.base_motors = [m for m in (self.bus2.motors if self.bus2 else []) if m.startswith("base")]
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _state_ft(self) -> dict[str, type]:
        # This defines the structure for observations and actions. Keep it complete.
        return dict.fromkeys(
            (
                "left_arm_shoulder_pan.pos", "left_arm_shoulder_lift.pos", "left_arm_elbow_flex.pos",
                "left_arm_wrist_flex.pos", "left_arm_wrist_roll.pos", "left_arm_gripper.pos",
                "right_arm_shoulder_pan.pos", "right_arm_shoulder_lift.pos", "right_arm_elbow_flex.pos",
                "right_arm_wrist_flex.pos", "right_arm_wrist_roll.pos", "right_arm_gripper.pos",
                "head_motor_1.pos", "head_motor_2.pos",
                "x.vel", "y.vel", "theta.vel",
            ),
            float,
        )

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras}

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._state_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._state_ft

    @property
    def is_connected(self) -> bool:
        bus1_ok = (self.bus1 is None) or self.bus1.is_connected
        bus2_ok = (self.bus2 is None) or self.bus2.is_connected
        return bus1_ok and bus2_ok and all(cam.is_connected for cam in self.cameras.values())

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected and (self.bus1 is not None or self.bus2 is not None):
            raise DeviceAlreadyConnectedError(f"{self} already connected")
        if self.bus1: self.bus1.connect()
        if self.bus2: self.bus2.connect()

        if self.calibration_fpath.is_file():
            logger.info(f"Loading calibration from {self.calibration_fpath}")
            try:
                if self.bus1 and self.bus1.motors: self.bus1.calibration = {k: v for k, v in self.calibration.items() if
                                                                            k in self.bus1.motors}
                if self.bus2 and self.bus2.motors: self.bus2.calibration = {k: v for k, v in self.calibration.items() if
                                                                            k in self.bus2.motors}
                if self.bus1 and self.bus1.calibration: self.bus1.write_calibration(self.bus1.calibration)
                if self.bus2 and self.bus2.calibration: self.bus2.write_calibration(self.bus2.calibration)
                logger.info("Calibration restored.")
            except Exception as e:
                logger.warning(f"Failed to restore calibration: {e}")
                if calibrate: self.calibrate()
        elif calibrate:
            self.calibrate()
        for cam in self.cameras.values(): cam.connect()
        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        bus1_calib = (self.bus1 is None) or self.bus1.is_calibrated
        bus2_calib = (self.bus2 is None) or self.bus2.is_calibrated
        return bus1_calib and bus2_calib

    def calibrate(self) -> None:
        logger.info(f"\nRunning calibration of {self}")
        calibration_left, calibration_right = {}, {}
        if self.bus1:
            left_motors = self.left_arm_motors + self.head_motors
            self.bus1.disable_torque()
            for name in left_motors: self.bus1.write("Operating_Mode", name, OperatingMode.POSITION.value)
            input("Move left arm and head motors to the middle of their range and press ENTER....")
            homing_offsets = self.bus1.set_half_turn_homings(left_motors)
            print("Move all left arm and head joints through their ranges. Press ENTER to stop...")
            range_mins, range_maxes = self.bus1.record_ranges_of_motion(left_motors)
            for name, motor in self.bus1.motors.items():
                calibration_left[name] = MotorCalibration(id=motor.id, drive_mode=0,
                                                          homing_offset=homing_offsets.get(name, 0),
                                                          range_min=range_mins.get(name, 0),
                                                          range_max=range_maxes.get(name, 4095))
            self.bus1.write_calibration(calibration_left)
        if self.bus2:
            right_motors = self.right_arm_motors + self.base_motors
            self.bus2.disable_torque(self.right_arm_motors)
            for name in self.right_arm_motors: self.bus2.write("Operating_Mode", name, OperatingMode.POSITION.value)
            input("Move right arm motors to the middle of their range and press ENTER....")
            homing_offsets = self.bus2.set_half_turn_homings(self.right_arm_motors)
            full_turn_motor, unknown_range_motors = self.base_motors, self.right_arm_motors
            print(f"Move all right arm joints through their ranges. Press ENTER to stop...")
            range_mins, range_maxes = self.bus2.record_ranges_of_motion(unknown_range_motors)
            for name in full_turn_motor: range_mins[name], range_maxes[name], homing_offsets[name] = 0, 4095, 0
            for name, motor in self.bus2.motors.items():
                calibration_right[name] = MotorCalibration(id=motor.id, drive_mode=0,
                                                           homing_offset=homing_offsets.get(name, 0),
                                                           range_min=range_mins.get(name, 0),
                                                           range_max=range_maxes.get(name, 4095))
            self.bus2.write_calibration(calibration_right)
        self.calibration = {**calibration_left, **calibration_right}
        if self.calibration:
            self._save_calibration()
            print("Calibration saved to", self.calibration_fpath)

    def configure(self):
        if self.bus1:
            self.bus1.disable_torque()
            for name in self.left_arm_motors + self.head_motors: self.bus1.write("Operating_Mode", name,
                                                                                 OperatingMode.POSITION.value)
            self.bus1.enable_torque()
        if self.bus2:
            self.bus2.disable_torque()
            for name in self.right_arm_motors: self.bus2.write("Operating_Mode", name, OperatingMode.POSITION.value)
            for name in self.base_motors: self.bus2.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
            self.bus2.enable_torque()

    def setup_motors(self) -> None:
        if self.bus1:
            for motor in chain(reversed(self.left_arm_motors), reversed(self.head_motors)):
                input(f"Connect board to '{motor}' motor only and press enter.")
                self.bus1.setup_motor(motor)
        if self.bus2:
            for motor in chain(reversed(self.right_arm_motors), reversed(self.base_motors)):
                input(f"Connect board to '{motor}' motor only and press enter.")
                self.bus2.setup_motor(motor)

    @staticmethod
    def _degps_to_raw(degps: float) -> int:
        speed_int = int(round(degps * (4096.0 / 360.0)))
        return max(min(speed_int, 32767), -32768)

    @staticmethod
    def _raw_to_degps(raw_speed: int) -> float:
        return raw_speed / (4096.0 / 360.0)

    def _body_to_wheel_raw(
            self,
            x: float,
            y: float,
            theta: float,
            wheel_radius: float = 0.05,
            base_radius: float = 0.125,
            max_raw: int = 3000,
    ) -> dict:
        """
        [Final Calibration] 3-Wheel Omni Kinematics with precise -150 degree correction.
        """
        # 将旋转速度从 deg/s 转换为 rad/s
        theta_rad = math.radians(theta)

        # === 1. 坐标系旋转修正 (核心) ===
        # 根据你的反馈 "前进走向10点钟方向"，我们将修正角度精确地设置为 -150 度
        correction_angle = math.radians(-175)

        # 应用旋转矩阵，得到修正后的 vx 和 vy
        vx_new = x * math.cos(correction_angle) - y * math.sin(correction_angle)
        vy_new = x * math.sin(correction_angle) + y * math.cos(correction_angle)

        # 使用修正后的速度进行后续计算
        x, y = vx_new, vy_new

        # === 2. 运动学矩阵 (Kiwi Drive) ===
        # 这个矩阵定义了三个轮子在物理上的角度分布
        # 既然旋转是好的，这个矩阵就是正确的，保持不变
        angles = np.radians(np.array([150, 270, 30]))
        velocity_vector = np.array([x, y, theta_rad])
        m = np.array([[np.cos(a), np.sin(a), base_radius] for a in angles])

        # 计算轮速 (deg/s)
        wheel_degps = m.dot(velocity_vector) / wheel_radius * (180.0 / np.pi)

        # === 3. 速度限幅 ===
        steps_per_deg = 4096.0 / 360.0
        raw_floats = [abs(degps) * steps_per_deg for degps in wheel_degps]
        if raw_floats and (max_val := max(raw_floats)) > max_raw:
            scale = max_raw / max_val
            wheel_degps *= scale

        # === 4. 转换为电机原始指令 ===
        wheel_raw = [self._degps_to_raw(deg) for deg in wheel_degps]

        # === 5. 返回指令字典 (保持原始极性，不加负号) ===
        # 既然旋转是好的，就证明这个组合是正确的
        return {
            "base_wheel_1": wheel_raw[0], # ID 7
            "base_wheel_2": wheel_raw[1], # ID 8
            "base_wheel_3": wheel_raw[2], # ID 9
        }

    def _wheel_raw_to_body(self, raw_1, raw_2, raw_3, wheel_radius: float = 0.05, base_radius: float = 0.125):
        return {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}

    def _from_keyboard_to_base_action(self, pressed_keys: np.ndarray):
        if self.teleop_keys["speed_up"] in pressed_keys: self.speed_index = min(self.speed_index + 1, 2)
        if self.teleop_keys["speed_down"] in pressed_keys: self.speed_index = max(self.speed_index - 1, 0)
        speed = self.speed_levels[self.speed_index]
        xy, th = speed["xy"], speed["theta"]
        x, y, theta = 0.0, 0.0, 0.0
        if self.teleop_keys["forward"] in pressed_keys: x += xy
        if self.teleop_keys["backward"] in pressed_keys: x -= xy
        if self.teleop_keys["left"] in pressed_keys: y += xy
        if self.teleop_keys["right"] in pressed_keys: y -= xy
        if self.teleop_keys["rotate_left"] in pressed_keys: theta += th
        if self.teleop_keys["rotate_right"] in pressed_keys: theta -= th
        return {"x.vel": x, "y.vel": y, "theta.vel": theta}

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected: raise DeviceNotConnectedError(f"{self} not connected")
        obs_dict = {}
        if self.bus1:
            motors_to_read1 = self.left_arm_motors + self.head_motors
            if motors_to_read1:
                pos1 = self.bus1.sync_read("Present_Position", motors_to_read1)
                for k, v in pos1.items(): obs_dict[f"{k}.pos"] = v
        if self.bus2:
            motors_to_read2 = self.right_arm_motors
            if motors_to_read2:
                pos2 = self.bus2.sync_read("Present_Position", motors_to_read2)
                for k, v in pos2.items(): obs_dict[f"{k}.pos"] = v

        obs_dict.update({"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0})
        for cam_key, cam in self.cameras.items(): obs_dict[cam_key] = cam.async_read()
        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected: raise DeviceNotConnectedError(f"{self} not connected")
        if self.bus1:
            targets1 = {k.replace(".pos", ""): v for k, v in action.items() if
                        k.startswith("left_arm_") or k.startswith("head_")}
            if targets1: self.bus1.sync_write("Goal_Position", targets1)
        if self.bus2:
            targets2 = {k.replace(".pos", ""): v for k, v in action.items() if k.startswith("right_arm_")}
            if targets2: self.bus2.sync_write("Goal_Position", targets2)
            vx, vy, th = action.get("x.vel", 0.0), action.get("y.vel", 0.0), action.get("theta.vel", 0.0)
            wheel_cmds = self._body_to_wheel_raw(vx, vy, th)
            if self.base_motors: self.bus2.sync_write("Goal_Velocity", wheel_cmds)
        return action

    def stop_base(self):
        if self.bus2 and self.bus2.is_connected and self.base_motors:
            self.bus2.sync_write("Goal_Velocity", dict.fromkeys(self.base_motors, 0), num_retry=3)

    def disconnect(self):
        # [MODIFIED] Ensure stop_base is called before disconnecting buses
        self.stop_base()
        if self.bus1 and self.bus1.is_connected: self.bus1.disconnect(self.config.disable_torque_on_disconnect)
        if self.bus2 and self.bus2.is_connected: self.bus2.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values(): cam.disconnect()
        logger.info(f"{self} disconnected.")