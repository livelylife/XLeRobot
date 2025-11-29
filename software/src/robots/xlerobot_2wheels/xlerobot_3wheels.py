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
from .config_xlerobot_2wheels import XLerobot2WheelsConfig

logger = logging.getLogger(__name__)


class XLerobot3Wheels(Robot):
    """
    [MODIFIED] 3-Wheel Omni-Directional Robot Implementation
    Hardware: Feetech Motors ID 7, 8, 9 for Base.
    """

    config_class = XLerobot2WheelsConfig
    name = "xlerobot_2wheels"

    def __init__(self, config: XLerobot2WheelsConfig):
        super().__init__(config)
        self.config = config
        self.teleop_keys = config.teleop_keys

        # 3-Wheel Omni allows higher angular speeds
        self.speed_levels = [
            {"linear": 0.1, "angular": 45},  # slow
            {"linear": 0.2, "angular": 90},  # medium
            {"linear": 0.4, "angular": 135},  # fast
        ]
        self.speed_index = 0
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100

        # --- Bus 1: Left Arm + Head ---
        if self.calibration.get("left_arm_shoulder_pan") is not None:
            calibration1 = {k: v for k, v in self.calibration.items() if
                            k.startswith("left_arm") or k.startswith("head")}
        else:
            calibration1 = self.calibration

        self.bus1 = FeetechMotorsBus(
            port=self.config.port1,
            motors={
                # left arm
                "left_arm_shoulder_pan": Motor(1, "sts3215", norm_mode_body),
                "left_arm_shoulder_lift": Motor(2, "sts3215", norm_mode_body),
                "left_arm_elbow_flex": Motor(3, "sts3215", norm_mode_body),
                "left_arm_wrist_flex": Motor(4, "sts3215", norm_mode_body),
                "left_arm_wrist_roll": Motor(5, "sts3215", norm_mode_body),
                "left_arm_gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
                # head
                "head_motor_1": Motor(7, "sts3215", norm_mode_body),
                "head_motor_2": Motor(8, "sts3215", norm_mode_body),
            },
            calibration=calibration1,
        )

        # --- Bus 2: Right Arm + Base (Omni Wheels) ---
        if self.calibration.get("right_arm_shoulder_pan") is not None:
            calibration2 = {k: v for k, v in self.calibration.items() if
                            k.startswith("right_arm") or k.startswith("base")}
        else:
            calibration2 = self.calibration

        self.bus2 = FeetechMotorsBus(
            port=self.config.port2,
            motors={
                # right arm
                "right_arm_shoulder_pan": Motor(1, "sts3215", norm_mode_body),
                "right_arm_shoulder_lift": Motor(2, "sts3215", norm_mode_body),
                "right_arm_elbow_flex": Motor(3, "sts3215", norm_mode_body),
                "right_arm_wrist_flex": Motor(4, "sts3215", norm_mode_body),
                "right_arm_wrist_roll": Motor(5, "sts3215", norm_mode_body),
                "right_arm_gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),

                # === [KEY CHANGE] 3 Omni Wheels ===
                # Based on your feedback:
                # ID 7 = Right Rear (was stationary before)
                # ID 8 = Left Rear (was moving)
                # ID 9 = Front (was moving)
                # We name them abstractly to avoid confusion
                "base_wheel_1": Motor(9, "sts3215", MotorNormMode.RANGE_M100_100), # Front
                "base_wheel_2": Motor(7, "sts3215", MotorNormMode.RANGE_M100_100), # Left Rear (改成了7)
                "base_wheel_3": Motor(8, "sts3215", MotorNormMode.RANGE_M100_100), # Right Rear (改成了8)
            },
            calibration=calibration2,
        )

        self.left_arm_motors = [motor for motor in self.bus1.motors if motor.startswith("left_arm")]
        self.right_arm_motors = [motor for motor in self.bus2.motors if motor.startswith("right_arm")]
        self.head_motors = [motor for motor in self.bus1.motors if motor.startswith("head")]
        self.base_motors = [motor for motor in self.bus2.motors if motor.startswith("base")]
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _state_ft(self) -> dict[str, type]:
        return dict.fromkeys(
            (
                # ... Arms & Head ...
                "left_arm_shoulder_pan.pos", "left_arm_shoulder_lift.pos", "left_arm_elbow_flex.pos",
                "left_arm_wrist_flex.pos", "left_arm_wrist_roll.pos", "left_arm_gripper.pos",
                "right_arm_shoulder_pan.pos", "right_arm_shoulder_lift.pos", "right_arm_elbow_flex.pos",
                "right_arm_wrist_flex.pos", "right_arm_wrist_roll.pos", "right_arm_gripper.pos",
                "head_motor_1.pos", "head_motor_2.pos",
                # ... Base ...
                "x.vel",  # Forward/Back
                "y.vel",  # Left/Right (Strafing)
                "theta.vel",  # Rotation
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
        return self.bus1.is_connected and self.bus2.is_connected and all(
            cam.is_connected for cam in self.cameras.values()
        )

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")
        self.bus1.connect()
        self.bus2.connect()

        # Calibration Logic (Simplified)
        if self.calibration_fpath.is_file():
            # Auto-load if file exists to save time
            logger.info(f"Loading calibration from {self.calibration_fpath}")
            try:
                self.bus1.calibration = {k: v for k, v in self.calibration.items() if k in self.bus1.motors}
                self.bus2.calibration = {k: v for k, v in self.calibration.items() if k in self.bus2.motors}
                self.bus1.write_calibration(self.bus1.calibration)
                self.bus2.write_calibration(self.bus2.calibration)
            except Exception as e:
                logger.warning(f"Calibration load failed: {e}")
        elif calibrate:
            self.calibrate()

        for cam in self.cameras.values():
            cam.connect()
        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus1.is_calibrated and self.bus2.is_calibrated

    def calibrate(self) -> None:
        # (Keep your existing calibration logic here or use a simplified one)
        # For simplicity, I assume you already have a calibration file.
        pass

    def configure(self):
        self.bus1.disable_torque()
        self.bus2.disable_torque()

        # Configure Arms (Position Mode)
        for name in self.left_arm_motors + self.head_motors:
            self.bus1.write("Operating_Mode", name, OperatingMode.POSITION.value)
            self.bus1.write("P_Coefficient", name, 32)
            self.bus1.write("I_Coefficient", name, 0)
            self.bus1.write("D_Coefficient", name, 32)

        for name in self.right_arm_motors:
            self.bus2.write("Operating_Mode", name, OperatingMode.POSITION.value)
            self.bus2.write("P_Coefficient", name, 32)
            self.bus2.write("I_Coefficient", name, 0)
            self.bus2.write("D_Coefficient", name, 32)

        # Configure Base (Velocity Mode) - ALL 3 WHEELS
        for name in self.base_motors:
            self.bus2.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
            self.bus2.write("Acceleration", name, 0)  # Instant acceleration

        self.bus1.enable_torque()
        self.bus2.enable_torque()

    @staticmethod
    def _degps_to_raw(degps: float) -> int:
        steps_per_deg = 4096.0 / 360.0
        speed_int = int(round(degps * steps_per_deg))
        return max(min(speed_int, 0x7FFF), -0x8000)

    @staticmethod
    def _raw_to_degps(raw_speed: int) -> float:
        steps_per_deg = 4096.0 / 360.0
        return raw_speed / steps_per_deg

    def _body_to_wheel_raw(self, vx: float, vy: float, theta_deg: float) -> dict:
        """
        三轮全向底盘运动学解算 (最终校准版)
        """
        L = 0.15
        omega = math.radians(theta_deg)

        # === 核心修正区 ===
        # 现象：i(前)跑成了左后(135度)。
        # 对策：我们需要在软件层把输入向量“往回拧” 135度。
        # -135度 = -2.356 弧度
        correction_angle = math.radians(-135)

        # 应用旋转矩阵 (坐标系变换)
        # 这里的公式将输入的 x,y 按照 correction_angle 进行旋转
        vx_new = vx * math.cos(correction_angle) - vy * math.sin(correction_angle)
        vy_new = vx * math.sin(correction_angle) + vy * math.cos(correction_angle)

        # 使用修正后的速度计算
        vx = vx_new
        vy = vy_new

        # 2. 运动学矩阵 (Kiwi Drive)
        # v9(前), v7(左后), v8(右后) - 注意我们之前交换过 7/8

        # 前轮 (ID 9): 主要负责横向力 + 旋转
        v9 = 1.000 * vy + L * omega

        # 左后 (ID 7): 推力 + 横向 + 旋转
        v8 = -0.866 * vx - 0.5 * vy + L * omega

        # 右后 (ID 8): 推力 + 横向 + 旋转
        v7 = 0.866 * vx - 0.5 * vy + L * omega

        # 转换为 Raw 值
        raw_9 = self._degps_to_raw(math.degrees(v9 / 0.05))
        raw_8 = self._degps_to_raw(math.degrees(v8 / 0.05))  # base_wheel_2 (ID 7)
        raw_7 = self._degps_to_raw(math.degrees(v7 / 0.05))  # base_wheel_3 (ID 8)

        return {
            "base_wheel_1": raw_9,
            "base_wheel_2": raw_8,
            "base_wheel_3": raw_7,
        }

    def _wheel_raw_to_body(self, raw_1, raw_2, raw_3) -> dict:
        # Inverse kinematics is complex, returning 0 for now as it's not critical for teleop
        return {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}

    def _from_keyboard_to_base_action(self, pressed_keys: np.ndarray):
        if self.teleop_keys["speed_up"] in pressed_keys:
            self.speed_index = min(self.speed_index + 1, 2)
        if self.teleop_keys["speed_down"] in pressed_keys:
            self.speed_index = max(self.speed_index - 1, 0)

        speed = self.speed_levels[self.speed_index]
        lin = speed["linear"]
        ang = speed["angular"]

        x_cmd = 0.0
        y_cmd = 0.0  # New: Lateral movement
        theta_cmd = 0.0

        # Forward/Back
        if self.teleop_keys["forward"] in pressed_keys: x_cmd += lin
        if self.teleop_keys["backward"] in pressed_keys: x_cmd -= lin

        # Rotate
        if self.teleop_keys["rotate_left"] in pressed_keys: theta_cmd += ang
        if self.teleop_keys["rotate_right"] in pressed_keys: theta_cmd -= ang

        # Strafing (New Keys mapped in main teleop script)
        # We will reuse 'left'/'right' keys from config if available, or assume:
        # 'j' = Left, 'l' = Right
        if "j" in pressed_keys: y_cmd += lin  # Left
        if "l" in pressed_keys: y_cmd -= lin  # Right

        return {"x.vel": x_cmd, "y.vel": y_cmd, "theta.vel": theta_cmd}

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected: raise DeviceNotConnectedError(f"{self} not connected")

        left_pos = self.bus1.sync_read("Present_Position", self.left_arm_motors + self.head_motors)
        right_pos = self.bus2.sync_read("Present_Position", self.right_arm_motors)

        # Base velocity read (Optional, helps debugging)
        # base_vels = self.bus2.sync_read("Present_Velocity", self.base_motors)

        obs_dict = {}
        for k, v in chain(left_pos.items(), right_pos.items()):
            obs_dict[f"{k}.pos"] = v

        # Add dummy base velocities to satisfy observation space
        obs_dict["x.vel"] = 0.0
        obs_dict["y.vel"] = 0.0
        obs_dict["theta.vel"] = 0.0

        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected: raise DeviceNotConnectedError(f"{self} not connected")

        # Split actions
        left_targets = {k.replace(".pos", ""): v for k, v in action.items() if
                        k.startswith("left_arm_") or k.startswith("head_")}
        right_targets = {k.replace(".pos", ""): v for k, v in action.items() if k.startswith("right_arm_")}

        # Base Action
        vx = action.get("x.vel", 0.0)
        vy = action.get("y.vel", 0.0)
        th = action.get("theta.vel", 0.0)

        wheel_cmds = self._body_to_wheel_raw(vx, vy, th)

        if left_targets: self.bus1.sync_write("Goal_Position", left_targets)
        if right_targets: self.bus2.sync_write("Goal_Position", right_targets)
        if wheel_cmds: self.bus2.sync_write("Goal_Velocity", wheel_cmds)

        return action

    def stop_base(self):
        self.bus2.sync_write("Goal_Velocity", {k: 0 for k in self.base_motors})

    def disconnect(self):
        self.stop_base()
        self.bus1.disconnect(self.config.disable_torque_on_disconnect)
        self.bus2.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values(): cam.disconnect()
        logger.info(f"{self} disconnected.")