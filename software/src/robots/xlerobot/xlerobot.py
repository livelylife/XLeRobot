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
    The robot includes a three omniwheel mobile base and a remote follower arm.
    The leader arm is connected locally (on the laptop) and its joint positions are recorded and then
    forwarded to the remote follower arm (after applying a safety clamp).
    In parallel, keyboard teleoperation is used to generate raw velocity commands for the wheels.
    """

    config_class = XLerobotConfig
    name = "xlerobot"

    def __init__(self, config: XLerobotConfig):
        super().__init__(config)
        self.config = config
        self.teleop_keys = config.teleop_keys

        # Define three speed levels and a current index
        # Adjusted for 3-wheel omni capabilities
        self.speed_levels = [
            {"xy": 0.1, "theta": 45},  # slow
            {"xy": 0.25, "theta": 90},  # medium
            {"xy": 0.4, "theta": 135},  # fast
        ]
        self.speed_index = 0  # Start at slow
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

        # --- Bus 2: Right Arm + Base ---
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

                # === Base Motors (3-Wheel Omni) ===
                # Using abstract names 1, 2, 3 to align with kinematic matrix
                # IDs verified: 7, 8, 9
                "base_wheel_1": Motor(7, "sts3215", MotorNormMode.RANGE_M100_100),
                "base_wheel_2": Motor(8, "sts3215", MotorNormMode.RANGE_M100_100),
                "base_wheel_3": Motor(9, "sts3215", MotorNormMode.RANGE_M100_100),
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
                "left_arm_shoulder_pan.pos",
                "left_arm_shoulder_lift.pos",
                "left_arm_elbow_flex.pos",
                "left_arm_wrist_flex.pos",
                "left_arm_wrist_roll.pos",
                "left_arm_gripper.pos",
                "right_arm_shoulder_pan.pos",
                "right_arm_shoulder_lift.pos",
                "right_arm_elbow_flex.pos",
                "right_arm_wrist_flex.pos",
                "right_arm_wrist_roll.pos",
                "right_arm_gripper.pos",
                "head_motor_1.pos",
                "head_motor_2.pos",
                "x.vel",
                "y.vel",
                "theta.vel",
            ),
            float,
        )

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

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

        # Check if calibration file exists and ask user if they want to restore it
        if self.calibration_fpath.is_file():
            logger.info(f"Calibration file found at {self.calibration_fpath}")
            # Try to auto-load to avoid terminal blocking in some environments
            try:
                self.bus1.calibration = {k: v for k, v in self.calibration.items() if k in self.bus1.motors}
                self.bus2.calibration = {k: v for k, v in self.calibration.items() if k in self.bus2.motors}
                self.bus1.write_calibration(self.bus1.calibration)
                self.bus2.write_calibration(self.bus2.calibration)
                logger.info("Calibration restored successfully from file!")
            except Exception as e:
                logger.warning(f"Failed to restore calibration from file: {e}")
                if calibrate:
                    self.calibrate()
        elif calibrate:
            logger.info("No calibration file found, proceeding with manual calibration...")
            self.calibrate()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus1.is_calibrated and self.bus2.is_calibrated

    def calibrate(self) -> None:
        # Calibration logic (simplified for brevity as focus is on kinematics)
        # You can implement full calibration here if needed.
        pass

    def configure(self):
        self.bus1.disable_torque()
        self.bus2.disable_torque()

        # Configure Arms and Head (Position Mode)
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

        # Configure Base (Velocity Mode) - 3 Wheels
        for name in self.base_motors:
            self.bus2.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
            self.bus2.write("Acceleration", name, 0)  # Instant acceleration for responsiveness

        self.bus1.enable_torque()
        self.bus2.enable_torque()

    def setup_motors(self) -> None:
        for motor in chain(reversed(self.left_arm_motors), reversed(self.head_motors)):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus1.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus1.motors[motor].id}")

        # Set up right arm motors
        for motor in chain(reversed(self.right_arm_motors), reversed(self.base_motors)):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus2.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus2.motors[motor].id}")

    @staticmethod
    def _degps_to_raw(degps: float) -> int:
        # Standard conversion for Feetech
        steps_per_deg = 4096.0 / 360.0
        speed_in_steps = degps * steps_per_deg
        speed_int = int(round(speed_in_steps))
        if speed_int > 0x7FFF:
            speed_int = 0x7FFF
        elif speed_int < -0x8000:
            speed_int = -0x8000
        return speed_int

    @staticmethod
    def _raw_to_degps(raw_speed: int) -> float:
        steps_per_deg = 4096.0 / 360.0
        return raw_speed / steps_per_deg

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
        [CORE LOGIC] 3-Wheel Omni Kinematics with Coordinate Correction.
        Converts body frame velocity (x, y, theta) to wheel velocities.
        """
        # Convert rotational velocity from deg/s to rad/s.
        theta_rad = theta * (np.pi / 180.0)

        # === 1. Coordinate System Correction ===
        # Correcting the -135 degree offset discovered during testing.
        correction_angle = math.radians(-135)

        # Apply rotation matrix
        vx_new = x * math.cos(correction_angle) - y * math.sin(correction_angle)
        vy_new = x * math.sin(correction_angle) + y * math.cos(correction_angle)

        # Update velocities to corrected values
        x, y = vx_new, vy_new

        # === 2. Kinematics Matrix (Kiwi Drive) ===
        # Assuming 120-degree separation.
        # Using a standard distribution: 150, 270, 30 degrees
        angles = np.radians(np.array([150, 270, 30]))

        # Create velocity vector [x, y, theta_rad]
        velocity_vector = np.array([x, y, theta_rad])

        # Matrix: [cos(a), sin(a), R_base]
        m = np.array([[np.cos(a), np.sin(a), base_radius] for a in angles])

        # Compute linear speeds (m/s)
        wheel_linear_speeds = m.dot(velocity_vector)
        # Compute angular speeds (rad/s)
        wheel_angular_speeds = wheel_linear_speeds / wheel_radius
        # Convert to deg/s
        wheel_degps = wheel_angular_speeds * (180.0 / np.pi)

        # Scaling to prevent saturation
        steps_per_deg = 4096.0 / 360.0
        raw_floats = [abs(degps) * steps_per_deg for degps in wheel_degps]
        max_raw_computed = max(raw_floats)
        if max_raw_computed > max_raw:
            scale = max_raw / max_raw_computed
            wheel_degps = wheel_degps * scale

        # Convert to raw commands
        wheel_raw = [self._degps_to_raw(deg) for deg in wheel_degps]

        return {
            "base_wheel_1": wheel_raw[0],  # ID 7
            "base_wheel_2": wheel_raw[1],  # ID 8
            "base_wheel_3": wheel_raw[2],  # ID 9
        }

    def _wheel_raw_to_body(
            self,
            raw_1,
            raw_2,
            raw_3,
            wheel_radius: float = 0.05,
            base_radius: float = 0.125,
    ) -> dict[str, Any]:
        """
        Inverse kinematics (Simplified placeholder).
        Since we added rotation correction, exact inverse is complex and unnecessary for teleop.
        Returning dummy 0s to satisfy observation space requirements.
        """
        return {
            "x.vel": 0.0,
            "y.vel": 0.0,
            "theta.vel": 0.0,
        }

    def _from_keyboard_to_base_action(self, pressed_keys: np.ndarray):
        # Helper for keyboard control
        if self.teleop_keys["speed_up"] in pressed_keys:
            self.speed_index = min(self.speed_index + 1, 2)
        if self.teleop_keys["speed_down"] in pressed_keys:
            self.speed_index = max(self.speed_index - 1, 0)

        speed_setting = self.speed_levels[self.speed_index]
        xy_speed = speed_setting["xy"]
        theta_speed = speed_setting["theta"]

        x_cmd = 0.0
        y_cmd = 0.0
        theta_cmd = 0.0

        if self.teleop_keys["forward"] in pressed_keys: x_cmd += xy_speed
        if self.teleop_keys["backward"] in pressed_keys: x_cmd -= xy_speed
        if self.teleop_keys["left"] in pressed_keys: y_cmd += xy_speed
        if self.teleop_keys["right"] in pressed_keys: y_cmd -= xy_speed
        if self.teleop_keys["rotate_left"] in pressed_keys: theta_cmd += theta_speed
        if self.teleop_keys["rotate_right"] in pressed_keys: theta_cmd -= theta_speed

        return {
            "x.vel": x_cmd,
            "y.vel": y_cmd,
            "theta.vel": theta_cmd,
        }

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start = time.perf_counter()

        # Read positions
        left_arm_pos = self.bus1.sync_read("Present_Position", self.left_arm_motors)
        right_arm_pos = self.bus2.sync_read("Present_Position", self.right_arm_motors)
        head_pos = self.bus1.sync_read("Present_Position", self.head_motors)

        # Base velocity read is complex with correction, returning dummy 0
        base_vel = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}

        left_arm_state = {f"{k}.pos": v for k, v in left_arm_pos.items()}
        right_arm_state = {f"{k}.pos": v for k, v in right_arm_pos.items()}
        head_state = {f"{k}.pos": v for k, v in head_pos.items()}

        obs_dict = {**left_arm_state, **right_arm_state, **head_state, **base_vel}

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        # Capture images
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        left_targets = {k.replace(".pos", ""): v for k, v in action.items() if
                        k.startswith("left_arm_") or k.startswith("head_")}
        right_targets = {k.replace(".pos", ""): v for k, v in action.items() if k.startswith("right_arm_")}

        # Extract base commands
        vx = action.get("x.vel", 0.0)
        vy = action.get("y.vel", 0.0)
        th = action.get("theta.vel", 0.0)

        # Compute corrected wheel velocities
        wheel_cmds = self._body_to_wheel_raw(vx, vy, th)

        # Sync Write
        if left_targets:
            self.bus1.sync_write("Goal_Position", left_targets)
        if right_targets:
            self.bus2.sync_write("Goal_Position", right_targets)
        if wheel_cmds:
            self.bus2.sync_write("Goal_Velocity", wheel_cmds)

        return action

    def stop_base(self):
        self.bus2.sync_write("Goal_Velocity", dict.fromkeys(self.base_motors, 0), num_retry=5)
        logger.info("Base motors stopped")

    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.stop_base()
        self.bus1.disconnect(self.config.disable_torque_on_disconnect)
        self.bus2.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")