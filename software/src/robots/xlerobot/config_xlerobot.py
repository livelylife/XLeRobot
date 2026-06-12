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

from dataclasses import dataclass, field

from lerobot.cameras.configs import CameraConfig, Cv2Rotation, ColorMode
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig

from ..config import RobotConfig


def xlerobot_cameras_config() -> dict[str, CameraConfig]:
    return {
        # 默认这里留空，你可以根据需要取消注释添加摄像头
        # "head": RealSenseCameraConfig(
        #     serial_number_or_name="125322060037",
        #     fps=30,
        #     width=1280,
        #     height=720,
        #     color_mode=ColorMode.BGR,
        #     rotation=Cv2Rotation.NO_ROTATION,
        #     use_depth=True
        # ),
    }


@RobotConfig.register_subclass("xlerobot")
@dataclass
class XLerobotConfig(RobotConfig):
    
    port1: str = "/dev/ttyACM1"  # port to connect to the bus (so101 + head camera)
    port2: str = "/dev/ttyACM0"  # port to connect to the bus (same as lekiwi setup)
    disable_torque_on_disconnect: bool = True
    max_relative_target: int | None = None

    cameras: dict[str, CameraConfig] = field(default_factory=xlerobot_cameras_config)
    use_degrees: bool = False

    # 默认按键映射
    teleop_keys: dict[str, str] = field(
        default_factory=lambda: {
            "forward": "i",
            "backward": "k",
            "left": "j",
            "right": "l",
            "rotate_left": "u",
            "rotate_right": "o",
            "speed_up": "n",
            "speed_down": "m",
            "quit": "b",
        }
    )

    # 3-wheel omni base parameters
    wheel_radius: float = 0.05  # Wheel radius in meters
    base_radius: float = 0.125  # Distance from robot center to each wheel in meters
    base_correction_degrees: float = -175.0  # Empirical command-frame correction used by the base kinematics
    odom_frame_id: str = "odom"
    odom_child_frame_id: str = "base_link"
    odom_linear_deadband: float = 0.003  # m/s; suppresses tiny stationary wheel-velocity noise
    odom_angular_deadband_degps: float = 1.0  # deg/s; suppresses tiny stationary yaw-rate noise

    # === 功能开关 (新增) ===
    # True = 启用, False = 禁用
    enable_left_arm: bool = True
    enable_right_arm: bool = True
    enable_head: bool = True
    enable_base: bool = True


@dataclass
class XLerobotHostConfig:
    # Network Configuration
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556

    # Duration of the application
    connection_time_s: int = 3600

    # Watchdog: stop the robot if no command is received for over 0.5 seconds.
    watchdog_timeout_ms: int = 500

    # If robot jitters decrease the frequency and monitor cpu load with `top` in cmd
    max_loop_freq_hz: int = 30


@RobotConfig.register_subclass("xlerobot_client")
@dataclass
class XLerobotClientConfig(RobotConfig):
    # Network Configuration
    remote_ip: str
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556

    teleop_keys: dict[str, str] = field(
        default_factory=lambda: {
            "forward": "i",
            "backward": "k",
            "left": "j",
            "right": "l",
            "rotate_left": "u",
            "rotate_right": "o",
            "speed_up": "n",
            "speed_down": "m",
            "quit": "b",
        }
    )

    cameras: dict[str, CameraConfig] = field(default_factory=xlerobot_cameras_config)

    polling_timeout_ms: int = 15
    connect_timeout_s: int = 5