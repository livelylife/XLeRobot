"""Odometry helpers for the XLeRobot 3-wheel omni base."""

import math
from dataclasses import dataclass
from typing import Any

import numpy as np


STEPS_PER_DEGREE = 4096.0 / 360.0


def degps_to_raw(degps: float) -> int:
    speed_int = int(round(degps * STEPS_PER_DEGREE))
    return max(min(speed_int, 32767), -32768)


def raw_to_degps(raw_speed: int | float) -> float:
    return float(raw_speed) / STEPS_PER_DEGREE


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def yaw_to_quaternion(yaw_rad: float) -> dict[str, float]:
    half_yaw = yaw_rad * 0.5
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half_yaw),
        "w": math.cos(half_yaw),
    }


def stamp_from_time(timestamp: float) -> dict[str, int]:
    sec = int(timestamp)
    nanosec = int(round((timestamp - sec) * 1_000_000_000))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return {"sec": sec, "nanosec": nanosec}


@dataclass
class Omni3Kinematics:
    wheel_radius: float = 0.05
    base_radius: float = 0.125
    wheel_angles_degrees: tuple[float, float, float] = (150.0, 270.0, 30.0)
    correction_degrees: float = -175.0

    def __post_init__(self) -> None:
        angles = np.radians(np.array(self.wheel_angles_degrees, dtype=float))
        self._matrix = np.array([[np.cos(a), np.sin(a), self.base_radius] for a in angles], dtype=float)
        self._matrix_pinv = np.linalg.pinv(self._matrix)
        self._correction_rad = math.radians(self.correction_degrees)

    def _apply_correction(self, x_vel: float, y_vel: float) -> tuple[float, float]:
        c = math.cos(self._correction_rad)
        s = math.sin(self._correction_rad)
        return x_vel * c - y_vel * s, x_vel * s + y_vel * c

    def _remove_correction(self, x_vel: float, y_vel: float) -> tuple[float, float]:
        c = math.cos(-self._correction_rad)
        s = math.sin(-self._correction_rad)
        return x_vel * c - y_vel * s, x_vel * s + y_vel * c

    def body_to_wheel_degps(self, x_vel: float, y_vel: float, theta_degps: float) -> np.ndarray:
        corrected_x, corrected_y = self._apply_correction(x_vel, y_vel)
        body_velocity = np.array([corrected_x, corrected_y, math.radians(theta_degps)], dtype=float)
        wheel_radps = self._matrix.dot(body_velocity) / self.wheel_radius
        return wheel_radps * (180.0 / math.pi)

    def wheel_raw_to_body(self, raw_1: int | float, raw_2: int | float, raw_3: int | float) -> dict[str, float]:
        wheel_degps = np.array([raw_to_degps(raw_1), raw_to_degps(raw_2), raw_to_degps(raw_3)], dtype=float)
        wheel_radps = np.radians(wheel_degps)
        wheel_linear = wheel_radps * self.wheel_radius
        corrected_x, corrected_y, theta_radps = self._matrix_pinv.dot(wheel_linear)
        x_vel, y_vel = self._remove_correction(float(corrected_x), float(corrected_y))
        return {
            "x.vel": x_vel,
            "y.vel": y_vel,
            "theta.vel": math.degrees(float(theta_radps)),
        }


def apply_velocity_deadband(
    vx: float,
    vy: float,
    omega_degps: float,
    linear_deadband: float = 0.003,
    angular_deadband_degps: float = 1.0,
) -> tuple[float, float, float]:
    if abs(vx) < linear_deadband:
        vx = 0.0
    if abs(vy) < linear_deadband:
        vy = 0.0
    if abs(omega_degps) < angular_deadband_degps:
        omega_degps = 0.0
    return vx, vy, omega_degps


@dataclass
class OdomTracker:
    max_dt: float = 0.5
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    last_time: float | None = None
    last_dt: float = 0.0

    def reset(self, timestamp: float | None = None) -> None:
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time_now() if timestamp is None else timestamp
        self.last_dt = 0.0

    def update(self, vx: float, vy: float, omega_degps: float, timestamp: float | None = None) -> None:
        now = time_now() if timestamp is None else timestamp
        if self.last_time is None:
            self.last_time = now
            self.last_dt = 0.0
            return

        dt = now - self.last_time
        self.last_time = now
        if dt <= 0:
            self.last_dt = 0.0
            return
        dt = min(dt, self.max_dt)
        self.last_dt = dt

        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)
        world_vx = cos_theta * vx - sin_theta * vy
        world_vy = sin_theta * vx + cos_theta * vy

        self.x += world_vx * dt
        self.y += world_vy * dt
        self.theta = normalize_angle(self.theta + math.radians(omega_degps) * dt)

    def as_odometry_dict(
        self,
        timestamp: float,
        vx: float,
        vy: float,
        omega_degps: float,
        frame_id: str = "odom",
        child_frame_id: str = "base_link",
        source: str = "wheel",
    ) -> dict[str, Any]:
        omega_radps = math.radians(omega_degps)
        return {
            "type": "nav_msgs/Odometry",
            "header": {
                "stamp": stamp_from_time(timestamp),
                "frame_id": frame_id,
            },
            "child_frame_id": child_frame_id,
            "pose": {
                "pose": {
                    "position": {"x": self.x, "y": self.y, "z": 0.0},
                    "orientation": yaw_to_quaternion(self.theta),
                }
            },
            "twist": {
                "twist": {
                    "linear": {"x": vx, "y": vy, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": omega_radps},
                }
            },
            "metadata": {
                "theta_deg": math.degrees(self.theta),
                "dt": self.last_dt,
                "source": source,
            },
        }


def time_now() -> float:
    import time

    return time.time()
