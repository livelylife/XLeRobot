import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import time

# 导入你的机器人控制库
# 路径根据你的实际情况，如果是 pi 用户可能是 /home/pi/workspace/...
sys.path.insert(0, "/home/wisx/workspace/lerobot/src")
try:
    from lerobot.robots.xlerobot.xlerobot import XLerobot
    from lerobot.robots.xlerobot.config_xlerobot import XLerobotConfig
except ImportError:
    # 备选路径
    sys.path.insert(0, "/home/joyandai/workspace/lerobot/src")
    from lerobot.robots.xlerobot.xlerobot import XLerobot
    from lerobot.robots.xlerobot.config_xlerobot import XLerobotConfig


class XLerobotDriver(Node):
    def __init__(self):
        super().__init__('xlerobot_driver_node')

        # --- 配置 ---
        self.CMD_TIMEOUT = 0.5  # 超时时间：0.5秒没收到指令就停车

        # --- 初始化机器人硬件 ---
        robot_config = XLerobotConfig(
            port1='/dev/ttyACM1',  # 请根据实际情况修改
            port2='/dev/ttyACM0',
            enable_base=True,
            enable_left_arm=False,
            enable_right_arm=False,
            enable_head=False,
        )
        try:
            self.robot = XLerobot(robot_config)
            # 禁用校准，快速启动
            self.robot.connect(calibrate=False)
            self.get_logger().info('✅ XLerobot 硬件连接成功!')
        except Exception as e:
            self.get_logger().error(f'❌ 连接失败: {e}')
            self.destroy_node()
            return

        # --- 状态变量 ---
        self.last_cmd_time = self.get_clock().now().nanoseconds
        self.is_moving = False

        # --- 订阅者 ---
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        # --- 看门狗定时器 (10Hz) ---
        self.timer = self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info(f'看门狗已启动 (超时阈值: {self.CMD_TIMEOUT}s)')

    def cmd_vel_callback(self, msg: Twist):
        """
        收到指令：执行动作，并刷新“最后指令时间”
        """
        # 刷新时间戳
        self.last_cmd_time = self.get_clock().now().nanoseconds
        self.is_moving = True

        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        # 简单的单位转换 (rad/s -> deg/s)
        omega_deg = omega * 180.0 / 3.1415926

        action = {
            "x.vel": vx,
            "y.vel": vy,
            "theta.vel": omega_deg
        }

        self.robot.send_action(action)
        # self.get_logger().info(f"执行: v={vx:.2f}, w={omega_deg:.2f}")

    def watchdog_callback(self):
        """
        定时检查：如果太久没收到指令，就强行停车
        """
        if not self.is_moving:
            return

        # 计算距离上次收到指令过了多久 (纳秒转秒)
        now = self.get_clock().now().nanoseconds
        elapsed = (now - self.last_cmd_time) / 1e9

        if elapsed > self.CMD_TIMEOUT:
            self.get_logger().warn(f'⚠️ 信号丢失 ({elapsed:.1f}s > {self.CMD_TIMEOUT}s)! 执行紧急停车。')
            self.stop_robot()

    def stop_robot(self):
        """发送零速度指令"""
        action = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
        self.robot.send_action(action)
        self.is_moving = False

    def on_shutdown(self):
        self.get_logger().info('节点关闭，停车...')
        if hasattr(self, 'robot'):
            self.stop_robot()
            self.robot.disconnect()


def main(args=None):
    rclpy.init(args=args)
    driver_node = XLerobotDriver()

    try:
        rclpy.spin(driver_node)
    except KeyboardInterrupt:
        pass
    finally:
        driver_node.on_shutdown()
        driver_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()