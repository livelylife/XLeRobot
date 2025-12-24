import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

# 导入你的机器人控制库
sys.path.insert(0, "/home/joyandai/workspace/lerobot/src")
from lerobot.robots.xlerobot.xlerobot import XLerobot
from lerobot.robots.xlerobot.config_xlerobot import XLerobotConfig

class XLerobotDriver(Node):
    def __init__(self):
        super().__init__('xlerobot_driver_node')

        # --- 初始化机器人硬件 ---
        robot_config = XLerobotConfig(
            port1='/dev/ttyACM0',
            port2='/dev/ttyACM1',
            enable_base=True, # 只启用底盘
            # 其他部分可以禁用以节省资源
            enable_left_arm=False,
            enable_right_arm=False,
            enable_head=False,
        )
        try:
            self.robot = XLerobot(robot_config)
            self.robot.connect()
            self.get_logger().info('XLerobot hardware connected successfully!')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to XLerobot hardware: {e}')
            # 如果硬件连接失败，关闭节点
            self.destroy_node()
            return

        # --- 创建 Twist 消息订阅者 ---
        # 订阅 /cmd_vel 话题，收到消息后调用 cmd_vel_callback
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        self.get_logger().info('XLerobot ROS Driver is ready to receive commands on /cmd_vel')

    def cmd_vel_callback(self, msg: Twist):
        """
        收到 /cmd_vel 消息后，将其转换为机器人能懂的硬件指令
        """
        # 从 Twist 消息中提取线速度和角速度
        vx = msg.linear.x  # 前后速度
        vy = msg.linear.y  # 左右速度 (全向轮)
        omega = msg.angular.z # 旋转速度 (单位: rad/s)

        # 将 rad/s 转换为 degree/s，如果你的库需要的话
        # 假设你的库需要的是 degree/s
        omega_deg = omega * 180.0 / 3.1415926

        # 构建 action 字典
        action = {
            "x.vel": vx,
            "y.vel": vy,
            "theta.vel": omega_deg # 确保单位正确！
        }

        # 发送指令给机器人硬件
        self.robot.send_action(action)

        self.get_logger().info(f"Executing command: vx={vx:.2f}, vy={vy:.2f}, omega={omega_deg:.2f} deg/s")

    def on_shutdown(self):
        # 节点关闭时，确保机器人停止
        self.get_logger().info('Shutting down. Stopping the robot...')
        action = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
        self.robot.send_action(action)
        self.robot.disconnect()

def main(args=None):
    rclpy.init(args=args)
    driver_node = XLerobotDriver()

    # 保持节点运行
    try:
        rclpy.spin(driver_node)
    except KeyboardInterrupt:
        pass
    finally:
        # 确保节点关闭时调用 on_shutdown
        driver_node.on_shutdown()
        driver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()