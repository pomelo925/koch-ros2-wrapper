import rclpy
from rclpy.node import Node
from lerobot.common.robot_devices.motors.configs import DynamixelMotorsBusConfig
from lerobot.common.robot_devices.motors.dynamixel import DynamixelMotorsBus
from lerobot.common.robot_devices.robots.configs import KochRobotConfig
from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot
import yaml
from sensor_msgs.msg import JointState
import numpy as np

CALIBRATION_FOLDER_PATH = '/ros2-ws/src/koch_wrapper/calibration'
CONFIG_PATH = '/ros2-ws/src/koch_wrapper/config/leader_follower.yaml'
PUBLISH_FPS = 50
SYNC_FPS = 100

class KochRobotNode(Node):
    def __init__(self):
        super().__init__('koch_robot_node')
        self.get_logger().info("Initializing Koch Robot Node...")
        self.config = self.load_config(CONFIG_PATH)
        self.robot = self.arm_initialize(self.config)

        # 建立四個 JointState Publisher
        self.left_leader_pub = self.create_publisher(JointState, '/left_leader/JointState', 10)
        self.right_leader_pub = self.create_publisher(JointState, '/right_leader/JointState', 10)
        self.left_follower_pub = self.create_publisher(JointState, '/left_follower/JointState', 10)
        self.right_follower_pub = self.create_publisher(JointState, '/right_follower/JointState', 10)

        # 設置定時器以指定的頻率同步位置
        self.timer = self.create_timer(1.0 / PUBLISH_FPS, self.publish_all_joint_states)
        self.timer = self.create_timer(1.0 / SYNC_FPS, self.sync_positions)

    def load_config(self, file_path):
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)

    def arm_initialize(self, config):
        leader_configs = [DynamixelMotorsBusConfig(port=arm['port'], motors=arm['motors']) for arm in config['leader_arms']]
        follower_configs = [DynamixelMotorsBusConfig(port=arm['port'], motors=arm['motors']) for arm in config['follower_arms']]

        leader_arms = [DynamixelMotorsBus(config) for config in leader_configs]
        follower_arms = [DynamixelMotorsBus(config) for config in follower_configs]

        for arm in leader_arms + follower_arms:
            arm.connect()

        robot_config = KochRobotConfig(
            leader_arms={"left": leader_configs[0], "right": leader_configs[1]},
            follower_arms={"left": follower_configs[0], "right": follower_configs[1]},
            cameras={},
        )
        robot = ManipulatorRobot(robot_config)
        robot.connect()
        return robot

    def sync_positions(self):
        # 讀取 leader 端的關節位置
        left_leader_pos_deg = self.robot.leader_arms["left"].read("Present_Position")
        right_leader_pos_deg = self.robot.leader_arms["right"].read("Present_Position")

        # 將 leader 端的位置同步到 follower
        self.robot.follower_arms["left"].write("Goal_Position", left_leader_pos_deg)
        self.robot.follower_arms["right"].write("Goal_Position", right_leader_pos_deg)
    

    def publish_all_joint_states(self):
        # 讀取 leader 端的關節位置
        left_leader_pos_deg = self.robot.leader_arms["left"].read("Present_Position")
        right_leader_pos_deg = self.robot.leader_arms["right"].read("Present_Position")

        # 讀取 follower 端的關節位置
        left_follower_pos_deg = self.robot.follower_arms["left"].read("Present_Position")
        right_follower_pos_deg = self.robot.follower_arms["right"].read("Present_Position")
        
        # 將角度轉換為弧度
        left_leader_pos_rad = np.radians(left_leader_pos_deg)
        right_leader_pos_rad = np.radians(right_leader_pos_deg)
        left_follower_pos_rad = np.radians(left_follower_pos_deg)
        right_follower_pos_rad = np.radians(right_follower_pos_deg)

        # 發布四個 JointState
        self.publish_joint_state(self.left_leader_pub, "left_leader_joint", left_leader_pos_rad)
        self.publish_joint_state(self.right_leader_pub, "right_leader_joint", right_leader_pos_rad)
        self.publish_joint_state(self.left_follower_pub, "left_follower_joint", left_follower_pos_rad)
        self.publish_joint_state(self.right_follower_pub, "right_follower_joint", right_follower_pos_rad)


    def publish_joint_state(self, publisher, joint_name, position):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [joint_name]
        msg.position = position.tolist()
        msg.velocity = []
        msg.effort = []
        publisher.publish(msg)

    def shutdown(self):
        self.get_logger().info("Shutting down Koch Robot Node...")
        self.robot.disconnect()


def main():
    rclpy.init()
    node = KochRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()