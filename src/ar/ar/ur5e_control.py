#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class UR5eTrajectoryCommander(Node):
    def __init__(self):
        super().__init__('ur5e_trajectory_commander')

        # Publisher to trajectory controller
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )


        # Short delay to ensure publisher connects
        self.timer = self.create_timer(1.0, self.send_trajectory_once)
        self.sent = False

    def send_trajectory_once(self):
        if self.sent:
            return

        msg = JointTrajectory()
        msg.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        point = JointTrajectoryPoint()
        # Target positions (radians)
        point.positions = [0.0, -1.57, 1.57, 0.0, -1.57, 0.0]
        point.time_from_start.sec = 3   # move in 3 seconds

        msg.points.append(point)

        self.publisher_.publish(msg)
        self.get_logger().info("Trajectory command sent!")
        self.sent = True


def main(args=None):
    rclpy.init(args=args)
    node = UR5eTrajectoryCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
