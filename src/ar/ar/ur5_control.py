import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import math
from unity_robotics_demo_msgs.msg import PosRot
import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from ament_index_python.packages import get_package_share_directory
class UR5Controller(Node):
    def __init__(self):
        super().__init__('ur5_controller')
        self.control_publisher = self.create_publisher(JointTrajectory,
                                                       '/joint_trajectory_controller/joint_trajectory',
                                                       10)
        #self.timer = self.create_timer(0.5, self.move_once)
        self.start_time = time.time()
        self.create_subscription(PosRot, '/pos_rot', self.callback, 10)

        pkg_dir = get_package_share_directory('ur_description')
        urdf_path = pkg_dir + "/urdf/ur5.urdf"

        self.ur5 = RobotWrapper.BuildFromURDF(urdf_path, [pkg_dir])

        self.q = np.array([0, -1.57, 0, -1.57, 0, 0])




    def move_once(self):
        msg = JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        current_time = time.time()
        point = JointTrajectoryPoint()
        angle = 0.1*math.atan2(math.sin(current_time), math.cos(current_time))

        
        point.positions = [angle, -1.57, 1.57, 0.0, 0.0, 0.0]
        self.get_logger().info(f"angle: {[angle, -1.57, 1.57, 0.0, 0.0, 0.0]}")
        point.time_from_start.sec = 1
        msg.points.append(point)
        self.control_publisher.publish(msg)
        #self.get_logger().info('Control command sent!')
    
    def callback(self, msg):
        self.get_logger().info(
        f"Received PosRot: x={msg.pos_x:.3f}, y={msg.pos_y:.3f}, z={msg.pos_z:.3f}, "
        f"qx={msg.rot_x:.3f}, qy={msg.rot_y:.3f}, qz={msg.rot_z:.3f}, qw={msg.rot_w:.3f}"
    )

        oMdes = pin.SE3(pin.Quaternion(msg.rot_w, msg.rot_x, msg.rot_y, msg.rot_z).toRotationMatrix(),
                        np.array([msg.pos_x, msg.pos_y, msg.pos_z]))
        
        #self.get_logger().info(f"SE3:{oMdes}")
        eps = 1e-4
        IT_MAX = 100
        alpha = 0.05          
        damp = 1e-6

        frame_id = self.ur5.model.getFrameId("wrist_3_link")

        for i in range(IT_MAX):

            pin.forwardKinematics(self.ur5.model, self.ur5.data, self.q)
            pin.updateFramePlacements(self.ur5.model, self.ur5.data)

            oM = self.ur5.data.oMf[frame_id]

            err_pos = np.array([msg.pos_x, msg.pos_y, msg.pos_z]) - oM.translation

            # rotation error (world frame)
            R_err = pin.Quaternion(msg.rot_w, msg.rot_x, msg.rot_y, msg.rot_z).toRotationMatrix() @ oM.rotation.T
            err_rot = pin.log3(R_err)

            # 6x1 twist error
            err = np.hstack([err_pos, err_rot])

            self.get_logger().info(f"err:{err}")

            if np.linalg.norm(err) < eps:
                break

            J = pin.computeFrameJacobian(
                self.ur5.model,
                self.ur5.data,
                self.q,
                frame_id,
                pin.ReferenceFrame.WORLD
            )

            dq = J.T @ np.linalg.inv(J @ J.T + damp * np.eye(6)) @ err

            # control step
            self.q += alpha * dq  


        msg = JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        point = JointTrajectoryPoint()
        point.positions = [self.q[0], self.q[1], self.q[2], self.q[3], self.q[4], self.q[5]]
        self.get_logger().info(f"angle: {[self.q[0], self.q[1], self.q[2], self.q[3], self.q[4], self.q[5]]}")
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200000000  # 0.2 sec
        msg.points.append(point)
        self.control_publisher.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = UR5Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()