import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import math
from unity_robotics_demo_msgs.msg import PosRot, Pos, JointDrag
import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from ament_index_python.packages import get_package_share_directory


class UR5Controller(Node):
    def __init__(self):
        super().__init__('ur5_controller')
        self.mode = 1
        self.control_publisher = self.create_publisher(
            JointTrajectory,
            #'/scaled_joint_trajectory_controller/joint_trajectory',
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        self.target_pos = None
        self.target_quat = None

        self.create_subscription(PosRot, '/pos_rot', self.pos_callback, 10)

        pkg_dir = get_package_share_directory('ur_description')
        urdf_path = pkg_dir + "/urdf/ur5.urdf"
        self.ur5 = RobotWrapper.BuildFromURDF(urdf_path, [pkg_dir])

        self.q = np.array([0.0, -1.57, 0.0, -1.57, 0.0, 0.0])

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("UR5 controller initialized (20Hz control loop)")

        ### path planning 
        self.create_subscription(Pos, '/pos', self.path_callback, 10)

        ### joint_dragging
        self.create_subscription(JointDrag, '/JointDrag', self.Drag_callback, 10)
        self.Drag_Joint = -1
        self.Drag_target = None
        self.frame_IDs = ['shoulder_link',
                          'upper_arm_link',
                          'forearm_link',
                          'wrist_1_link',
                          'wrist_2_link',
                          'wrist_3_link']
        
    def pos_callback(self, msg: PosRot):
        self.mode = 1
        self.target_pos = np.array([msg.pos_x, msg.pos_y, msg.pos_z])
        self.target_quat = pin.Quaternion(msg.rot_w, msg.rot_x, msg.rot_y, msg.rot_z)
        
        self.get_logger().info(
            f"[Callback] Updated target pose: "
            f"pos=({msg.pos_x:.3f}, {msg.pos_y:.3f}, {msg.pos_z:.3f}), "
            f"quat=({msg.rot_x:.3f}, {msg.rot_y:.3f}, {msg.rot_z:.3f}, {msg.rot_w:.3f})"
        )


    def path_callback(self, msg: Pos):
        self.mode = 2
        self.target_pos = np.array([msg.x, msg.y, msg.z])
        pin.forwardKinematics(self.ur5.model, self.ur5.data, self.q)
        pin.updateFramePlacements(self.ur5.model, self.ur5.data)
        
        frame_id = self.ur5.model.getFrameId("wrist_3_link")
        current_rot = self.ur5.data.oMf[frame_id].rotation
        
        self.target_quat = pin.Quaternion(current_rot)
        self.get_logger().info(
            f"[Callback] Updated target pose: "
            f"pos=({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})"
        )
        
    def Drag_callback(self, msg:JointDrag):
        self.mode = 3
        self.Drag_Joint = msg.joint_index
        self.Drag_target = np.array([msg.x, msg.y, msg.z])
        pin.forwardKinematics(self.ur5.model, self.ur5.data, self.q)
        pin.updateFramePlacements(self.ur5.model, self.ur5.data)
        
        frame_id = self.ur5.model.getFrameId("wrist_3_link")
        current_rot = self.ur5.data.oMf[frame_id].rotation
        current_trans = self.ur5.data.oMf[frame_id].translation
        
        self.target_quat = pin.Quaternion(current_rot)
        self.target_pos = current_trans
        
        self.get_logger().info(
            f"[Callback] Updated target pose: "
            f"pos=({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})"
        )       
        
    def control_loop(self):
            
        if self.target_pos is None:
            return

        if self.mode == 3:
            frame_id = self.ur5.model.getFrameId(self.frame_IDs[self.Drag_Joint])
        else:
            frame_id = self.ur5.model.getFrameId("wrist_3_link")

        alpha = 0.05
        damp = 1e-2
        iterations = 5
        pi = math.pi
        joint_min = np.array([-2*pi, -2*pi, -2*pi, -2*pi, -2*pi, -2*pi])
        joint_max = np.array([ 2*pi,  2*pi,  2*pi,  2*pi,  2*pi,  2*pi])


        for _ in range(iterations):

            # Forward kinematics
            pin.forwardKinematics(self.ur5.model, self.ur5.data, self.q)
            pin.updateFramePlacements(self.ur5.model, self.ur5.data)

            oM = self.ur5.data.oMf[frame_id]

            # Position error
            if self.mode == 3:
                err_pos = self.Drag_target - oM.translation
            else:
                err_pos = self.target_pos - oM.translation

            J = pin.computeFrameJacobian(
                self.ur5.model, self.ur5.data, self.q, frame_id, pin.ReferenceFrame.WORLD
            )
            # Rotation error


            if self.mode == 2:
                self.target_quat = pin.Quaternion(1, 0, 0, 0)
            
            if self.mode == 3:
                J = J[0:3, :]
                err = err_pos
                dq = J.T @ np.linalg.inv(J @ J.T + damp * np.eye(3)) @ err

            else:
                R_target = self.target_quat.toRotationMatrix()
                R_err = R_target @ oM.rotation.T
                err_rot = pin.log3(R_err)
                err = np.hstack([err_pos, err_rot])
                dq = J.T @ np.linalg.inv(J @ J.T + damp * np.eye(6)) @ err

            
            if self.mode == 3:
                mask = np.zeros(6)
                mask[:self.Drag_Joint] = 1
                dq = dq * mask
            dq = np.clip(dq, -0.05, 0.05)
            self.q += alpha * dq
            self.q = np.clip(self.q, joint_min, joint_max)


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
        point.positions = self.q.tolist()

        point.time_from_start.sec = 0
        if self.mode == 2 or self.mode == 3:
            point.time_from_start.sec = 5  
        elif self.mode == 1:
            point.time_from_start.nanosec = 300_000_000  


        msg.points.append(point)
        self.control_publisher.publish(msg)

        self.get_logger().info(f"[Control Loop] q = {self.q}")
    


def main(args=None):
    rclpy.init(args=args)
    node = UR5Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
