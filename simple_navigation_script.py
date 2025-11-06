#!/usr/bin/env python3
# This is the simple navigation script already on Mirte but we modified it 
import os
import yaml
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory

def load_pose_from_yaml(label: str):
    pkg_share = get_package_share_directory('mirte_location_markers')
    poses_path = os.path.join(pkg_share, 'locations', 'stored_poses.yaml')
    with open(poses_path, 'r') as f:
        data = yaml.safe_load(f) or {}
    if label not in data:
        raise KeyError(f"Label '{label}' not found in {poses_path}")
    return data[label]

class SimpleNav(Node):
    def __init__(self):
        super().__init__('simple_navigation_script')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/mirte_master_arm_controller/follow_joint_trajectory')
        self.arm_client.wait_for_server()
        self.cmd_vel_pub = self.create_publisher(Twist, '/mirte_base_controller/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(String, '/door_cmd', 10)

    def send_goal_and_wait(self, label: str):
        pose_dict = load_pose_from_yaml(str(label))
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        pos = pose_dict['position']
        ori = pose_dict['orientation']
        goal.pose.pose.position.x = float(pos['x'])
        goal.pose.pose.position.y = float(pos['y'])
        goal.pose.pose.position.z = float(pos.get('z', 0.0))
        goal.pose.pose.orientation.x = float(ori.get('x', 0.0))
        goal.pose.pose.orientation.y = float(ori.get('y', 0.0))
        goal.pose.pose.orientation.z = float(ori['z'])
        goal.pose.pose.orientation.w = float(ori['w'])

        self.get_logger().info(f"Sending goal to label '{label}'...")
        self.client.wait_for_server()
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2.")
            return False

        result_future = goal_handle.get_result_async()
        self.get_logger().info("Waiting for Nav2 to finish...")
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        from action_msgs.msg import GoalStatus
        if result and result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Goal '{label}' SUCCEEDED.")
            return True
        else:
            self.get_logger().warn(f"Goal '{label}' failed (status={getattr(result, 'status', 'unknown')}).")
            return False

    def send_arm_trajectory(self, joint_positions, duration_sec=3.0):
        traj = JointTrajectory()
        traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_joint']

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        traj.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        send_goal_future = self.arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Arm goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info("Arm movement completed")
        return True
    # to drive forward or backward witthout needing coordinates, just like teleoperation keys
    def drive(self, forward=True, duration=2.0):
        """Drive like teleop key 'i' (forward) or ',' (backward) for a short time."""
        twist = Twist()
        twist.linear.x = 0.2 if forward else -0.2
        twist.angular.z = 0.0
        self.get_logger().info(f"Driving {'forward' if forward else 'backward'} for {duration}s")

        start = time.time()
        while time.time() - start < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)

        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Stopped movement.")
    # to oper or close the gripper via the /door_cmd topic 
    def control_gripper(self, open=True):
        msg = String()
        msg.data = "o" if open else "c"
        self.gripper_pub.publish(msg)
        self.get_logger().info(f"{'Opening' if open else 'Closing'} gripper")
        time.sleep(5)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info("Published stop command.")

def main():
    rclpy.init()
    node = SimpleNav()

    try:
        # 1. Lower gripper
        node.send_arm_trajectory([0.0, -1.45, -1.45, 1.45])
        
        # 2. Drive forward 
        node.drive(forward=True, duration=2.0)

        # 3. Open gripper
        node.control_gripper(open=False)

        # 4. Lift gripper (just slightly to avoid smashing the arm )
        node.send_arm_trajectory([0.0, -1.45, -0.8, 1.0])

        # 5. Drive backward 
        node.drive(forward=False, duration=4.0)

        # 6. Lift the arm in the neutral driving position
        node.send_arm_trajectory([0.0, 0.0, -1.55, 0.0])

        # 7. Go to stored_point (specify where you want to go in stored_poses.yaml)
        node.send_goal_and_wait("2")

        # 8. Lower gripper
        node.send_arm_trajectory([0.0, -0.71, -2.0, 1.2])

        # 9. Close gripper
        node.control_gripper(open=True)

        # 10. Drive backward (out of the cube)
        node.drive(forward=False, duration=3.0)

        # 11. Go back to "start" ans raise the arm 
        node.send_arm_trajectory([0.0, 0.0, 0.0, 0.0])
        node.send_goal_and_wait("start")

        node.stop_robot()
        time.sleep(0.5)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()