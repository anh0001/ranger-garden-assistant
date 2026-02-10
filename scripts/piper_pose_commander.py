#!/usr/bin/env python3
"""
PiPER End-Effector Pose Commander using MoveIt 2

This script demonstrates sending end-effector pose goals to the PiPER arm
using MoveIt 2's native ROS 2 action interface. The complete pipeline:
1. Script sends MoveGroup action goal to /move_action
2. MoveIt plans trajectory to reach the target pose
3. Trajectory is sent as FollowJointTrajectory action
4. piper_follow_joint_trajectory_bridge receives and executes on hardware

Usage:
    # Terminal 1: Launch MoveIt with the arm
    ros2 launch ranger_piper_moveit demo.launch.py

    # Terminal 2: Run this script
    source /opt/ros/humble/setup.bash && source install/setup.bash
    python3 scripts/piper_pose_commander.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
import time


class PiperPoseCommander(Node):
    def __init__(self):
        super().__init__('piper_pose_commander')

        # Create action client for MoveGroup
        self._action_client = ActionClient(self, MoveGroup, '/move_action')

        # Subscribe to joint states
        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        self._current_joint_state = None

        # Wait for action server
        self.get_logger().info("Waiting for /move_action action server...")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveGroup action server (/move_action) not available!")
            raise RuntimeError("MoveGroup action server (/move_action) not available")

        self.get_logger().info("Connected to /move_action action server")
        
        # Wait for joint states
        self.get_logger().info("Waiting for joint states...")
        timeout = 5.0
        start_time = time.time()
        while self._current_joint_state is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self._current_joint_state is None:
            self.get_logger().warn("No joint states received yet")
        else:
            self.get_logger().info("Received joint states")
            self._log_current_state()

    def _joint_state_callback(self, msg):
        """Callback for joint state updates"""
        self._current_joint_state = msg

    def _log_current_state(self):
        """Log current joint values"""
        if self._current_joint_state is None:
            self.get_logger().warn("No joint state available")
            return

        self.get_logger().info("Current joint values:")
        for name, position in zip(self._current_joint_state.name, 
                                    self._current_joint_state.position):
            if 'piper' in name.lower() or 'link' in name.lower():
                self.get_logger().info(f"  {name}: {position:.3f} rad")

    def go_to_joint_goal(self, joint_positions: dict):
        """
        Move to target joint positions

        Args:
            joint_positions: Dictionary mapping joint names to target positions (radians)
        """
        self.get_logger().info(f"Moving to joint goal: {joint_positions}")

        # Create goal message
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "piper_arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3

        # Set joint constraints
        joint_constraint_list = []
        for joint_name, target_pos in joint_positions.items():
            constraint = Constraints()
            # Note: In MoveIt 2, we typically set the target via start_state and goal_constraints
            # For simplicity, this is a basic implementation
            goal_msg.request.goal_constraints.append(constraint)

        # Send goal
        self.get_logger().info("Sending goal to MoveGroup...")
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by MoveGroup")
            return False

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("Motion execution succeeded!")
            return True
        else:
            self.get_logger().error(f"Motion failed with error code: {result.error_code.val}")
            return False

    def go_to_pose_goal(self, pose: Pose, frame_id: str = "base_link"):
        """
        Move end-effector to target pose

        Args:
            pose: Target pose for the end-effector
            frame_id: Reference frame for the pose
        """
        self.get_logger().info("Planning to pose goal:")
        self.get_logger().info(f"  Frame: {frame_id}")
        self.get_logger().info(f"  Position: x={pose.position.x:.3f}, "
                              f"y={pose.position.y:.3f}, "
                              f"z={pose.position.z:.3f}")
        self.get_logger().info(f"  Orientation: x={pose.orientation.x:.3f}, "
                              f"y={pose.orientation.y:.3f}, "
                              f"z={pose.orientation.z:.3f}, "
                              f"w={pose.orientation.w:.3f}")

        # Create goal message
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "piper_arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3

        # Set pose goal as constraints
        constraints = Constraints()
        
        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = frame_id
        pos_constraint.link_name = "piper_link6"  # End-effector link
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        
        # Create a small bounding box around target position
        bounding_volume = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.001]  # 1mm tolerance
        bounding_volume.primitives.append(primitive)
        
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.pose.position = pose.position
        pose_stamped.pose.orientation.w = 1.0
        bounding_volume.primitive_poses.append(pose_stamped.pose)
        
        pos_constraint.constraint_region = bounding_volume
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)
        
        # Orientation constraint
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = frame_id
        orient_constraint.link_name = "piper_link6"
        orient_constraint.orientation = pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.1
        orient_constraint.absolute_y_axis_tolerance = 0.1
        orient_constraint.absolute_z_axis_tolerance = 0.1
        orient_constraint.weight = 1.0
        constraints.orientation_constraints.append(orient_constraint)
        
        goal_msg.request.goal_constraints.append(constraints)

        # Send goal
        self.get_logger().info("Sending goal to MoveGroup...")
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by MoveGroup")
            return False

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("Motion execution succeeded!")
            return True
        else:
            self.get_logger().error(f"Motion failed with error code: {result.error_code.val}")
            return False

    def go_to_position(self, x: float, y: float, z: float, frame_id: str = "base_link"):
        """
        Move end-effector to target position with downward-pointing orientation

        Args:
            x, y, z: Target position coordinates in meters
            frame_id: Reference frame
        """
        # Create target pose with downward-pointing orientation
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        # Downward pointing (nose down 90 degrees)
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 0.707
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 0.707

        return self.go_to_pose_goal(target_pose, frame_id)


def main():
    # Initialize ROS
    rclpy.init()

    # Create pose commander node
    commander = PiperPoseCommander()

    try:
        # Example 1: Move to specific pose
        input("\nPress Enter to move to custom pose (forward reach)...")
        target_pose = Pose()
        target_pose.position.x = 0.3  # 30cm forward
        target_pose.position.y = 0.0  # centered
        target_pose.position.z = 0.4  # 40cm high
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 0.707
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 0.707
        commander.go_to_pose_goal(target_pose)

        # Example 2: Move to position (downward orientation)
        input("\nPress Enter to move to position (0.25, 0.1, 0.35)...")
        commander.go_to_position(0.25, 0.1, 0.35)

        # Example 3: Move to another position
        input("\nPress Enter to move to position (0.30, -0.1, 0.40)...")
        commander.go_to_position(0.30, -0.1, 0.40)

        commander.get_logger().info("\nDemo completed successfully!")

    except KeyboardInterrupt:
        commander.get_logger().info("Interrupted by user")
    except Exception as e:
        commander.get_logger().error(f"Error during execution: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        commander.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
