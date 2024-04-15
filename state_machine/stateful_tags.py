import rospy
import math
import sys
from geometry_msgs.msg import PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from moveit_commander import (
    MoveGroupCommander,
    MoveItCommanderException,
    roscpp_initialize,
    roscpp_shutdown,
)


class RobotNode(object):
    SEARCHING = 0
    MOVING_BASE = 1
    MOVING_ARM = 2
    RESETTING_ARM = 3

    def __init__(self):
        # Initialize ROS, subscribers, and publishers
        self.state = self.SEARCHING
        self.tag_detected = False
        self.commandQueue = []

    def tag_detections_callback(self, tag_detections):
        if not tag_detections.detections:
            self.state = self.SEARCHING
            self.tag_detected = False
            return
        self.tag_detected = True
        detection = tag_detections.detections[0]
        tag_id = detection.id[0]
        tag_pose = detection.pose.pose.pose

        # Calculate the distance to the tag
        distance = self.calculate_distance(tag_pose)

        if distance > 1.0:
            if self.state != self.MOVING_BASE:
                print(f"Moving base towards tag {tag_id}")
                self.state = self.MOVING_BASE
                self.execute_moving_base_state(tag_pose)
        else:
            if self.state != self.MOVING_ARM:
                print(f"Moving arm towards tag {tag_id}")
                self.state = self.MOVING_ARM
                self.execute_moving_arm_state(tag_pose)

    def calculate_distance(self, tag_pose):
        try:
            # Ensure the transform is available
            trans = self.buffer.lookup_transform(
                "base_link", tag_pose.header.frame_id, rospy.Time(0)
            )

            # Transform the tag pose from its frame to the base_link frame
            pose_transformed = tf2_geometry_msgs.do_transform_pose(tag_pose, trans)

            # Extract the transformed position
            transformed_position = pose_transformed.pose.position

            # Calculate the Euclidean distance
            distance = math.sqrt(
                transformed_position.x**2
                + transformed_position.y**2
                + transformed_position.z**2
            )

            return distance
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logerr("Error transforming tag pose: %s" % str(e))
            return None

    def execute_resetting_arm_state(self):
        try:
            # For simplicity, we're using a named target assumed to be predefined in the robot's SRDF
            # If using a zero state or specific joint values, you would set them differently
            self.commander.set_named_target("stowed")

            # Command the arm to move to the default pose
            self.commander.go()
            rospy.loginfo("Arm resetting to default pose.")

            # After resetting, you might want to transition to another state, such as SEARCHING
            self.state = self.SEARCHING
        except MoveItCommanderException as e:
            rospy.logerr("Error in resetting arm to default pose: %s" % str(e))

    def execute_searching_state(self):
        rospy.loginfo("Searching for tags: rotating base")
        try:
            # Create a goal pose for rotation
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "base_link"
            goal_pose.header.stamp = rospy.Time.now()

            # Keep the position same as the current position but change the orientation to rotate the base
            # Here, we're assuming a simple rotation around the z-axis (yaw)
            # Adjust the angle as needed for the desired rotation speed and direction
            # Example: rotating 45 degrees (0.785 radians) to the right
            yaw_rotation = 0.785  # Adjust this value as needed
            q = quaternion_from_euler(0, 0, yaw_rotation)

            goal_pose.pose.orientation.x = q[0]
            goal_pose.pose.orientation.y = q[1]
            goal_pose.pose.orientation.z = q[2]
            goal_pose.pose.orientation.w = q[3]

            # Append the rotate command to the command queue
            self.commandQueue.append(("base", goal_pose))
            rospy.loginfo("Added rotate base command to the queue.")
        except Exception as e:
            rospy.logerr(
                f"Error in executing searching state (rotating base): {str(e)}"
            )

    def execute_moving_base_state(self):
        if not self.tag_detected or self.current_tag_pose is None:
            rospy.loginfo("No tag detected or tag pose not available.")
            return

        try:
            # Convert the tag pose to the robot's base frame
            trans = self.buffer.lookup_transform(
                "base_link", self.current_tag_pose.header.frame_id, rospy.Time(0)
            )
            pose_transformed = tf2_geometry_msgs.do_transform_pose(
                self.current_tag_pose, trans
            )

            # Adjust the pose as needed before sending it as a goal
            # For example, you might want to adjust the orientation or the position slightly
            # to ensure the robot approaches the tag correctly
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "base_link"
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.pose.position = pose_transformed.pose.position

            # Adjust orientation (example: facing towards the tag directly)
            # This is a placeholder for whatever orientation adjustment you need
            quaternion = (
                pose_transformed.pose.orientation.x,
                pose_transformed.pose.orientation.y,
                pose_transformed.pose.orientation.z,
                pose_transformed.pose.orientation.w,
            )
            _, _, yaw = euler_from_quaternion(quaternion)
            q = quaternion_from_euler(0, 0, yaw)  # Adjust as necessary
            goal_pose.pose.orientation.x = q[0]
            goal_pose.pose.orientation.y = q[1]
            goal_pose.pose.orientation.z = q[2]
            goal_pose.pose.orientation.w = q[3]

            # Publish the goal pose to the move_base
            # self.move_base_pub.publish(goal_pose)
            self.commandQueue.append(("base", goal_pose))
            rospy.loginfo(f"Published move_base goal for tag.")
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logerr("Error transforming tag pose to base_link frame: %s" % str(e))

    def execute_moving_arm_state(self):
        if not self.tag_detected or self.current_tag_pose is None:
            rospy.loginfo("No tag detected or tag pose not available for moving arm.")
            return

        try:
            # Convert the tag pose to the robot's base frame (assuming the arm's planning frame is 'base_link')
            trans = self.buffer.lookup_transform(
                "base_link", self.current_tag_pose.header.frame_id, rospy.Time(0)
            )
            pose_transformed = tf2_geometry_msgs.do_transform_pose(
                self.current_tag_pose, trans
            )

            # Adjust the pose as needed before commanding the arm
            # For example, adjust the orientation or the position slightly
            # to ensure the arm approaches the tag correctly
            # Here, we make the hand point in a nicer way (no roll or pitch) and adjust yaw
            yaw = math.atan2(
                pose_transformed.pose.position.y, pose_transformed.pose.position.x
            )
            q = quaternion_from_euler(0, 0, yaw)
            pose_transformed.pose.orientation.x = q[0]
            pose_transformed.pose.orientation.y = q[1]
            pose_transformed.pose.orientation.z = q[2]
            pose_transformed.pose.orientation.w = q[3]

            # Set the target pose for the arm
            self.commandQueue.append(("arm", pose_transformed))
            rospy.loginfo("Arm moving to target pose.")
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            MoveItCommanderException,
        ) as e:
            rospy.logerr("Error in moving arm towards tag: %s" % str(e))

    def process_command_queue(self):
        if len(self.commandQueue) > 0:
            command_type, target = self.commandQueue.pop(0)
            if command_type == "base":
                self.move_base_pub.publish(target)
            elif command_type == "arm":
                self.commander.set_pose_target(target)
                self.commander.go()
            elif command_type == "stow":
                self.execute_resetting_arm_state()
            # After processing, check for state transitions
            self.check_and_update_state(command_type)

    def check_and_update_state(self, last_command_type):
        # Check if the last command was an arm command and not the stowing command
        if last_command_type == "arm" and self.state != self.RESETTING_ARM:
            # Look at the next two states in the command queue
            if len(self.commandQueue) >= 2:
                next_command_type, _ = self.commandQueue[0]
                next_next_command_type, _ = self.commandQueue[1]

                # If the next two states are not arm commands, insert the stowing command
                if next_command_type != "arm" and next_next_command_type != "arm":
                    self.commandQueue.insert(0, ("arm", "stow"))
                    self.state = self.RESETTING_ARM
                    rospy.loginfo(
                        "Inserted stowing command before transitioning to next state."
                    )
            # If there are less than 2 commands in the queue, just insert the stowing command
            else:
                self.commandQueue.insert(0, ("arm", "stow"))
                self.state = self.RESETTING_ARM
                rospy.loginfo(
                    "Inserted stowing command before transitioning to next state."
                )

    def start(self):
        rate = rospy.Rate(1)  # Adjust the rate as needed
        while not rospy.is_shutdown():
            self.process_command_queue()
            # State transitions are now handled after command processing
            rate.sleep()


if __name__ == "__main__":
    roscpp_initialize(sys.argv)
    rospy.init_node("robot_node")
    node = RobotNode()
    node.start()
    roscpp_shutdown()
