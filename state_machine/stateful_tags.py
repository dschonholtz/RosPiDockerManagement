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
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class RobotNode(object):
    SEARCHING = 0
    MOVING_BASE = 1
    MOVING_ARM = 2
    RESETTING_ARM = 3


    def __init__(self):
        # Initialize ROS, subscribers, and publishers
        self.state = self.SEARCHING
        self.buffer = tf2_ros.Buffer(cache_time=rospy.Duration(6))
        self.tflistener = tf2_ros.TransformListener(self.buffer)
        self.current_detection = None
        self.last_tag_processed = rospy.Time().now().to_sec()

        rospy.Subscriber(
            "/tag_detections", AprilTagDetectionArray, self.tag_detections_callback
        )
        self.commander = MoveGroupCommander("arm")
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server(rospy.Duration(5))

    def tag_detections_callback(self, tag_detections):
        if tag_detections.detections:
            self.current_detection = tag_detections.detections[0]
            self.last_tag_processed = rospy.Time().now().to_sec()
            if self.state == RobotNode.SEARCHING:
                self.move_base_client.cancel_all_goals()


    def get_pose_msg_from_detection(self, detection):
        camera_frame = detection.pose.header.frame_id
        tag_pose = detection.pose.pose.pose
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = camera_frame
        pose_msg.pose.position.x = tag_pose.position.x
        pose_msg.pose.position.y = tag_pose.position.y
        pose_msg.pose.position.z = tag_pose.position.z

        pose_msg.pose.orientation.x = tag_pose.orientation.x
        pose_msg.pose.orientation.y = tag_pose.orientation.y
        pose_msg.pose.orientation.z = tag_pose.orientation.z
        pose_msg.pose.orientation.w = tag_pose.orientation.w
        return pose_msg

    def calculate_distance(self, detection):
        try:
            # Ensure the transform is available
            camera_frame = detection.pose.header.frame_id
            trans = self.buffer.lookup_transform(
                "capstone/base_link", camera_frame, rospy.Time(0)
            )

            pose_msg = self.get_pose_msg_from_detection(
                detection=self.current_detection
            )
            pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_msg, trans)

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
            self.commander.go(wait=True)
            rospy.loginfo("Arm resetting to default pose.")
        except MoveItCommanderException as e:
            rospy.logerr("Error in resetting arm to default pose: %s" % str(e))

    def get_current_base_orientation(self):
        try:
            # Get the current transform from the base_link to the map frame
            trans = self.buffer.lookup_transform(
                "map", "capstone/base_link", rospy.Time(0)
            )

            # Extract the orientation from the transform
            orientation = trans.transform.rotation

            # Convert the quaternion orientation to Euler angles
            (roll, pitch, yaw) = euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w]
            )

            return (roll, pitch, yaw)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rospy.logerr("Failed to get current base orientation")
            return (0, 0, 0)  # Return default orientation if transformation fails

    def execute_searching_state(self):
        rospy.loginfo("Searching for tags: rotating base")
        try:
            # Create a goal pose for rotation
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "capstone/base_link"

            goal_pose.header.stamp = rospy.Time(0)

            # # Get the current orientation of the base
            current_orientation = self.get_current_base_orientation()

            # # Calculate the new orientation by adding 45 degrees (0.785 radians) to the current yaw
            yaw_rotation = 0.785  # 45 degrees in radians
            new_yaw = current_orientation[2] + yaw_rotation
            print(f"current yaw: {current_orientation[2]}")
            print(f"new yaw: {new_yaw}")
            q = quaternion_from_euler(0, 0, new_yaw)


            goal_pose = self.buffer.transform(goal_pose, "map")
            goal_pose.pose.orientation.x = q[0]
            goal_pose.pose.orientation.y = q[1]
            goal_pose.pose.orientation.z = q[2]
            goal_pose.pose.orientation.w = q[3]

            self.move_base_client.cancel_all_goals()
            goal = MoveBaseGoal()
            goal.target_pose = goal_pose
            # Set up your goal here
            print('about to send searching goal')
            self.move_base_client.send_goal(goal)
            print('goal sent waitin')
            self.move_base_client.wait_for_result()
            print('getting results')
        except Exception as e:
            rospy.logerr(
                f"Error in executing searching state (rotating base): {str(e)}"
            )

    def execute_moving_base_state(self):
        if not self.tag_detected or self.current_detection is None:
            rospy.loginfo("No tag detected or tag pose not available.")
            return

        try:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "tag_" + str(self.current_detection.id[0])
            pose_msg.pose.position.x = 0
            pose_msg.pose.position.y = 0
            pose_msg.pose.position.z = 1
            q = quaternion_from_euler(0, -1.5707, 1.5707)
            pose_msg.pose.orientation.x = q[0]
            pose_msg.pose.orientation.y = q[1]
            pose_msg.pose.orientation.z = q[2]
            pose_msg.pose.orientation.w = q[3]
            # Convert the tag pose to the robot's base frame
            trans = self.buffer.lookup_transform(
                "capstone/base_link",
                "tag_" + str(self.current_detection.id[0]),
                rospy.Time(),
            )

            # Create a transform message
            transform = tf2_ros.TransformStamped()

            transform.transform.translation.x = trans.transform.translation.x
            transform.transform.translation.y = trans.transform.translation.y
            transform.transform.translation.z = trans.transform.translation.z
            transform.transform.rotation.x = trans.transform.rotation.x
            transform.transform.rotation.y = trans.transform.rotation.y
            transform.transform.rotation.z = trans.transform.rotation.z
            transform.transform.rotation.w = trans.transform.rotation.w

            # Transform the tag pose from the camera frame to the map frame
            pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_msg, transform)
            pose_transformed.header.frame_id = "capstone/base_link"

            # make the hand point in a nicer way (no roll or pitch)
            quaternion = (
                pose_transformed.pose.orientation.x,
                pose_transformed.pose.orientation.y,
                pose_transformed.pose.orientation.z,
                pose_transformed.pose.orientation.w,
            )
            roll, pitch, yaw = euler_from_quaternion(quaternion)

            q = quaternion_from_euler(0, 0, yaw + 3.14159)
            pose_transformed.pose.orientation.x = q[0]
            pose_transformed.pose.orientation.y = q[1]
            pose_transformed.pose.orientation.z = q[2]
            pose_transformed.pose.orientation.w = q[3]

            pose_transformed.pose.position.z = 0

            # Publish the goal pose to the move_base
            self.move_base_client.cancel_all_goals()
            goal = MoveBaseGoal()
            goal.target_pose = pose_transformed
            self.move_base_client.send_goal(goal)
            rospy.loginfo("Sending goal to move_base")

            # Optionally, wait for the server to finish performing the action.
            self.move_base_client.wait_for_result()

            # Check the result (optional)
            if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
            else:
                rospy.loginfo("Goal failed!")
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logerr("Error transforming tag pose to base_link frame: %s" % str(e))

    def execute_moving_arm_state(self):
        if not self.tag_detected or self.current_detection is None:
            rospy.loginfo("No tag detected or tag pose not available for moving arm.")
            return

        try:
            tag_id = self.current_detection.id
            print("trying to poke tag ")
            print(tag_id)
            print("\n")
            tag_pose = self.current_detection.pose.pose.pose

            # Convert the apriltag detection to a PoseStamped message in the camera_link frame
            camera_frame = self.current_detection.pose.header.frame_id
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = camera_frame
            pose_msg.pose.position.x = tag_pose.position.x
            pose_msg.pose.position.y = tag_pose.position.y
            pose_msg.pose.position.z = tag_pose.position.z

            pose_msg.pose.orientation.x = tag_pose.orientation.x
            pose_msg.pose.orientation.y = tag_pose.orientation.y
            pose_msg.pose.orientation.z = tag_pose.orientation.z
            pose_msg.pose.orientation.w = tag_pose.orientation.w

            transform = self.buffer.lookup_transform(
                "capstone/base_link", camera_frame, self.current_detection.pose.header.stamp
            )

            # Transform the tag pose from the camera frame to the map frame
            pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_msg, transform)
            pose_transformed.header.frame_id = "capstone/base_link"

            # make the hand point in a nicer way (no roll or pitch)
            yaw = math.atan2(
                pose_transformed.pose.position.y, pose_transformed.pose.position.x
            )
            q = quaternion_from_euler(0, 0, yaw)
            pose_transformed.pose.orientation.x = q[0]
            pose_transformed.pose.orientation.y = q[1]
            pose_transformed.pose.orientation.z = q[2]
            pose_transformed.pose.orientation.w = q[3]

            # pose_transformed = self.tflistener.transform(pose_msg, "map")

            self.commander.set_pose_target(pose_transformed)
            self.commander.go()

            rospy.loginfo("Arm moving to target pose.")
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            MoveItCommanderException,
        ) as e:
            rospy.logerr("Error in moving arm towards tag: %s" % str(e))

    def handle_state_machine(self):
        # if rospy.Time().now().to_sec() - self.last_seconds_processed > 5:
        # self.last_seconds_processed = rospy.Time().now().to_sec()
        if not self.current_detection or rospy.Time().now().to_sec() - self.last_tag_processed > 5:
            self.state = self.SEARCHING
            self.tag_detected = False
            self.execute_resetting_arm_state()
            print('searching')
            self.execute_searching_state()
            return
        self.tag_detected = True
        detection = self.current_detection
        tag_id = detection.id[0]
        
        # Calculate the distance to the tag
        distance = self.calculate_distance(detection)
        print(f"distance: {distance}")
        print(f"state: {self.state}")
        if distance > 1.6:
            if self.state != self.MOVING_BASE:
                print(f"Moving base towards tag {tag_id}")
                self.state = self.MOVING_BASE
                self.execute_resetting_arm_state()
                self.execute_moving_base_state()
        else:
            if self.state != self.MOVING_ARM:
                print(f"Moving arm towards tag {tag_id}")
                self.state = self.MOVING_ARM
                self.execute_moving_arm_state()


    def start(self):
        rate = rospy.Rate(0.2)  # Adjust the rate as needed
        while not rospy.is_shutdown():
            # State transitions are now handled after command processing
            self.handle_state_machine()
            rate.sleep()


if __name__ == "__main__":
    roscpp_initialize(sys.argv)
    rospy.init_node("robot_node")
    node = RobotNode()
    node.start()
    roscpp_shutdown()
