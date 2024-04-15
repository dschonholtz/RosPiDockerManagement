#!/usr/bin/env python3

"""
Replicates the gazebo_robot_testbed control scheme, but uses MoveIt

"""
import math
import sys
import time

import rospy
from tf.transformations import quaternion_from_euler

from moveit_commander import MoveGroupCommander, MoveItCommanderException, roscpp_initialize, roscpp_shutdown

from geometry_msgs.msg import PoseStamped
from common_robot_codebase.srv import SetString

### april tag imports
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection

# from tf2_ros import TransformListener, TransformStamped, Buffer, tf2_geometry_msgs
import tf2_ros
import tf2_geometry_msgs



class PokeTagNode(object):
    def __init__(self):
        self.robotName = str(rospy.get_param("~robot_name", "robot"))
        self.setpoints = rospy.get_param("~setpoints", {})

        self.startTime = rospy.Time.now().to_sec()
        self.commandQueue = []

        self.commander = MoveGroupCommander("arm")


        ## tag poke stuff
        self.buffer = tf2_ros.Buffer()
        tflistener = tf2_ros.TransformListener(self.buffer)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_detections_callback)
        self.next_command_time = 0


    def goToTarget(self, target):
        self.commandQueue.append(target)

    def tag_detections_callback(self, tag_detections):
        if (rospy.Time.now().to_sec() < self.next_command_time):
            return

        self.next_command_time = rospy.Time.now().to_sec() + 5


        # Loop through all detected tags
        for detection in tag_detections.detections:
            # Extract tag information
            tag_id = detection.id
            print("trying to poke tag ")
            print(tag_id)
            print("\n")
            tag_pose = detection.pose.pose.pose
            tag_size = detection.size

            # Convert the apriltag detection to a PoseStamped message in the camera_link frame
            camera_frame = detection.pose.header.frame_id
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = camera_frame
            pose_msg.pose.position.x = tag_pose.position.x
            pose_msg.pose.position.y = tag_pose.position.y
            pose_msg.pose.position.z = tag_pose.position.z

            pose_msg.pose.orientation.x = tag_pose.orientation.x
            pose_msg.pose.orientation.y = tag_pose.orientation.y
            pose_msg.pose.orientation.z = tag_pose.orientation.z
            pose_msg.pose.orientation.w = tag_pose.orientation.w

            # Convert the apriltag detection (camera frame) to a PoseStamped message in the map frame
            trans = self.buffer.lookup_transform('capstone/base_link', camera_frame, rospy.Time())

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
            pose_transformed.header.frame_id = 'capstone/base_link'

            #make the hand point in a nicer way (no roll or pitch)
            yaw = math.atan2(pose_transformed.pose.position.y, pose_transformed.pose.position.x)
            q = quaternion_from_euler(0,0,yaw)
            pose_transformed.pose.orientation.x = q[0]
            pose_transformed.pose.orientation.y = q[1]
            pose_transformed.pose.orientation.z = q[2]
            pose_transformed.pose.orientation.w = q[3]


            self.goToTarget(pose_transformed)

    def runNode(self):
        if rospy.Time.now().to_sec() < self.startTime + 1:
            return

        if len(self.commandQueue) > 0:
            command = self.commandQueue.pop(0)
            self._executeCommand(command)

    def _executeCommand(self, target):
        if type(target) == str:
            try:
                self.commander.set_named_target(target)
            except MoveItCommanderException:
                return
        elif type(target) == PoseStamped:
            self.commander.set_pose_target(target)
        else:
            return

        self.commander.go()

    def start(self):
        rate_param = float(rospy.get_param('~rate', '1'))
        rate = rospy.Rate(rate_param)
        while not rospy.is_shutdown():
            self.runNode()
            rate.sleep()


if __name__ == '__main__':
    roscpp_initialize(sys.argv)
    rospy.init_node('poke_tag')

    node = PokeTagNode()
    node.start()

    roscpp_shutdown()