#!/usr/bin/env python3

"""
Replicates the gazebo_robot_testbed control scheme, but uses MoveIt

"""
import math
import sys
import time

import rospy
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

from moveit_commander import MoveGroupCommander, MoveItCommanderException, roscpp_initialize, roscpp_shutdown

from geometry_msgs.msg import PoseStamped
from common_robot_codebase.srv import SetString

### april tag imports
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection

# from tf2_ros import TransformListener, TransformStamped, Buffer, tf2_geometry_msgs
import tf2_ros
import tf2_geometry_msgs



class DriveTagNode(object):
    def __init__(self):

        ## tag poke stuff
        self.buffer = tf2_ros.Buffer()
        tflistener = tf2_ros.TransformListener(self.buffer)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_detections_callback)
        self.next_command_time = 0

        # Create publisher for move_base destination pose
        self.move_base_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)


    def tag_detections_callback(self, tag_detections):


        # Loop through all detected tags
        for detection in tag_detections.detections:
            # Extract tag information
            tag_id = detection.id
            print("trying to drive to tag ")
            print(tag_id)
            print("\n")

            pose_msg = PoseStamped()
            pose_msg.header.frame_id = 'tag_'+str(tag_id[0])
            pose_msg.pose.position.x = 0
            pose_msg.pose.position.y = 0
            pose_msg.pose.position.z = 1
            q = quaternion_from_euler(0, -1.5707, 1.5707)
            pose_msg.pose.orientation.x = q[0]
            pose_msg.pose.orientation.y = q[1]
            pose_msg.pose.orientation.z = q[2]
            pose_msg.pose.orientation.w = q[3]

            # Convert the apriltag detection (camera frame) to a PoseStamped message in the map frame
            trans = self.buffer.lookup_transform('capstone/base_link', 'tag_'+str(tag_id[0]), rospy.Time())

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
            quaternion = (pose_transformed.pose.orientation.x, pose_transformed.pose.orientation.y, pose_transformed.pose.orientation.z, pose_transformed.pose.orientation.w)
            roll, pitch, yaw = euler_from_quaternion(quaternion)

            q = quaternion_from_euler(0,0,yaw+3.14159)
            pose_transformed.pose.orientation.x = q[0]
            pose_transformed.pose.orientation.y = q[1]
            pose_transformed.pose.orientation.z = q[2]
            pose_transformed.pose.orientation.w = q[3]

            pose_transformed.pose.position.z = 0

            self.move_base_pub.publish(pose_transformed)



    def start(self):
        rate_param = float(rospy.get_param('~rate', '1'))
        rate = rospy.Rate(rate_param)
        while not rospy.is_shutdown():
            rospy.spin()
            rate.sleep()


if __name__ == '__main__':
    roscpp_initialize(sys.argv)
    rospy.init_node('drive_tag')

    node = DriveTagNode()
    node.start()

    roscpp_shutdown()