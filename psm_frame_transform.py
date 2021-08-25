#!/usr/bin/env python
import dvrk
import PyKDL
import rospy
import tf
from geometry_msgs.msg import Pose
import time
import numpy as np
import math

rospy.init_node('psm_base_transform')

tf_listener = tf.TransformListener()
time.sleep(1)
tf_listener.getFrameStrings()


def tf_to_pykdl_frame(tfl_frame):
	pos, rot_quat = tfl_frame
	pos2 = PyKDL.Vector(*pos)
	rot = PyKDL.Rotation.Quaternion(*rot_quat)
	return PyKDL.Frame(rot, pos2)

def update_transforms(): #update the transforms   
	
	tf_psm1_to_cam = tf_to_pykdl_frame(tf_listener.lookupTransform('Vision_sensor_left', 'J1_PSM1',rospy.Time()))
	tf_psm2_to_cam = tf_to_pykdl_frame(tf_listener.lookupTransform('Vision_sensor_left', 'J1_PSM2',rospy.Time()))
	tf_ecm_to_cam = tf_to_pykdl_frame(tf_listener.lookupTransform('Vision_sensor_left', 'J1_ECM',rospy.Time()))

	PSM_J1_TO_BASE_LINK = PyKDL.Frame(PyKDL.Rotation.RPY(math.pi/2.0, -math.pi, 0), PyKDL.Vector())
	ECM_J1_TO_BASE_LINK = PyKDL.Frame(PyKDL.Rotation.RPY(math.pi/2.0, -math.pi/2.0, 0), PyKDL.Vector())

	fix = PyKDL.Frame(PyKDL.Rotation.RPY(0,math.pi,0), PyKDL.Vector())

	frame1 =  fix * tf_psm1_to_cam * PSM_J1_TO_BASE_LINK
	frame2 = fix * tf_psm2_to_cam * PSM_J1_TO_BASE_LINK
	frame3 = fix * tf_ecm_to_cam * ECM_J1_TO_BASE_LINK

	rot1 = frame1.M.GetQuaternion()
	rot2 = frame2.M.GetQuaternion()
	rot3 = frame3.M.GetQuaternion()

	trans1 = frame1.p
	trans2 = frame2.p
	trans3 = frame3.p

	pose1.position.x = trans1[0]
	pose1.position.y = trans1[1]
	pose1.position.z = trans1[2]
	
	pose1.orientation.x = rot1[0]
	pose1.orientation.y = rot1[1]
	pose1.orientation.z = rot1[2]
	pose1.orientation.w = rot1[3]

	pose2.position.x = trans2[0]
	pose2.position.y = trans2[1]
	pose2.position.z = trans2[2]
	
	pose2.orientation.x = rot2[0]
	pose2.orientation.y = rot2[1]
	pose2.orientation.z = rot2[2]
	pose2.orientation.w = rot2[3]

	pose3.position.x = trans3[0]
	pose3.position.y = trans3[1]
	pose3.position.z = trans3[2]
	
	pose3.orientation.x = rot3[0]
	pose3.orientation.y = rot3[1]
	pose3.orientation.z = rot3[2]
	pose3.orientation.w = rot3[3]

pub1 = rospy.Publisher('/dvrk/PSM1/set_base_frame', Pose, queue_size=10)
pub2 = rospy.Publisher('/dvrk/PSM2/set_base_frame', Pose, queue_size=10)
pub3 = rospy.Publisher('/dvrk/ECM/set_base_frame', Pose, queue_size=10)

rate = rospy.Rate(100) # 10hz
pose1 = Pose()
pose2 = Pose()
pose3 = Pose()
while not rospy.is_shutdown():
	update_transforms()
	pub1.publish(pose1)
	pub2.publish(pose2)
	pub3.publish(pose3)
	rate.sleep()
