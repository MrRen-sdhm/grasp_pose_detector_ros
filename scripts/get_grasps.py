#!/usr/bin/env python
# -*- coding: utf-8 -*-

# from __future__ import print_function
from __future__ import division

import numpy as np
import rospy
import tf
from transforms3d import quaternions # http://matthew-brett.github.io/transforms3d/
from gpd_ros.msg import GraspConfigList


min_score = 0.95
# camera_frame = "kinect2_rgb_optical_frame"
camera_frame = "camera_color_optical_frame"


def callback(msg):
    grasps = msg.grasps
    
    if len(grasps) > 0:
        print("\n")
        # rospy.loginfo('Received %d grasps.', len(grasps))
        print "\033[0;32m%s\033[0m" % ("[INFO GRASP] Received " + str(len(grasps)) + " grasps")
      
        # print grasps
        # print grasps[0].score.data
        print "\033[0;32m%s\033[0m" % ("[INFO GRASP] Score: " + str(grasps[0].score.data) )

        # rot = np.array([ (grasps[0].approach.x, grasps[0].approach.y, grasps[0].approach.z), 
        #                  (grasps[0].binormal.x, grasps[0].binormal.y, grasps[0].binormal.z), 
        #                  (grasps[0].axis.x, grasps[0].axis.y, grasps[0].axis.z) ])
        if grasps[0].score.data > min_score :
            # 旋转矩阵的第一列为坐标系x轴方向向量，第二列为坐标系y轴方向向量，第三列为坐标系z轴方向向量
            rot = np.array([ (grasps[0].approach.x, grasps[0].binormal.x, grasps[0].axis.x), 
                        (grasps[0].approach.y, grasps[0].binormal.y, grasps[0].axis.y), 
                        (grasps[0].approach.z, grasps[0].binormal.z, grasps[0].axis.z) ])
            print "\033[0;32m%s\033[0m" % "[INFO GRASP] Rotation matrix:\n", rot

            quaternion = quaternions.mat2quat(rot) # [w, x, y, z]
            quaternion_y = quaternions.axangle2quat([0, 1, 0], np.pi/2) # 绕y轴旋转90度的四元数
            quaternion_z = quaternions.axangle2quat([0, 0, 1], np.pi/2) # 绕z轴旋转90度的四元数
            # 四元数相乘，即将原抓取姿态绕y轴旋转90度, 再绕z轴旋转90度，目的为使抓取姿态坐标系与末端执行器坐标系匹配
            quaternion = quaternions.qmult(quaternion, quaternion_y) 
            # quaternion = quaternions.qmult(quaternion, quaternion_z)
            print "\033[0;32m%s\033[0m" % "[INFO GRASP] Quaternion:", quaternion, "\n"

            br = tf.TransformBroadcaster()
            br.sendTransform((grasps[0].position.x, grasps[0].position.y, grasps[0].position.z), # (x, y , z)
                        (quaternion[1], quaternion[2], quaternion[3], quaternion[0]), # (x, y, z, w)
                        rospy.Time.now(), "grasp", camera_frame)


# Create a ROS node.
rospy.init_node('get_grasps')

# Subscribe to the ROS topic that contains the grasps.
sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, callback)

# Wait for grasps to arrive.
rate = rospy.Rate(100)
  
while not rospy.is_shutdown():    
    rate.sleep()


# rot = np.array([(0.82710765 , 0.5400628 , -0.15564418), 
# (0.47098615, -0.81711523, -0.33240749), 
# (-0.30670015,  0.20163052 ,-0.93020436)])
# quaternion = quaternions.mat2quat(rot)
# print quaternion # [ 0.26578311  0.66126224 -0.56370768 -0.41752274]

# M = quaternions.quat2mat(quaternion)
# print M
# print M[:,0]
