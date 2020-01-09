#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time

from gpd_ros.srv import detect_grasps

service = '/detect_grasps_server_pointnet_realsense/detect_grasps'
# service = '/detect_grasps_server/detect_grasps'

def get_grasps_client():
    rospy.wait_for_service(service, timeout=1)
    try:
        detectGrasps = rospy.ServiceProxy(service, detect_grasps)
        resp = detectGrasps()
#         print resp.grasp_configs.grasps
        print "[INFO] Get %d grasps" % len(resp.grasp_configs.grasps)
        return resp.grasp_configs
    except rospy.ServiceException, e:
        print "[SRVICE] Service call failed: %s" % e


if __name__ == "__main__":
    while True:
        start = time.time()
        get_grasps_client()
        print "[INFO] Get grasps took:", time.time()-start