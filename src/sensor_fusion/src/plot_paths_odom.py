#!/usr/bin/env python
# skripta za prikaz napake / razlike med potmi

import rospy
import math
import nav_msgs.msg
import matplotlib.pyplot as plt
import tf2_ros
import tf2_geometry_msgs
import marvelmind_nav.msg
from tf.transformations import quaternion_from_euler


def extract_coordinates(path):

  x = []
  y = []
  
  for i in range(0, len(path.poses)) :
    x.append(path.poses[i].pose.position.x)
    y.append(path.poses[i].pose.position.y)
  
  return (x, y)
  
def plot_paths():
  tfBuffer = tf2_ros.Buffer()
  listener = tf2_ros.TransformListener(tfBuffer)
        
  odom_path = rospy.wait_for_message("/odom_path", nav_msgs.msg.Path)
  laser_path = rospy.wait_for_message("/laser_path", nav_msgs.msg.Path)
  filter_path = rospy.wait_for_message("/filter_path", nav_msgs.msg.Path)
  gps_path = rospy.wait_for_message("/GPS_path", nav_msgs.msg.Path)
  
  # transform gps_path from map -> odom frame
  time = rospy.Time.now()
  
  poseGps = gps_path.poses[0]
  poseGps.header.stamp = time
  poseGps = tfBuffer.transform(poseGps, 'odom', rospy.Duration(1.0))
  
  gps_path_odom = nav_msgs.msg.Path()
  gps_path_odom.header.frame_id = "odom"
  gps_path_odom.header.stamp = time
  
  for pose in gps_path.poses:
    pose.header.stamp = time
    pose = tfBuffer.transform(pose, 'odom', rospy.Duration(1.0))
    gps_path_odom.poses.append(pose)
    
  
  (odomx, odomy) = extract_coordinates(odom_path)
  (laserx, lasery) = extract_coordinates(laser_path)
  (filterx, filtery) = extract_coordinates(filter_path)
  (gpsx, gpsy) = extract_coordinates(gps_path_odom)
  
  plt.plot(odomx, odomy, 'r', label='odometry')
  plt.plot(laserx, lasery, 'm', label='laser')
  plt.plot(filterx, filtery, 'g', label='filter')
  plt.plot(gpsx, gpsy, 'b', label='gps')
  plt.xlabel('x')
  plt.ylabel('y')
  plt.title("trajectories")
  plt.legend()
  plt.show()
  

if __name__ == "__main__":

  try:
    rospy.init_node("plot_paths")
    rate = rospy.Rate(10)
    plot_paths()
  except rospy.ROSInterruptException: pass

  
