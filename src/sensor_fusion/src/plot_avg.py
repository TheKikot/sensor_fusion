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


def distance(path1, path2):

  dist = []
  
  length = len(path2.poses)
  if len(path1.poses) < len(path2.poses) :
    print("path1 is shorter than path2 by %d", len(path2.poses)-len(path1.poses))
    length = len(path1.poses)
  else :
    print("path1 is longer than path2 by %d", len(path1.poses)-len(path2.poses))
  
  for i in range(0, length) :
    dist.append(
      (path1.poses[i].pose.position.x - path2.poses[i].pose.position.x)**2
      + (path1.poses[i].pose.position.y - path2.poses[i].pose.position.y)**2
    )
  
  return dist


def plot_distances():
  tfBuffer = tf2_ros.Buffer()
  listener = tf2_ros.TransformListener(tfBuffer)
        
  odom_path = rospy.wait_for_message("/average_measurements/odom", nav_msgs.msg.Path)
  laser_path = rospy.wait_for_message("/average_measurements/laser", nav_msgs.msg.Path)
  filter_path = rospy.wait_for_message("/average_measurements/filter", nav_msgs.msg.Path)
  gps_path = rospy.wait_for_message("/average_measurements/gps", nav_msgs.msg.Path)
  
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
    
  
  odom_dist = distance(odom_path, gps_path_odom)
  laser_dist = distance(laser_path, gps_path_odom)
  filter_dist = distance(filter_path, gps_path_odom)
  
  plt.plot(range(0, len(odom_dist)), odom_dist, 'r', label='odometry')
  plt.plot(range(0, len(laser_dist)), laser_dist, 'm', label='laser')
  plt.plot(range(0, len(filter_dist)), filter_dist, 'g', label='filter')
  
  plt.xlabel('point in trajectory')
  plt.ylabel('square distance between points')
  plt.title("Square trajectory error")
  plt.legend()
  plt.show()
  print("mean square error for odometry: ", sum(odom_dist)/len(odom_dist))
  print("mean square error for laser: ", sum(laser_dist)/len(laser_dist))
  print("mean square error for filter: ", sum(filter_dist)/len(filter_dist))

if __name__ == "__main__":

  try:
    rospy.init_node("path_distance")
    rate = rospy.Rate(10)
    plot_distances()
  except rospy.ROSInterruptException: pass

  
