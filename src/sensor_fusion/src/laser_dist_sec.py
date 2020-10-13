#!/usr/bin/env python
# skripta za prikaz napake / razlike med potmi

import rospy
import nav_msgs.msg
import geometry_msgs.msg
import matplotlib.pyplot as plt
#import math


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
  
def calculate_distances():
  rospy.init_node("path_distance")
  rate = rospy.Rate(10)
  
    
  odom_path = rospy.wait_for_message("/_odom_path", nav_msgs.msg.Path)
  laser_path = rospy.wait_for_message("/_laser_path", nav_msgs.msg.Path)
  filter_path = rospy.wait_for_message("/_filter_path", nav_msgs.msg.Path)
  sec_filter_path = rospy.wait_for_message("/_filter2_path", nav_msgs.msg.Path)
  gps_path = rospy.wait_for_message("/_GPS_path", nav_msgs.msg.Path)
  
  odom_dist = distance(odom_path, laser_path)
  filter_dist = distance(filter_path, laser_path)
  sec_dist = distance(sec_filter_path, laser_path)
  
  
  plt.plot(range(0, len(odom_dist)), odom_dist, 'r', label='odometry')
  plt.plot(range(0, len(filter_dist)), filter_dist, 'g', label='filter')
  plt.plot(range(0, len(sec_dist)), sec_dist, 'y', label='secondary_filter')
  plt.xlabel('point in trajectory')
  plt.ylabel('square distance between points')
  plt.title("Square trajectory error")
  plt.legend()
  plt.show()
  

if __name__ == "__main__":

  try:
    calculate_distances()
  except rospy.ROSInterruptException: pass

  
