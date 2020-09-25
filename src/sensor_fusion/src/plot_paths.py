#!/usr/bin/env python
# skripta za prikaz napake / razlike med potmi

import rospy
import nav_msgs.msg
import geometry_msgs.msg
import matplotlib.pyplot as plt
#import math


def extract_coordinates(path):

  x = []
  y = []
  
  for i in range(0, len(path)) :
    x.append(path.poses[i].pose.position.x)
    y.append(path.poses[i].pose.position.y)
  
  return (x, y)
  
def plot_paths():
  rospy.init_node("plot_paths")
  rate = rospy.Rate(10)
  
    
  odom_path = rospy.wait_for_message("/odom_path", nav_msgs.msg.Path)
  laser_path = rospy.wait_for_message("/laser_path", nav_msgs.msg.Path)
  filter_path = rospy.wait_for_message("/filter_path", nav_msgs.msg.Path)
  gps_path = rospy.wait_for_message("/GPS_path", nav_msgs.msg.Path)
  
  (odomx, odomy) = extract_coordinates(odom_path)
  (laserx, lasery) = extract_coordinates(laser_path)
  (filterx, filtery) = extract_coordinates(filter_path)
  (gpsx, gpsy) = extract_coordinates(gps_path)
  
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
    plot_paths()
  except rospy.ROSInterruptException: pass

  
