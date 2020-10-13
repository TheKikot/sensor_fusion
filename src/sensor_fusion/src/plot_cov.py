#!/usr/bin/env python
# skripta za prikaz variance

import rospy
import std_msgs.msg
import nav_msgs.msg
import geometry_msgs.msg
import matplotlib.pyplot as plt
import sensor_fusion.msg


def plot_covariances():
  rospy.init_node("plot_cov")
  rate = rospy.Rate(10)
  
    
  
  
  odom_cov = rospy.wait_for_message("/covariance/laser", sensor_fusion.msg.FloatArray)
  laser_cov = rospy.wait_for_message("/covariance/odometry", sensor_fusion.msg.FloatArray)
  filter_cov = rospy.wait_for_message("/covariance/filter", sensor_fusion.msg.FloatArray)
  
  
  plt.plot(range(0, len(odom_cov.data)), odom_cov.data, 'r', label='odometry')
  plt.plot(range(0, len(laser_cov.data)), laser_cov.data, 'm', label='laser')
  plt.plot(range(0, len(filter_cov.data)), filter_cov.data, 'g', label='filter')
  
  plt.xlabel('point in trajectory')
  plt.ylabel('summed x and y variance')
  plt.title("Position variance")
  plt.legend()
  plt.show()
    

if __name__ == "__main__":

  try:
    plot_covariances()
  except rospy.ROSInterruptException: pass


  
