#!/usr/bin/env python
# skripta za objavljanje senzorskih meritev

import rospy
import std_msgs.msg
import nav_msgs.msg
import geometry_msgs.msg
import sensor_fusion.msg
  
def build_cov():
  rospy.init_node("cov_build")
  rate = rospy.Rate(10)
  
  cov_laser_pub = rospy.Publisher("/covariance/laser", sensor_fusion.msg.FloatArray, queue_size=10)
  cov_odom_pub = rospy.Publisher("/covariance/odometry", sensor_fusion.msg.FloatArray, queue_size=10)
  cov_filter_pub = rospy.Publisher("/covariance/filter", sensor_fusion.msg.FloatArray, queue_size=10)
  cov_laser= []
  cov_odom = []
  cov_filter = []
 
 
  while not rospy.is_shutdown():
    laser_position = rospy.wait_for_message("/laser_position2", nav_msgs.msg.Odometry)
    odometry_position = rospy.wait_for_message("/odometry_position2", nav_msgs.msg.Odometry)
    filterMsg = rospy.wait_for_message("/ekf2", nav_msgs.msg.Odometry)
    
    #print(type(laser_position.pose.covariance[0]))
    #print(type(abs(laser_position.pose.covariance[0])+abs(laser_position.pose.covariance[7])))
    cov_laser.append(abs(laser_position.pose.covariance[0])+abs(laser_position.pose.covariance[7]))
    cov_odom.append(abs(odometry_position.pose.covariance[0])+abs(odometry_position.pose.covariance[7]))
    cov_filter.append(abs(filterMsg.pose.covariance[0])+abs(filterMsg.pose.covariance[7]))
    
    cov_laser_msg= sensor_fusion.msg.FloatArray()
    cov_odom_msg = sensor_fusion.msg.FloatArray()
    cov_filter_msg = sensor_fusion.msg.FloatArray()
    cov_laser_msg.data = cov_laser
    cov_odom_msg.data = cov_odom
    cov_filter_msg.data = cov_filter
    
    cov_laser_pub.publish(cov_laser_msg)
    cov_odom_pub.publish(cov_odom_msg)
    cov_filter_pub.publish(cov_filter_msg)
    
    rate.sleep()


if __name__ == "__main__":

  try:
    build_cov()
  except rospy.ROSInterruptException: pass

  
