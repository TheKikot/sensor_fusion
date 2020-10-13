#!/usr/bin/env python
# skripta za objavljanje senzorskih meritev

import rospy
import std_msgs.msg
import nav_msgs.msg
import geometry_msgs.msg
import marvelmind_nav.msg
from tf.transformations import quaternion_from_euler
#from visualization_msgs.msg import Marker
  
def processLaser(lasMsg):
  q = quaternion_from_euler(0.0, 0.0, lasMsg.theta)
  
  odomMsg = nav_msgs.msg.Odometry()
  odomMsg.header.stamp = rospy.Time.now()
  odomMsg.header.frame_id = "odom"
  odomMsg.child_frame_id = "hokuyo_laser"
  
  odomMsg.pose.pose.position.x = lasMsg.x
  odomMsg.pose.pose.position.y = lasMsg.y
  odomMsg.pose.pose.position.z = 0.4
  
  odomMsg.pose.pose.orientation.x = q[0]
  odomMsg.pose.pose.orientation.y = q[1]
  odomMsg.pose.pose.orientation.z = q[2]
  odomMsg.pose.pose.orientation.w = q[3]
  
  odomMsg.pose.covariance = [1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 1e-2]
  
  posePub = rospy.Publisher("/laser_position2", nav_msgs.msg.Odometry, queue_size=100)
  posePub.publish(odomMsg)
  return odomMsg
  
def processOdometry(odomMsg):
    
  odomMsg.pose.covariance = [1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 1e3]
  
  posePub = rospy.Publisher("/odometry_position2", nav_msgs.msg.Odometry, queue_size=100)
  posePub.publish(odomMsg)
  return odomMsg
  
def convert():
  rospy.init_node("convert_pose")
  rate = rospy.Rate(10)
  
  cov_laser_pub = rospy.Publisher("/covariance/laser", std_msgs.msg.Float64, queue_size=10)
  cov_odom_pub = rospy.Publisher("/covariance/odometry", std_msgs.msg.Float64, queue_size=10)
  cov_filter_pub = rospy.Publisher("/covariance/filter", std_msgs.msg.Float64, queue_size=10)
  cov_laser= []
  cov_odom = []
  cov_filter = []
 
 
  while not rospy.is_shutdown():
    lasMsg = rospy.wait_for_message("/pose2D", geometry_msgs.msg.Pose2D)
    odomMsg = rospy.wait_for_message("/odom", nav_msgs.msg.Odometry)
    laser_position = processLaser(lasMsg)
    odometry_position = processOdometry(odomMsg)
    rate.sleep()
    

if __name__ == "__main__":

  try:
    convert()
  except rospy.ROSInterruptException: pass

  
