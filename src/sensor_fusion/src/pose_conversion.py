#!/usr/bin/env python
# skripta za objavljanje senzorskih meritev

import rospy
import nav_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
  
def processLaser(lasMsg):
  q = quaternion_from_euler(0.0, 0.0, lasMsg.theta)
  
  odomMsg = nav_msgs.msg.Odometry()
  odomMsg.header.frame_id = "odom"
  odomMsg.child_frame_id = "hokuyo_laser"
  
  odomMsg.pose.pose.position.x = lasMsg.x
  odomMsg.pose.pose.position.y = lasMsg.y
  odomMsg.pose.pose.position.z = 0.4
  
  odomMsg.pose.pose.orientation.x = q[0]
  odomMsg.pose.pose.orientation.y = q[1]
  odomMsg.pose.pose.orientation.z = q[2]
  odomMsg.pose.pose.orientation.w = q[3]
  
  odomMsg.pose.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
  
  posePub = rospy.Publisher("/laser_position", nav_msgs.msg.Odometry, queue_size=100)
  posePub.publish(odomMsg)
  
  
def convert():
  rospy.init_node("convert_pose")
  rate = rospy.Rate(10)
 
  lasSub = rospy.Subscriber("/pose2D", geometry_msgs.msg.Pose2D, processLaser)
  
  while not rospy.is_shutdown():
    rate.sleep()
  

if __name__ == "__main__":

  try:
    convert()
  except rospy.ROSInterruptException: pass

  
