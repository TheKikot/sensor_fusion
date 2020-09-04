#!/usr/bin/env python
# skripta za objavljanje senzorskih meritev

import rospy
import nav_msgs.msg
import geometry_msgs.msg
import math


def toQuat(roll, pitch, yaw):

  cy = math.cos(yaw * 0.5)
  sy = math.sin(yaw * 0.5)
  cp = math.cos(pitch * 0.5)
  sp = math.sin(pitch * 0.5)
  cr = math.cos(roll * 0.5)
  sr = math.sin(roll * 0.5)

  q = [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
      ]

  return q  
  
  
def processLaser(lasMsg):
  #print "theta: %0.2f" % (lasMsg.theta)
  odomMsg = nav_msgs.msg.Odometry()
  odomMsg.pose.pose.position.x = lasMsg.x
  odomMsg.pose.pose.position.y = lasMsg.y
  odomMsg.pose.pose.position.z = 0.0
  q = toQuat(0.0, 0.0, lasMsg.theta)
  odomMsg.pose.pose.orientation.x = q[1]
  odomMsg.pose.pose.orientation.y = q[2]
  odomMsg.pose.pose.orientation.z = q[3]
  odomMsg.pose.pose.orientation.w = q[0]
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

  
