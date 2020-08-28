#!/usr/bin/env python
# skripta za objavljanje senzorskih meritev

import roslib#; roslib.load_manifest('pioneerControl')
import rospy
import nav_msgs.msg
import p2os_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg

def processOdometry(self, odoMsg):
  print "linear: x=%0.2f, y=%0.2f, z=%0.2f" %(odoMsg.pose.pose.position.x,odoMsg.pose.pose.position.y, odoMsg.pose.pose.position.z)
  print "angular(quaternion): x=%0.2f, y=%0.2f, z=%0.2f, w=%0.2f" %(odoMsg.pose.pose.orientation.x,odoMsg.pose.pose.orientation.y, odoMsg.pose.pose.orientation.z, odoMsg.pose.pose.orientation.w)

def processSonar(self, sonMsg, sonLock):
  print sonMsg.ranges 
  
def processLaser(lasMsg):
  print "Got laser message."

if __name__ == "__main__":
  #ns = "pioneer"
  sonSub = rospy.Subscriber("/sonar", p2os_msgs.msg.SonarArray, processSonar)
  lasSub = rospy.Subscriber("/base_scan", sensor_msgs.msg.LaserScan, processLaser)
  odoSub = rospy.Subscriber("/odom", nav_msgs.msg.Odometry, processOdometry)
  '''
  ## publishers
  velPub = rospy.Publisher(ns+"/cmd_vel", geometry_msgs.msg.Twist)
  motPub = rospy.Publisher(ns+"/cmd_motor_state", p2os_msgs.msg.MotorState)

  count = 0
  vel = geometry_msgs.msg.Twist()
  vel.linear.x = 0.1
  vel.angular.z = 0.0
  '''
  rospy.init_node("sensor_readings")
  rate = rospy.Rate(1)
  while not rospy.is_shutdown():
    #velPub.publish(vel)
    rate.sleep()
  '''
	  if count >20:
	    vel.linear.x = 0.0
	    vel.angular.z = 0.0
	    velPub.publish(vel)
	    break
	  count += 1
  '''
