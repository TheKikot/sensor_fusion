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
  
  lasSub = rospy.Subscriber("/pose2D", geometry_msgs.msg.Pose2D, processLaser)
  odoSub = rospy.Subscriber("/odom", nav_msgs.msg.Odometry, processOdometry)
  
  
  
  rospy.init_node("sensor_fusion")
  rate = rospy.Rate(1)
  while not rospy.is_shutdown():
    #velPub.publish(vel)
    print "Estimated position: x:%0.2f y:%0.2f z:%0.2f roll:%0.2f pitch:%0.2f yaw:%0.2f" %
    (.1, .2, .3, .4, .5, .6) 
    rate.sleep()
  
