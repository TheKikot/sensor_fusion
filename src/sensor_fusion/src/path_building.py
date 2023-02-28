#!/usr/bin/env python
# skripta za shranjevanje poti 

import rospy
import nav_msgs.msg
import geometry_msgs.msg
import marvelmind_nav.msg
import message_filters

# spremenljivke ki hranijo zapis poti
global pathOdom
global pathFilter
global pathFilter2
global pathLaser
global pathGPS

# pretvorba formata informacije o polozaju
def odom_to_path(msg, path, frame_id):
  pose = geometry_msgs.msg.PoseStamped()
  pose.pose = msg.pose.pose
  pose.header.stamp = msg.header.stamp
  pose.header.frame_id = msg.header.frame_id
  
  path.header.stamp = rospy.Time.now()
  path.header.frame_id = frame_id
  path.poses.append(pose)
  
  return path
  
# doda vsaki poti novi polozaj
def addToPath(odom_msg, laser_msg, filter_msg, filter2_msg, gps_msg):
  global pathOdom
  global pathLaser
  global pathFilter
  global pathFilter2
  global pathGPS
  
  pathOdom = odom_to_path(odom_msg, pathOdom, "odom")
  pathLaser = odom_to_path(laser_msg, pathLaser, "odom")
  pathFilter = odom_to_path(filter_msg, pathFilter, "odom")
  pathFilter2 = odom_to_path(filter2_msg, pathFilter2, "odom")
  pathGPS = odom_to_path(gps_msg, pathGPS, "map")
  
  pathPub = rospy.Publisher("/odom_path", nav_msgs.msg.Path, queue_size=10)
  pathPub.publish(pathOdom)
  
  pathPub = rospy.Publisher("/laser_path", nav_msgs.msg.Path, queue_size=10)
  pathPub.publish(pathLaser)
  
  pathPub = rospy.Publisher("/filter_path", nav_msgs.msg.Path, queue_size=10)
  pathPub.publish(pathFilter)
  
  pathPub = rospy.Publisher("/filter2_path", nav_msgs.msg.Path, queue_size=10)
  pathPub.publish(pathFilter2)
  
  pathPub = rospy.Publisher("/GPS_path", nav_msgs.msg.Path, queue_size=10)
  pathPub.publish(pathGPS)
  
# glavna funkcija, ki 10x v sekundi pocaka na vse 4 polozaje in klice addToPath, da jih doda v poti
def make_path():
  rospy.init_node("path_building")
  rate = rospy.Rate(10)
  global pathOdom
  global pathLaser
  global pathFilter
  global pathFilter2
  global pathGPS

  pathOdom = nav_msgs.msg.Path()
  pathLaser = nav_msgs.msg.Path()
  pathFilter = nav_msgs.msg.Path()
  pathFilter2 = nav_msgs.msg.Path()
  pathGPS = nav_msgs.msg.Path()
  
  while not rospy.is_shutdown():
    odom_msg = rospy.wait_for_message("/odom", nav_msgs.msg.Odometry)
    laser_msg = rospy.wait_for_message("/laser_position", nav_msgs.msg.Odometry)
    filter_msg = rospy.wait_for_message("/odometry/filtered", nav_msgs.msg.Odometry)
    filter2_msg = rospy.wait_for_message("/odometry/filtered2", nav_msgs.msg.Odometry)
    gps_msg = rospy.wait_for_message("/hedgehog_position", nav_msgs.msg.Odometry)
    
    addToPath(odom_msg, laser_msg, filter_msg, filter2_msg, gps_msg)
    
    rate.sleep()


if __name__ == "__main__":

  try:
    make_path()
  except rospy.ROSInterruptException: pass

  
