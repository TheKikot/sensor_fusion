#!/usr/bin/env python
# skripta za shranjevanje poti 

import rospy
import nav_msgs.msg
import geometry_msgs.msg
import marvelmind_nav.msg
#import math

global pathOdom
global pathFilter
global pathLaser
global pathGPS
global gps_init_pose


def addOdometry(msg):
  global pathOdom
  
  
  pose = geometry_msgs.msg.PoseStamped()
  pose.pose = msg.pose.pose
  pose.header.stamp = msg.header.stamp
  pose.header.frame_id = msg.header.frame_id
  
  pathOdom.header.stamp = rospy.Time.now()
  pathOdom.header.frame_id = "odom"
  pathOdom.poses.append(pose)
  
  pathPub = rospy.Publisher("/odom_path", nav_msgs.msg.Path, queue_size=100)
  pathPub.publish(pathOdom)


def addLaser(msg):
  global pathLaser
  
  
  pose = geometry_msgs.msg.PoseStamped()
  pose.pose = msg.pose.pose
  pose.header.stamp = msg.header.stamp
  pose.header.frame_id = msg.header.frame_id
  
  pathLaser.header.stamp = rospy.Time.now()
  pathLaser.header.frame_id = "odom"
  pathLaser.poses.append(pose)
  
  pathPub = rospy.Publisher("/laser_path", nav_msgs.msg.Path, queue_size=100)
  pathPub.publish(pathLaser)


def addFilter(msg):
  global pathFilter
  
  
  pose = geometry_msgs.msg.PoseStamped()
  pose.pose = msg.pose.pose
  pose.header.stamp = msg.header.stamp
  pose.header.frame_id = msg.header.frame_id
  
  pathFilter.header.stamp = rospy.Time.now()
  pathFilter.header.frame_id = "odom"
  pathFilter.poses.append(pose)
  
  pathPub = rospy.Publisher("/filter_path", nav_msgs.msg.Path, queue_size=100)
  pathPub.publish(pathFilter)

def addGPS(msg):
  global pathGPS
  
  
  pose = geometry_msgs.msg.PoseStamped()
  pose.pose.position.x = msg.x_m*gps_init_pose[3] + gps_init_pose[0]
  pose.pose.position.y = msg.y_m*gps_init_pose[4] + gps_init_pose[1]
  #pose.pose.position.z = msg.z_m*gps_init_pose[5] + gps_init_pose[2]
  pose.header.stamp = rospy.Time.now()
  pose.header.frame_id = "map"
  
  pathGPS.header.stamp = rospy.Time.now()
  pathGPS.header.frame_id = "map"
  pathGPS.poses.append(pose)
  
  pathPub = rospy.Publisher("/GPS_path", nav_msgs.msg.Path, queue_size=100)
  pathPub.publish(pathGPS)


  
def make_path():
  rospy.init_node("path_building")
  rate = rospy.Rate(10)
  global pathOdom
  global pathLaser
  global pathFilter
  global pathGPS
  global gps_init_pose
  pathOdom = nav_msgs.msg.Path()
  pathLaser = nav_msgs.msg.Path()
  pathFilter = nav_msgs.msg.Path()
  pathGPS = nav_msgs.msg.Path()
  gps_init_pose = []
  
  odom_pose = rospy.wait_for_message("/odom", nav_msgs.msg.Odometry)
  gps_pose = rospy.wait_for_message("/hedge_pos", marvelmind_nav.msg.hedge_pos)
  gps_init_pose.append( odom_pose.pose.pose.position.x - gps_pose.x_m )
  gps_init_pose.append( odom_pose.pose.pose.position.y - gps_pose.y_m )
  gps_init_pose.append( odom_pose.pose.pose.position.z - gps_pose.z_m )
  gps_init_pose.append( 1.0 )
  gps_init_pose.append( 1.0 )
  gps_init_pose.append( 1.0 )
  
  odomSub = rospy.Subscriber("/odom", nav_msgs.msg.Odometry, addOdometry)
  laserSub = rospy.Subscriber("/laser_position", nav_msgs.msg.Odometry, addLaser)
  filterSub = rospy.Subscriber("/odometry/filtered", nav_msgs.msg.Odometry, addFilter)
  gpsSub = rospy.Subscriber("/hedge_pos", marvelmind_nav.msg.hedge_pos, addGPS)
  
  
  while not rospy.is_shutdown():
    rate.sleep()
  

if __name__ == "__main__":

  try:
    make_path()
  except rospy.ROSInterruptException: pass

  
