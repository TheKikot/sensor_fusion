#!/usr/bin/env python
# skripta za shranjevanje poti 
#<!-- this is an experimental script for testing the curent configuration with data from command readings (/mobile_base/commands/velocity) -->

#TODO


import rospy
import nav_msgs.msg
import geometry_msgs.msg
import marvelmind_nav.msg
import message_filters

global pathOdom
global pathFilter
global pathFilter2
global pathLaser
global pathGPS
global gps_init_pose


def odom_to_path(msg, path, frame_id):
  pose = geometry_msgs.msg.PoseStamped()
  pose.pose = msg.pose.pose
  pose.header.stamp = msg.header.stamp
  pose.header.frame_id = msg.header.frame_id
  
  path.header.stamp = rospy.Time.now()
  path.header.frame_id = frame_id
  path.poses.append(pose)
  
  return path
  

def addToPath(odom_msg, laser_msg, filter_msg, gps_msg, filter2_msg):
  #print("called addToPath")
  global pathOdom
  global pathLaser
  global pathFilter
  global pathFilter2
  global pathGPS
  
  pathOdom = odom_to_path(odom_msg, pathOdom, "odom")
  pathLaser = odom_to_path(laser_msg, pathLaser, "odom")
  pathFilter = odom_to_path(filter_msg, pathFilter, "odom")
  pathGPS = odom_to_path(gps_msg, pathGPS, "odom")
  pathFilter2 = odom_to_path(filter2_msg, pathFilter2, "odom")
  
  pathPub = rospy.Publisher("/_odom_path", nav_msgs.msg.Path, queue_size=10)
  pathPub.publish(pathOdom)
  
  pathPub = rospy.Publisher("/_laser_path", nav_msgs.msg.Path, queue_size=10)
  pathPub.publish(pathLaser)
  
  pathPub = rospy.Publisher("/_filter_path", nav_msgs.msg.Path, queue_size=10)
  pathPub.publish(pathFilter)
  
  pathPub = rospy.Publisher("/_GPS_path", nav_msgs.msg.Path, queue_size=10)
  pathPub.publish(pathGPS)
  
  pathPub = rospy.Publisher("/_filter2_path", nav_msgs.msg.Path, queue_size=10)
  pathPub.publish(pathFilter2)
  
def make_path():
  rospy.init_node("path_building")
  rate = rospy.Rate(10)
  global pathOdom
  global pathLaser
  global pathFilter
  global pathGPS
  global pathFilter2
  global gps_init_pose
  pathOdom = nav_msgs.msg.Path()
  pathLaser = nav_msgs.msg.Path()
  pathFilter = nav_msgs.msg.Path()
  pathGPS = nav_msgs.msg.Path()
  pathFilter2 = nav_msgs.msg.Path()
  gps_init_pose = []
  
  odom_pose = rospy.wait_for_message("/odom", nav_msgs.msg.Odometry)
  gps_pose = rospy.wait_for_message("/hedgehog_position", nav_msgs.msg.Odometry)
  gps_init_pose.append( odom_pose.pose.pose.position.x - gps_pose.pose.pose.position.x )
  gps_init_pose.append( odom_pose.pose.pose.position.y - gps_pose.pose.pose.position.y )
  gps_init_pose.append( odom_pose.pose.pose.position.z - gps_pose.pose.pose.position.z )
  #print 'x bias: ', gps_init_pose[0], ', y bias: ', gps_init_pose[1] 
  
  while not rospy.is_shutdown():
    odom_msg = rospy.wait_for_message("/odom", nav_msgs.msg.Odometry)
    laser_msg = rospy.wait_for_message("/laser_position", nav_msgs.msg.Odometry)
    filter_msg = rospy.wait_for_message("/odometry/filtered", nav_msgs.msg.Odometry)
    gps_msg = rospy.wait_for_message("/hedgehog_position", nav_msgs.msg.Odometry)
    filter2_msg = rospy.wait_for_message("/ekf2", nav_msgs.msg.Odometry)
    
    # popravljanje koordinat in orientacije
    #b = gps_msg.pose.pose.position.x + gps_init_pose[0]
    #a = gps_msg.pose.pose.position.y + gps_init_pose[1]
    #gps_msg.pose.pose.position.x =  (a)
    #gps_msg.pose.pose.position.y =  (b)
    #gps_msg.pose.pose.position.z =  (gps_msg.pose.pose.position.z + gps_init_pose[2])
    addToPath(odom_msg, laser_msg, filter_msg, gps_msg, filter2_msg)
    
    rate.sleep()
  
  '''
  odomSub = message_filters.Subscriber("/odom", nav_msgs.msg.Odometry)
  laserSub = message_filters.Subscriber("/laser_position", nav_msgs.msg.Odometry)
  filterSub = message_filters.Subscriber("/odometry/filtered", nav_msgs.msg.Odometry)
  gpsSub = message_filters.Subscriber("/hedgehog_position", nav_msgs.msg.Odometry)
  
  ts = message_filters.TimeSynchronizer([odomSub, laserSub, filterSub, gpsSub], 100)
  ts.registerCallback(addToPath)
  
  print("initialized!")
  rospy.spin()
  '''

if __name__ == "__main__":

  try:
    make_path()
  except rospy.ROSInterruptException: pass

  
