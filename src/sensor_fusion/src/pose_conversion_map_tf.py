#!/usr/bin/env python
# skripta za objavljanje senzorskih meritev

import rospy
import nav_msgs.msg
import geometry_msgs.msg
import marvelmind_nav.msg
import math
import tf
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

global init_gps_msg
br = tf.TransformBroadcaster()
#tfPub = rospy.Publisher("map_to_odom", marvelmind_nav.msg.hedge_pos_ang, queue_size=10)
last_gps_measurement = (0,0,0) # used to average current measurement with previous to get the middle point between the two beacons

def publish_tf():
  global init_gps_msg
  global br
  br.sendTransform((init_gps_msg.x_m, init_gps_msg.y_m, init_gps_msg.z_m),
                     quaternion_from_euler(0.0, 0.0, ((math.pi*init_gps_msg.angle/180) )),
                     rospy.Time.now(), "odom", "map")
  #tfPub.publish(init_gps_msg)

# reads the initial pose by averaging num readings from indoor gps
def init_pose(num):
  global init_gps_msg
  init_gps_msg=marvelmind_nav.msg.hedge_pos_ang()
  init_gps_msg.x_m = 0
  init_gps_msg.y_m = 0
  init_gps_msg.z_m = 0
  init_gps_msg.angle = 0
  msg = [0 for i in range(num)]
  for i in range(num):
    msg[i] = rospy.wait_for_message("/hedge_pos_ang", marvelmind_nav.msg.hedge_pos_ang, timeout=3)
    init_gps_msg.x_m += (msg[i].x_m)/num
    init_gps_msg.y_m += (msg[i].y_m)/num
    init_gps_msg.z_m += (msg[i].z_m)/num
    init_gps_msg.angle += (msg[i].angle)/num
  

                       
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
  
  odomMsg.pose.covariance = [1e-5, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 1e-5, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
  
  posePub = rospy.Publisher("/laser_position", nav_msgs.msg.Odometry, queue_size=100)
  posePub.publish(odomMsg)
  
def processGPS(gpsMsg):
  global last_gps_measurement
  q = quaternion_from_euler(0.0, 0.0, ((math.pi*gpsMsg.angle/180) ))
  
  odomMsg = nav_msgs.msg.Odometry()
  odomMsg.header.stamp = rospy.Time.now()
  odomMsg.header.frame_id = "map"
  odomMsg.child_frame_id = "hedgehog"
  
  odomMsg.pose.pose.position.x = (gpsMsg.x_m + last_gps_measurement[0])/2 # averaging with the measurement from the other paired hedgehog
  odomMsg.pose.pose.position.y = (gpsMsg.y_m + last_gps_measurement[1])/2 # averaging with the measurement from the other paired hedgehog
  odomMsg.pose.pose.position.z = gpsMsg.z_m
  
  odomMsg.pose.pose.orientation.x = q[0]
  odomMsg.pose.pose.orientation.y = q[1]
  odomMsg.pose.pose.orientation.z = q[2]
  odomMsg.pose.pose.orientation.w = q[3]
  
  
  odomMsg.pose.covariance = [1e-10, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 1e-10, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
                      
  last_gps_measurement = (gpsMsg.x_m, gpsMsg.y_m, gpsMsg.angle)   
  posePub = rospy.Publisher("/hedgehog_position", nav_msgs.msg.Odometry, queue_size=100)
  posePub.publish(odomMsg)
  
def convert(): 
  lasSub = rospy.Subscriber("/pose2D", geometry_msgs.msg.Pose2D, processLaser)
  gpsSub = rospy.Subscriber("/hedge_pos_ang", marvelmind_nav.msg.hedge_pos_ang, processGPS)
  
  while not rospy.is_shutdown():
    publish_tf()
    rate.sleep()
  

if __name__ == "__main__":

  try:
    rospy.init_node("convert_pose")
    rate = rospy.Rate(50)
 
    init_pose(20)
    convert()
  except rospy.ROSInterruptException: pass

  
