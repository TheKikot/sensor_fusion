#!/usr/bin/env python
# skripta za objavljanje senzorskih meritev

import rospy
import nav_msgs.msg
import geometry_msgs.msg
import marvelmind_nav.msg
import math
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker

diff_x = 0
diff_y = 0
diff_angle = 0

def init_pose():
  global diff_x
  global diff_y
  global diff_angle
  init_pose_odom=rospy.wait_for_message("/odom", nav_msgs.msg.Odometry, timeout=3)
  init_gps_msg=rospy.wait_for_message("/hedge_pos_ang", marvelmind_nav.msg.hedge_pos_ang, timeout=3)
  init_angle_odom=euler_from_quaternion([init_pose_odom.pose.pose.orientation.x, init_pose_odom.pose.pose.orientation.y, init_pose_odom.pose.pose.orientation.z, init_pose_odom.pose.pose.orientation.w])
  #print("Init angle: ", init_angle_odom)
  
  diff_x = init_pose_odom.pose.pose.position.x - init_gps_msg.x_m
  diff_y = init_pose_odom.pose.pose.position.y - init_gps_msg.y_m
  diff_angle = init_angle_odom[2] - init_gps_msg.angle
  print(diff_x)
  print(diff_y)
  print(diff_angle)
  
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
  
  odomMsg.pose.covariance = [0.000001, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.000001, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.00001, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
  
  posePub = rospy.Publisher("/laser_position", nav_msgs.msg.Odometry, queue_size=100)
  posePub.publish(odomMsg)
  
def processGPS(gpsMsg):
  q = quaternion_from_euler(0.0, 0.0, ((math.pi*gpsMsg.angle/180) )) # - diff_angle))
  
  odomMsg = nav_msgs.msg.Odometry()
  odomMsg.header.stamp = rospy.Time.now()
  odomMsg.header.frame_id = "hedge-map"
  odomMsg.child_frame_id = "hedgehog"
  
  odomMsg.pose.pose.position.x = gpsMsg.x_m #+ diff_x
  odomMsg.pose.pose.position.y = gpsMsg.y_m #+ diff_y
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
  '''                           
  odomMsg.pose.covariance = [1e-09, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.001, 1e-09, 0.0, 0.0, 0.0,
                             0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 1e-09]
  '''                        
  posePub = rospy.Publisher("/hedgehog_position", nav_msgs.msg.Odometry, queue_size=100)
  posePub.publish(odomMsg)
  
def convert(): 
  lasSub = rospy.Subscriber("/pose2D", geometry_msgs.msg.Pose2D, processLaser)
  gpsSub = rospy.Subscriber("/hedge_pos_ang", marvelmind_nav.msg.hedge_pos_ang, processGPS)
  
  while not rospy.is_shutdown():
    rate.sleep()
  

if __name__ == "__main__":

  try:
    rospy.init_node("convert_pose")
    rate = rospy.Rate(50)
 
    init_pose()
    print(diff_x)
    print(diff_y)
    print(diff_angle)
    convert()
  except rospy.ROSInterruptException: pass

  
