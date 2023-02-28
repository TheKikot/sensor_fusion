#!/usr/bin/env python
# skripta za samostojno voznjo robota

import rospy
import math
import nav_msgs.msg
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler, euler_from_quaternion

measurements_odom = nav_msgs.msg.Path()
measurements_laser = nav_msgs.msg.Path()
measurements_filter = nav_msgs.msg.Path()
measurements_gps = nav_msgs.msg.Path()
odomPub = rospy.Publisher("/average_measurements/odom", nav_msgs.msg.Path, queue_size=10)
laserPub = rospy.Publisher("/average_measurements/laser", nav_msgs.msg.Path, queue_size=10)
filterPub = rospy.Publisher("/average_measurements/filter", nav_msgs.msg.Path, queue_size=10)
gpsPub = rospy.Publisher("/average_measurements/gps", nav_msgs.msg.Path, queue_size=10)


# initializes an Odometry msg with zeros
def init_zero_odom():
  odomMsg = nav_msgs.msg.Odometry()
  
  odomMsg.pose.pose.position.x = 0.0
  odomMsg.pose.pose.position.y = 0.0
  
  odomMsg.pose.pose.orientation.x = 0.0
  odomMsg.pose.pose.orientation.y = 0.0
  odomMsg.pose.pose.orientation.z = 0.0
  odomMsg.pose.pose.orientation.w = 0.0
  
  return odomMsg

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

# returns odom1 + odom2*factor, where odom1 and odom2 are nav_msgs.msg.Odometry objects and factor is a real number
def add_multiplied(odom1, odom2, factor):
  result = nav_msgs.msg.Odometry()
  
  result.header.frame_id = odom2.header.frame_id
  result.header.stamp = odom2.header.stamp
  result.child_frame_id = odom2.child_frame_id
  
  result.pose.pose.position.x = odom1.pose.pose.position.x + factor * odom2.pose.pose.position.x
  result.pose.pose.position.y = odom1.pose.pose.position.y + factor * odom2.pose.pose.position.y
  #print odom1.pose.pose.orientation.x
  q1 = (odom1.pose.pose.orientation.x, odom1.pose.pose.orientation.y, odom1.pose.pose.orientation.z, odom1.pose.pose.orientation.w)
  q2 = (odom2.pose.pose.orientation.x, odom2.pose.pose.orientation.y, odom2.pose.pose.orientation.z, odom2.pose.pose.orientation.w)
  q3 = quaternion_from_euler(0.0, 0.0, ( euler_from_quaternion(q1)[2] + factor * euler_from_quaternion(q2)[2] )
                                 )
  result.pose.pose.orientation.x = q3[0]
  result.pose.pose.orientation.y = q3[1]
  result.pose.pose.orientation.z = q3[2]
  result.pose.pose.orientation.w = q3[3]
  
  
  result.pose.covariance = odom2.pose.covariance
  
  return result 

# measures timesMeasured times, averages the measurements and adds the average to measurements_<source> global vars
def measure(timesMeasured):
  global measurements_odom
  global measurements_laser
  global measurements_filter
  global measurements_gps

  odom_avg = init_zero_odom()
  laser_avg = init_zero_odom()
  filter_avg = init_zero_odom()
  gps_avg = init_zero_odom()
  
  for i in range(timesMeasured):
    print "measuring..."
    odom_msg = rospy.wait_for_message("/odom", nav_msgs.msg.Odometry)
    laser_msg = rospy.wait_for_message("/laser_position", nav_msgs.msg.Odometry)
    filter_msg = rospy.wait_for_message("/odometry/filtered", nav_msgs.msg.Odometry)
    gps_msg = rospy.wait_for_message("/hedgehog_position", nav_msgs.msg.Odometry)
        
    odom_avg = add_multiplied(odom_avg, odom_msg, 1/timesMeasured)
    laser_avg = add_multiplied(laser_avg, laser_msg, 1/timesMeasured)
    filter_avg = add_multiplied(filter_avg, filter_msg, 1/timesMeasured)
    gps_avg = add_multiplied(gps_avg, gps_msg, 1/timesMeasured)
    
  measurements_odom = odom_to_path(odom_avg, measurements_odom, "odom")
  measurements_laser = odom_to_path(laser_avg, measurements_laser, "odom")
  measurements_filter = odom_to_path(filter_avg, measurements_filter, "odom")
  measurements_gps = odom_to_path(gps_avg, measurements_gps, "map")
  
  odomPub.publish(measurements_odom)
  laserPub.publish(measurements_laser)
  filterPub.publish(measurements_filter)
  gpsPub.publish(measurements_gps)

# accepts number of turns or steps, legth or time of a step or turn in 1/10s, drive speed and turn speed
def drive(numOfTurns, driveTime, driveSpeed, turnSpeed):
  forwardDrive = geometry_msgs.msg.Twist()
  forwardDrive.linear.x = driveSpeed
  forwardDrive.linear.y = 0
  forwardDrive.linear.z = 0
  forwardDrive.angular.x = 0
  forwardDrive.angular.y = 0
  forwardDrive.angular.z = 0
  
  turnLeft = geometry_msgs.msg.Twist()
  turnLeft.linear.x = 0
  turnLeft.linear.y = 0
  turnLeft.linear.z = 0
  turnLeft.angular.x = 0
  turnLeft.angular.y = 0
  turnLeft.angular.z = turnSpeed
  
  stay = geometry_msgs.msg.Twist()
  stay.linear.x = 0
  stay.linear.y = 0
  stay.linear.z = 0
  stay.angular.x = 0
  stay.angular.y = 0
  stay.angular.z = 0
  
  twistPub = rospy.Publisher("/mobile_base/commands/velocity",
                             geometry_msgs.msg.Twist, queue_size = 100)
  for i in range(numOfTurns):
    for i in range(driveTime):
      twistPub.publish(forwardDrive)
      rospy.sleep(0.1)
    for i in range(driveTime):
      twistPub.publish(turnLeft)
      rospy.sleep(0.1)
  
    twistPub.publish(stay)
    # save the points from all sources, average 10 measurements
    measure(10)
    rospy.sleep(0.1)
    
if __name__ == "__main__":

  try:
    rospy.init_node("drive_robot")
    rospy.Rate(10)
    drive(20, 10, 0.2, 0.2)
  except rospy.ROSInterruptException: pass
  
