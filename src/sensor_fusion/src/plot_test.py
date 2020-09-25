#!/usr/bin/env python
# skripta za prikaz napake / razlike med potmi

import rospy
import nav_msgs.msg
import geometry_msgs.msg
import matplotlib.pyplot as plt
import numpy as np
#import math


def distance(path1, path2):

  dist = []
  
  length = len(path2)
  if len(path1) < len(path2) :
    print("path1 is shorter than path2 by %d", len(path2)-len(path1))
    length = len(path1)
  else :
    print("path1 is longer than path2 by %d", len(path1)-len(path2))
  
  for i in range(0, length) :
    dist.append(
      (path1[i] - path2[i])**2
    )
  
  return dist
  
def calculate_distances():
  odom_path = range(0, 100) + np.random.normal(0,1.0,100)
  laser_path = range(0, 100) + np.random.normal(0,0.5,100)
  filter_path = range(0, 100) + np.random.normal(0,0.3,100)
  gps_path = range(0, 100)

  
    
  
  odom_dist = distance(odom_path, gps_path)
  laser_dist = distance(laser_path, gps_path)
  filter_dist = distance(filter_path, gps_path)
  
  
  plt.plot(range(0, len(odom_dist)), odom_dist, 'r', label='odometry')
  plt.plot(range(0, len(laser_dist)), laser_dist, 'm', label='laser')
  plt.plot(range(0, len(filter_dist)), filter_dist, 'g', label='filter')
  plt.xlabel('point in trajectory')
  plt.ylabel('square distance between points')
  plt.title("Square trajectory error")
  plt.legend()
  plt.show()
  
  
  

if __name__ == "__main__":

  try:
    calculate_distances()
  except rospy.ROSInterruptException: pass

  
