#!/usr/bin/env python

import rospy
import roslib
import sys
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, UInt8
from tello_driver.msg import TelloStatus
from time import sleep

global canLand, hori_speed, move_dis, last_time
canLand = False
hori_speed = 0
move_dis = 0
last_time = 0.0

def TO():
  
  takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size = 1)
  while takeoff_pub.get_num_connections() == 0:
    pass
  sleep(5)
  print("Takeoff")
  rate = rospy.Rate(10)
  msg = Empty()
  rospy.loginfo(msg)
  takeoff_pub.publish(msg)
  rate.sleep()

def ts_callback(data):
  global canLand, hori_speed, last_time, move_dis
  hori_speed = data.speed_horizontal_mps
  if last_time == 0.0:
    last_time = rospy.get_time()
  else:
    move_dis += (rospy.get_time() - last_time) * 0.2
    last_time = rospy.get_time()
  if data.fly_mode == 12:
    canLand = True

def cmd():
  #The queue_size argument is New in ROS hydro and limits the amount of queued messages if any subscriber is not receiving them fast enough
  cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size = 10)
  land_sub = rospy.Subscriber('/tello/status', TelloStatus, ts_callback)
  rate = rospy.Rate(10)

  count = 0

  global canLand, hori_speed, move_dis
  move_dis = 0
  while not rospy.is_shutdown():
    if move_dis < 1.0:
      msg = Twist()
      msg.linear.x = 0.2
      cmd_pub.publish(msg)
      rate.sleep()
    else:
      print(move_dis)
      break
  msg = Twist()
  cmd_pub.publish(msg)
  rate.sleep()  
  print("end loop")

def rotate():
  cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size = 10)
  rate = rospy.Rate(10)
  
  t = rospy.get_time()
  while rospy.get_time() -t < 5:
    msg = Twist()
    msg.angular.z = -0.36
    cmd_pub.publish(msg)
    rate.sleep() 
  print("end rotate")

def L():
  global canLand
  land_sub = rospy.Subscriber('/tello/status', TelloStatus, ts_callback)
  land_pub = rospy.Publisher('/tello/land', Empty, queue_size = 1)
  rate = rospy.Rate(10)
  
  while canLand is not True:
    msg = Empty()
    land_pub.publish(msg)
    rate.sleep()

if __name__ == '__main__':
    rospy.init_node('tello_net_pub', anonymous=True)
    TO()
    sleep(3)
    cmd()
    sleep(3)
    rotate()
    sleep(3)
    cmd()
    sleep(3)
    L()
    sleep(3)
    sys.exit(0)
