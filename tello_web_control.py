#!/usr/bin/env python

import rospy
import roslib
import sys
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, UInt8
from tello_driver.msg import TelloStatus
from time import sleep
from contours import Contours
from tello_driver.msg import test

def flip(value):
    rate = rospy.Rate(1)
    pub = rospy.Publisher('/tello/flip', UInt8, queue_size = 1)

    while pub.get_num_connections() == 0:
        pass

    msg = UInt8()
    msg.data = value
    pub.publish(msg)
    rate.sleep()

global canLand, hori_speed, move_dis, last_time
canLand = False
hori_speed = 0
move_dis = 0
last_time = 0.0

global isTakeoff
isTakeoff = False

def takeoff_callback(data):
    global isTakeoff
    if not isTakeoff and data.is_flying == True:
        isTakeoff = True
        
def takeoff():
    takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size = 1)

    while takeoff_pub.get_num_connections() == 0:
        pass
    rate = rospy.Rate(10)
    # takeoff
    msg = Empty()
    rospy.loginfo(msg)
    takeoff_pub.publish(msg)
    rate.sleep()

    global isTakeoff
    # takeoff callback
    land_sub = rospy.Subscriber('/tello/status', TelloStatus, takeoff_callback)

    cnt = 0
    while not isTakeoff:
        if cnt == 50:
            rospy.loginfo('Takeoff fail')
            # sys.exit(0)
            break
        rate.sleep()
        cnt += 1
    land_sub.unregister()
    print("Takeoff")  

def land():
    global canLand
    land_sub = rospy.Subscriber('/tello/status', TelloStatus, ts_callback)
    land_pub = rospy.Publisher('/tello/land', Empty, queue_size = 1)
    rate = rospy.Rate(1)
  
    while canLand is not True:
        print("Land")    
        msg = Empty()
        land_pub.publish(msg)
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
# guard height        
global isHeight, guard_max_height, guard_min_height
isHeight = False
guard_max_height = 1
guard_min_height = 0
# forward
global forward_move, forward_timeout
forward_move = False
forward_timeout = 20

def cmd_vel_callback(data):
    global forward_move, guard_max_height, guard_min_height, isHeight
    if data.cmd_pitch_ratio > 0:
        forward_move = True
    # check height
    # rospy.loginfo(guard_max_height)
    # rospy.loginfo(guard_min_height)
    # rospy.loginfo(isHeight)
    if guard_max_height != 1 and guard_max_height < data.height_m:        
        isHeight = True
    elif guard_min_height != 0 and guard_min_height > data.height_m:
        isHeight = True
    
def cmd_vel(times, twist, height = None):
    cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size = 10)
    rate = rospy.Rate(10)

    # check move
    global forward_move, forward_timeout, isHeight, guard_max_height, guard_min_height
    if height:
        if twist.linear.z > 0:
            guard_max_height = height
        elif twist.linear.z < 0:
            guard_min_height = height
        
    land_sub = rospy.Subscriber('/tello/status', TelloStatus, cmd_vel_callback)
    cnt = 0
    while not forward_move:
        if cnt > forward_timeout:
            break
        cmd_pub.publish(twist)
        rate.sleep()
        cnt += 1
    # is move
        
    cnt = 0
    while cnt < times:
        if isHeight:
            rospy.loginfo('guard height')
            break
        rate.sleep()
        cnt += 1

    land_sub.unregister()
    msg = Twist()
    cmd_pub.publish(msg)
    rate.sleep()      
    guard_max_height = 1
    guard_min_height = 0
    isHeight = False
    forward_move = False
    forward_timeout = 100

    print('end loop')
  
def rotate(times):
    rospy.loginfo('start rotate')
    cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size = 10)
    rate = rospy.Rate(10)
  
    cnt = 0
    while not rospy.is_shutdown():
        if cnt < times:      
            msg = Twist()
            msg.angular.z = -1
            cmd_pub.publish(msg)
            rate.sleep()
            cnt += 1
        else:
            break
    msg = Twist()
    msg.angular.z = 0
    cmd_pub.publish(msg)
    rate.sleep()
    print("end rotate")

global status_time
status_time = 0.0
def_get_status_sec = 0.5    

def status_callback(data):

    # 0.5 sec to get status
    global status_time
    if status_time == 0.0:
        status_time = rospy.get_time()        
    now = rospy.get_time()
        
    if now - status_time > def_get_status_sec:
        status_time = now

        # print log
        rospy.loginfo(data.battery_percentage)
        rospy.loginfo(data.height_m)
        
        # print("cmd_vspeed_ratio: %s", data.cmd_vspeed_ratio)        
        # print("cmd_fast_mode: %s", data.cmd_fast_mode)

        # is_battery_low
        if data.is_battery_low:
            print('battery_low')

def work1():
    land_sub = rospy.Subscriber('/tello/status', TelloStatus, status_callback)
    # takeoff
    takeoff()
    
    # foward down
    twist = Twist()
    twist.linear.x = 0.1
    twist.linear.z = -0.3
    cmd_vel(80, twist, 0.5)

    # forward up
    twist = Twist()
    twist.linear.x = 0.1
    twist.linear.z = 0.3
    cmd_vel(80, twist, 0.7)

    # #capture
    # capture = Capture('/home/grant/img')
    # capture.run()

    # rotate
    twist = Twist()
    twist.angular.z = -0.3
    cmd_vel(37, twist)

    # foward down
    twist = Twist()
    twist.linear.x = 0.1
    twist.linear.z = -0.5
    cmd_vel(80, twist, 0.3)

    # forward up
    twist = Twist()
    twist.linear.x = 0.1
    twist.linear.z = 0.5
    cmd_vel(80, twist, 0.7)

    # rotate
    twist = Twist()
    twist.angular.z = 0.3
    cmd_vel(43, twist)

    # flip
    flip(0)

    # forward
    twist = Twist()
    twist.linear.x = 0.1   
    cmd_vel(20, twist)

    #land
    land()

cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size = 10)


global check
check = False
global detectCnt
detectCnt = 0
global rect_time
rect_time = 0
global land_time
land_time = -1

global isGo
isGo = False
global insideCnt
insideCnt = 1

def callback(data):
    global isGo
    global insideCnt

    if not isTakeoff:
        return
    global land_time
    if land_time != -1:
        if (rospy.get_time() - land_time) > 100.0:
            print('time out land')
            msg = Twist()
            cmd_pub.publish(msg)
            land()
    else:
        land_time = rospy.get_time()
        
    global rect_time
    if (rect_time == 0):
        rect_time = rospy.get_time()
            
    rate = rospy.Rate(10)
    global check
    target = data.point
    center = (480, 240)
    dx = target[0] - center[0]
    dy = target[1] - center[1]
    
    global detectCnt
    if detectCnt > 1:
        detectCnt = 0
    else:
        detectCnt += 1
        return
    print('data:', data)
    print('dxdy:', dx, dy)

    if not isGo:        
        if rospy.get_time() - rect_time > 1.0:
            rect_time = rospy.get_time()
            print('move: empty')
            msg = Twist()
            cmd_pub.publish(msg)
        else:
            msg = Twist()
            
            if abs(dx) < 30 and abs(dy) < 30:
                print('inside')

                if target[2] == -1:
                    if insideCnt > 4:
                        msg = Twist()
                        cmd_pub.publish(msg)
                        rate.sleep()
                        isGo = True
                        print('isGo')
                        rate.sleep()
                        return
                    else:
                        insideCnt += 1
                        return
                else:
                    msg.linear.x = 0.1
            else:# abs(dx) >= 20 or abs(dy) >= 20:
                insideCnt = 1
                print('out')
                rateY = 1
                if abs(dx) > 150:
                    rateY = 2

                rateZ = 1
                if abs(dy) > 150:
                    rateZ = 1
                    
                if abs(dx) >= 45:
                    y = -dx / abs(dx) * 0.1 * rateY
                    msg.linear.y = y
                if abs(dy) >= 30:
                    z = -dy / abs(dy) * 0.2 * rateZ
                    msg.linear.z = z
                if target[2] != -1:
                    msg.linear.x = 0.1
                else:
                    if abs(dx) >= 45 or abs(dy) > 30:
                        msg.linear.x = -0.1

            print('move: x: %s, y: %s, z: %s', msg.linear.x, msg.linear.y, msg.linear.z)
            cmd_pub.publish(msg)
    print("")
    rate.sleep()

    
if __name__ == '__main__':
    rospy.init_node('tello_net_pub', anonymous=True)    
    # Contours().start()
    takeoff()
    pub = rospy.Subscriber('/selfDefined', test, callback)
    
    while(not isGo):
        pass
    
    # forward
    twist = Twist()
    twist.linear.x = 0.2
    twist.linear.z = -0.1    
    cmd_vel(70, twist)
    print('end land')
    land()
    sys.exit(0)
