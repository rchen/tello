#!/usr/bin/env python
import rospy
import roslib
import sys
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, UInt8
from tello_driver.msg import TelloStatus, test
from statemachine import StateMachine, State
from time import sleep

class sMachine(StateMachine):
    takeoff = State('Takeoff', initial = True)
    hover = State('Hover')
    correction = State('Correction')
    forward = State('Forward')
    addSp = State('AddSp')

    to_takeoff = takeoff.to(takeoff)
    to_hover = hover.to(hover) | addSp.to(hover) | correction.to(hover) | forward.to(hover) | takeoff.to(hover)
    to_correction = hover.to(correction) | correction.to(correction) | forward.to(correction)
    to_forward = hover.to(forward) | correction.to(forward) | forward.to(forward)
    to_addSp = forward.to(addSp) | correction.to(addSp)
    
class MyModel(object):
    def __init__(self, state):
        self.state = state
        self.target = (-1,-1,1)
        self.center = (480, 180)
        self.check = False
        self.canLand = False
        self.rec_time = 0
        self.self_pub = rospy.Subscriber('/selfDefined', test, self.cback)
        self.cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size = 10)
        self.land_sub = rospy.Subscriber('/tello/status', TelloStatus, self.ts_callback)
        self.land_pub = rospy.Publisher('/tello/land', Empty, queue_size = 1)
        self.change_pub = rospy.Publisher('/selfChanged', Empty, queue_size = 10)
        self.rate = rospy.Rate(10)
        self.last_state = None
        # takeoff
        self.takeoff_pub = None
        self.isTakeoff = False

    def cback(self, data):
        self.rec_time = rospy.get_time()
        self.target = data.point
        
    def ts_callback(self, data):
        if data.fly_mode == 12:
            self.canLand = True
        elif data.fly_mode == 6:
            self.isTakeoff = True
            
    def takeoff(self, fsm):
        if self.takeoff_pub == None:
            self.takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size = 1)
            while self.takeoff_pub.get_num_connections() == 0:
                pass
            print('gogo')
            self.rate.sleep()
            # takeoff
            msg = Empty()
            self.takeoff_pub.publish(msg)
        if self.isTakeoff:
            fsm.to_hover()
        self.rate.sleep()
        
    def run(self, fsm):
        #print(type(self))
        while not rospy.is_shutdown():
            pass
            if self.last_state == None:
                self.last_state = self.state
                print(self.last_state)
            else:
                if self.last_state != self.state:
                    self.last_state = self.state
                    print(self.last_state)
            if fsm.is_takeoff:
                self.takeoff(fsm)
            elif fsm.is_hover:
                self.cmd_pub.publish(Twist())
                self.rate.sleep()
                if (self.target[0] == -1 and self.target[1] == -1) or (rospy.get_time() - self.rec_time > 1.0):
                    fsm.to_hover()
                elif abs(self.target[0] - self.center[0]) >= 60 or abs(self.target[1] - self.center[1]) >= 30:
                    fsm.to_correction()
                elif abs(self.target[0] - self.center[0]) < 60 and abs(self.target[1] - self.center[1]) < 30:
                    fsm.to_forward()
            elif fsm.is_correction:
                msg = Twist()
                if abs(self.target[0] - self.center[0]) >= 60:
                    msg.linear.y = -(self.target[0] - self.center[0]) / abs((self.target[0] - self.center[0])) * 0.1
                if abs(self.target[1] - self.center[1]) >= 30:
                    msg.linear.z = -(self.target[1] - self.center[1]) / abs((self.target[1] - self.center[1])) * 0.2
                self.cmd_pub.publish(msg)
                self.rate.sleep()
                if rospy.get_time() - self.rec_time > 1.0:
                    fsm.to_hover()
                elif self.target[2] == -1:
                    fsm.to_addSp()
                elif abs(self.target[0] - self.center[0]) >= 60 or abs(self.target[1] - self.center[1]) >= 30:
                    fsm.to_correction()
                elif abs(self.target[0] - self.center[0] < 60) and abs(self.target[1] - self.center[1] < 30):
                    fsm.to_forward()
            elif fsm.is_forward:
                msg = Twist()
                msg.linear.x = 0.2
                self.cmd_pub.publish(msg)
                self.rate.sleep()
                if rospy.get_time() - self.rec_time > 1.0:
                    fsm.to_hover()
                elif self.target[2] == -1:
                    fsm.to_addSp()
                elif abs(self.target[0] - self.center[0]) >= 60 or abs(self.target[1] - self.center[1]) >= 30:
                    fsm.to_correction()
                elif abs(self.target[0] - self.center[0] < 60) and abs(self.target[1] - self.center[1] < 30):
                    fsm.to_forward()
            elif fsm.is_addSp:
                msg = Twist()
                msg.linear.x = 0.2
                #msg.linear.z = 0.2
                self.cmd_pub.publish(msg)
                self.rate.sleep()
                sleep(2)
                msg = Twist()
                msg.linear.x = 0.24 
                #msg.linear.z = 0.2
                self.cmd_pub.publish(msg)
                self.rate.sleep()
                sleep(2)
                self.change_pub.publish(Empty())
                self.rate.sleep()
                msg = Twist()
                self.cmd_pub.publish(msg)
                self.rate.sleep()
                fsm.to_hover()

def L():
  global canLand

  rate = rospy.Rate(10)
  
  while canLand is not True:
    msg = Empty()
    land_pub.publish(msg)
    rate.sleep()

if __name__ == '__main__':
    rospy.init_node('h264_pub', anonymous=True)
    obj = MyModel(state='takeoff')
    fsm = sMachine(obj)
    obj.run(fsm)

