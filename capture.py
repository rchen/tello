#!/usr/bin/env python
import rospy
from h264_image_transport.msg import H264Packet
import av
import threading
from test_h264_sub import StandaloneVideoStream
import cv2
from create_file import CreateFile

class Capture(object):

    def __init__(self, folder):
        self.stream = StandaloneVideoStream()
        cf = CreateFile(folder)
        self.path = cf.getPath()
        
    def run(self):
        rospy.Subscriber("/tello/image_raw/h264", H264Packet, self.callback)
        container = av.open(self.stream)
        rospy.loginfo('start capture')
        cnt = 0
        for frame in container.decode(video=0):
            self.image = frame.to_image()
            if self.image:
                break
            else:
                pass

        self.stream.close()
        self.image.save(self.path, "JPEG")
        rospy.loginfo('end capture')

            
    def callback(self, msg):
        # rospy.loginfo('frame: %d bytes' % len(msg.data))
        self.stream.add_frame(msg.data)
