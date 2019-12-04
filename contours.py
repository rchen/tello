#! /usr/bin/env python
import rospy
import cv2
import numpy as np
import time
from test_h264_sub import StandaloneVideoStream
from h264_image_transport.msg import H264Packet
import av
from tello_driver.msg import test

class Contours(object):
    def __init__(self):
        self.stream = StandaloneVideoStream()

    def start(self):
        # fourcc = cv2.VideoWriter_fourcc('X', 'V', 'I', 'D')
        # out = cv2.VideoWriter('/home/grant/test.avi', fourcc, 30.0, (1920, 720))
        
        rospy.Subscriber("/tello/image_raw/h264", H264Packet, self.callback)
        pub = rospy.Publisher('/selfDefined', test, queue_size = 1)
        rospy.loginfo('stream start')
        container = av.open(self.stream)
        rospy.loginfo('main: opened')

        frame_skip = 300

        for frame in container.decode(video=0):
            if 0 < frame_skip:
                frame_skip -= 1
                continue
            start_time = time.time()
            image = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)
            blurred_img = cv2.GaussianBlur(image, (13, 13), 0)
            hsv_img = cv2.cvtColor(blurred_img.copy(), cv2.COLOR_BGR2HSV)
            red_mask = self.findMask(hsv_img)
            (contour_i, contour_contours, contour_h) = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            show_image= cv2.cvtColor(contour_i, cv2.COLOR_GRAY2BGR)
            cv2.drawContours(show_image, contour_contours, -1, (0,0,255), -1)

            out_max_contours = max(contour_contours, key = cv2.contourArea)
            rect = cv2.minAreaRect(out_max_contours)
            rect_width, rect_height = rect[1]
            ce_x = rect[0][0] + 1/2*rect_width
            ce_y = rect[0][1] + 1/2*rect_height
            old_center = [int(ce_x), int(ce_y)]
            if old_center[0] == 0 and old_center[1] == 0:
                pub.publish(test([int(old_center[0]), int(old_center[1]), 1]))
            else:
                # cv2.putText(show_image, str(rect_width * rect_height, (10, 40), 5,2, 255))
                if rect_width * rect_height > 960 * 720 * 0.25:
                    pub.publish(test([int(old_center[0]), int(old_center[1]), -1]))
                    print('Quit')
                    # rospy.signal_shutdown('Quit')
                else:
                    pub.publish(test([int(old_center[0]), int(old_center[1]), 1]))                    

            # out.write(np.concatenate((blurred_img, show_image), axis=1))
            # cv2.imshow('result', np.concatenate((blurred_img, show_image), axis=1))
            cv2.waitKey(1)
            if frame.time_base < 1.0/60:
                time_base = 1.0 / 60
            else:
                time_base = frame.time_base
            frame_skip = int((time.time() - start_time)/time_base)
    
    def callback(self, msg):
        self.stream.add_frame(msg.data)

    def findMask(self, hsv_img):
      lower_red_0 = np.array([0, 70, 0]) 
      upper_red_0 = np.array([5, 255, 255])
      lower_red_1 = np.array([175, 70, 0]) 
      upper_red_1 = np.array([180, 255, 255])
      red_mask0 = cv2.inRange(hsv_img, lower_red_0, upper_red_0)
      red_mask1 = cv2.inRange(hsv_img, lower_red_1, upper_red_1)
      red_mask = cv2.bitwise_or(red_mask0, red_mask1) 
      return red_mask

    
if __name__ == '__main__':
    # try:
    rospy.init_node('h264_listener')
    Contours().start()        
    # except BaseException:
    #     traceback.print_exc()
    # finally:
    #     stream.close()
    #     cv2.destroyAllWindows()
