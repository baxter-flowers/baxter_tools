#!/usr/bin/python2

import sys, cv2, datetime, time
import rospy, cv_bridge

from copy import deepcopy
from baxter_interface.camera import CameraController
from sensor_msgs.msg import Image
from threading import Lock

class BaxterVideoRecorder():
    def open_camera(self, camera, x_res, y_res):
        if camera == "left":
            cam = CameraController("left_hand_camera")
        elif camera == "right":
            cam = CameraController("right_hand_camera")
        elif camera == "head":
            cam = CameraController("head_camera")
        else:
            sys.exit("ERROR - open_camera - Invalid camera")

        # set camera parameters
        cam.resolution          = (int(x_res), int(y_res))
        cam.exposure            = -1             # range, 0-100 auto = -1
        cam.gain                = -1             # range, 0-79 auto = -1
        cam.white_balance_blue  = -1             # range 0-4095, auto = -1
        cam.white_balance_green = -1             # range 0-4095, auto = -1
        cam.white_balance_red   = -1             # range 0-4095, auto = -1
        cam.open()

    def close_camera(self, camera):
        if camera == "left":
            cam = CameraController("left_hand_camera")
        elif camera == "right":
            cam = CameraController("right_hand_camera")
        elif camera == "head":
            cam = CameraController("head_camera")
        else:
            sys.exit("ERROR - close_camera - Invalid camera")
        cam.close()

    def left_handler(self, image):
        cv_image = self.cv_bridge.imgmsg_to_cv2(image)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)  # Remove depth
        if self.recording:
            self.left_writer.write(cv_image)
        with self.lock_cv:
            cv2.imshow("Left image", cv_image)
        cv2.waitKey(1)

    def right_handler(self, image):
        image = deepcopy(image)
        cv_image = self.cv_bridge.imgmsg_to_cv2(image)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)  # Remove depth
        if self.recording:
            self.right_writer.write(cv_image)
        with self.lock_cv:
            cv2.imshow("Right image", cv_image)
        cv2.waitKey(1)

    def __init__(self, width, height):
        self.recording = False
        self.lock_cv = Lock() # openCV has issues with concurrent imshow

        # Tricky behaviour: 2 cameras are always kept opened with L/R powered but not opened by default
        # Closing a closed camera raises an unclear error, so let's catch a possible failure during closure
        # https://groups.google.com/a/rethinkrobotics.com/d/msg/brr-users/VPqJDkXTVzE/4uk93xRmtQwJ

        print "Closing head cam..."
        try:
            self.close_camera("head")
        except AttributeError:
            print "Probably already closed"
        print "Opening left cam..."
        self.open_camera("left", width, height)
        print "Opening right cam..."
        self.open_camera("right", width, height)

        self.left_writer = None
        self.right_writer = None

        self.str_now = str(datetime.datetime.now()).split('.')[0].replace(' ', '_')
        self.cv_bridge = cv_bridge.CvBridge()

        rospy.Subscriber('/cameras/left_hand_camera/image', Image, self.left_handler)
        rospy.Subscriber('/cameras/right_hand_camera/image', Image, self.right_handler)

        fourcc = cv2.cv.CV_FOURCC(*'fmp4')
        self.left_writer = cv2.VideoWriter(self.str_now+'_left.avi', fourcc, 20, (width, height))
        self.right_writer = cv2.VideoWriter(self.str_now+'_right.avi', fourcc, 20, (width, height))

        raw_input('Ready to record, press Enter')
        self.recording = True
        raw_input('Recording... press Enter to stop')
        self.recording = False

        self.left_writer.release()
        self.right_writer.release()

if __name__ == '__main__':
    rospy.init_node('lr_camera_recorder')
    BaxterVideoRecorder(1280, 800)
