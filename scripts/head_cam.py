#!/usr/bin/python2

import argparse
import socket
import sys

import rospy
import rosgraph

import std_srvs.srv

from baxter_core_msgs.srv import (
    ListCameras,
)
from baxter_interface.camera import CameraController


def open_camera(camera, res, *_args, **_kwds):
    cam = CameraController(camera)
    cam.close()
    cam.resolution = res
    cam.open()


def close_camera(camera, *_args, **_kwds):
    cam = CameraController(camera)
    cam.close()

if __name__ == '__main__':
    rospy.init_node('head_camera')
    close_camera("left_hand_camera", None)
    close_camera("right_hand_camera", None)
    open_camera("head_camera", [1280,800])
    sys.exit(0)
