#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic


import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
# import pickle
import cv2
import base64

scale_percent = 50

def pub_to_web(data):
    image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    dim = (width, height)
    image = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
    image = cv2.rotate(image, cv2.cv2.ROTATE_90_CLOCKWISE)
    image = cv2.flip(image, 0)
    _, buffer = cv2.imencode('.jpg', image)
    image_as_str = base64.b64encode(buffer).decode('utf-8')
    compressed_img_publisheer.publish(image_as_str)

def listener():

    global compressed_img_publisheer
    compressed_img_publisheer = rospy.Publisher('hockey_robot/cam_pub/rgb', String, queue_size=10)
    rospy.init_node('cam_pub', anonymous = True)
    # global pred_publisher

    rospy.Subscriber('/hockey_robot/camera1/image_raw', Image, pub_to_web)
    # spin() simply keeps python from exiting until this node is stopped
    
    rospy.spin()

if __name__ == '__main__':
    # f = open("demofile3.txt", "w")
    # save_dir = '/home/futong/catkin_ws/src/air_hockey_robot/hockey_robot_gazebo/scripts'
    bridge = CvBridge()
    listener()
    # f.close()