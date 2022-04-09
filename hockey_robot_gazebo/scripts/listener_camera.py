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

import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

import json
# import pickle
import cv2
# from .planner import plan_strategy
microsecond_between_each_frame = 200


class predicter():
    def __init__(self):
        self.time_stamp = 0
        self.ball_location_y = 0
        self.ball_location_x = 0
        self.last_time_stamp = 0
        self.last_location_y = 0
        self.last_location_x = 0
        self.prediction = {}
        self.mistake_range = 2

        #for now i don't know the a number, so the result must be wrong
        self.y_top = 310.5
        self.y_down = 704.5
        self.x_speed = 0
        self.y_speed = 0
        self.x_pending = 0
        self.x_pending = 0
        self.time_range = 30
        #x = x_speed*t + x_pending+a*t*t/2
        #y = y_speed*t + y_pending+a*t*t/2

    def set_current_status(self,x,y,time_stamp):
        if self.last_time_stamp == time_stamp:
            return self.prediction
        print("start_setting_points")
        self.last_time_stamp = self.time_stamp
        self.time_stamp = int(time_stamp)
        self.last_location_x = self.ball_location_x
        self.last_location_y = self.ball_location_y
        self.ball_location_x = x
        self.ball_location_y = y
        return self.predict()
    
    def build_new_predict(self):
        self.x_speed = (self.ball_location_x - self.last_location_x)/(self.time_stamp-self.last_time_stamp)
        self.x_pending = self.ball_location_x - self.x_speed*self.time_stamp
        self.y_speed = (self.ball_location_y - self.last_location_y)/(self.time_stamp-self.last_time_stamp)
        self.y_pending = self.ball_location_y - self.y_speed*self.time_stamp
        self.prediction = {self.time_stamp:[float(self.ball_location_x), float(self.ball_location_y)]}
        self.continue_predict()

    def regular_y(self,y):
        while self.y_top > y or self.y_down < y:
            if y > self.y_down:
                y = 2*self.y_down - y
            elif y < self.y_top:
                y = 2*self.y_top - y
        return y

    def continue_predict(self):
        predicted = len(self.prediction.keys())
        while predicted <= self.time_range:
            target_time_stamp = self.time_stamp + predicted*microsecond_between_each_frame
            self.prediction[target_time_stamp] = [self.prediction[self.time_stamp][0]+self.x_speed*predicted*microsecond_between_each_frame,self.regular_y(self.prediction[self.time_stamp][1]+self.y_speed*predicted*microsecond_between_each_frame)]
            predicted+=1
    
    def predict(self):
        if self.time_stamp in self.prediction.keys():
            if abs(self.ball_location_x - self.prediction[self.time_stamp][0]) <= self.mistake_range and abs(self.ball_location_y - self.prediction[self.time_stamp][1]) <= self.mistake_range:
                for time_key in list(self.prediction):
                    if time_key < self.time_stamp:
                        self.prediction.pop(time_key)
                if len(self.prediction.keys()) >= self.time_range*2/3:
                    return self.prediction
                else:
                    self.continue_predict()
                    return self.prediction
        self.build_new_predict()
        return self.prediction


default_predicter = predicter()
                

def detect_coordinates_of_red_balls(img):
    img = cv2.rotate(img, cv2.ROTATE_180)
    # filename = '{}{}.jpg'.format(save_dir,0)
    # cv2.imwrite(filename, img)
    img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(img_hsv,lower_red, upper_red)

    lower_red = np.array([170,50,50])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(img_hsv,lower_red, upper_red)

    mask = mask0+mask1
    #1 locate the ball
    img[np.where(mask==0)] = 0
    img[444:575,101:125,:] = 0
    img[444:575,890:925,:] = 0
    img = cv2.medianBlur(img,15)
    img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(img_gray,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=15,maxRadius=25)
    # print(circles)
    if circles is not None:
        x = circles[0][0][0]
        y = circles[0][0][1]
        return True,x,y
    else:
        return False,0,0

def convert_image_coordinate_into_actual(prediction):
    for coordinates in prediction.values():
        x = coordinates[0]
        y = coordinates[1]
        coordinates[0] = 0.0024571*x - 0.3931429
        coordinates[1] = 0.0025445*y - 1.3066175
    return prediction

def callback(data : Image):
    # print(data.header.stamp)
    current_time = int(time.time()*100)
    if current_time % microsecond_between_each_frame <= microsecond_between_each_frame//10:
        current_time = microsecond_between_each_frame*(current_time//microsecond_between_each_frame)
        print(current_time)
        img = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        # filename = '{}{}.jpg'.format(save_dir,data.header.seq)
        # cv2.imwrite(filename, img)
        # # img = cv2.
        # filename = '{}{}.jpg'.format(save_dir,data.header.seq)
        # cv2.imwrite(filename, img)
        detected,x,y = detect_coordinates_of_red_balls(img)
        if not detected:
            return
        # print(data.header.seq)
        prediction = default_predicter.set_current_status(x,y,current_time)
        # # pred_res = str(pickle.dumps(prediction))
        # pred_res = json.dumps(prediction)
        # pred_publisher.publish(pred_res)
        prediction = convert_image_coordinate_into_actual(prediction)
        print(prediction)
        # plan_strategy(prediction)
        #here suppose to call the actuall function of policy
        return
    
    # rospy.loginfo(rospy.get_caller_id() + 'position: %s', data.position)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous = True)
    # global pred_publisher

    # rospy.init_node('path_predictor', anonymous=True)
    # pred_publisher = rospy.Publisher('/hockey_robot/predicter/pred_res', String, queue_size=10)
    rospy.Subscriber('/hockey_robot/camera1/image_raw', Image, callback)
    # spin() simply keeps python from exiting until this node is stopped
    
    rospy.spin()

if __name__ == '__main__':
    # f = open("demofile3.txt", "w")
    # save_dir = '/home/futong/catkin_ws/src/air_hockey_robot/hockey_robot_gazebo/scripts'
    bridge = CvBridge()
    listener()
    # f.close()