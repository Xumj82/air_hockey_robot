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
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

import json
# import pickle
import cv2
import sys
import message_filters
from sensor_msgs.msg import JointState
# #print(sys.path)
# sys.path.append('scripts')
# from planner import plan_strategy


### global config
HOCKEY_PATH_LENGTH = 4

STRATEGY_3_SAFE_LINE = 0.4
STRATEGY_3_ATTACK_LINE = 0.2
STRATEGY_3_PREPARE_RANGE = 5

microsecond_between_each_frame = 2

allowed_path_mistake = 1

default_strategy = 3
tracker_pub = rospy.Publisher('hockey_robot/joint1_position_controller/command', Float64, queue_size=10)
pusher_pub = rospy.Publisher('hockey_robot/joint2_position_controller/command', Float64, queue_size=10)

## region

## Planner

class planner():
    def __init__(self):
        self.predict_data = []
        self.strategy = default_strategy
        self.current_target = []

        self.hockey_path = [0,0,0,0]

        self.strategy_1_defence_position = 0
        self.strategy_2_defence_position_x = 0
        self.strategy_2_defence_position_y = 0

        self.strategy_3_defence_position_x = 0
        self.strategy_3_defence_position_y = 0

    #basic tools for operating ros    
    def setPusher1_position(self,data):
        pusher_pub.publish(data)

    def setTrack1_position(self,data):
        tracker_pub.publish(data)

    def reset_position(self):
        pusher_pub.publish(0)
        tracker_pub.publish(0)
    
    #stategy check and set

    def check_strategy_1(self):
        hockey_position_list = list(self.predict_data.values())
        if hockey_position_list[-1][0]<0:
            return True
        elif hockey_position_list[-1][0]>0:
            return False
    def strategy_1(self):
        #print(1)
        hockey_position_list = list(self.predict_data.values())
        for i in range(len(hockey_position_list)-1):
            if hockey_position_list[i][0] >0 and hockey_position_list[i+1][0] <=0 :
                self.strategy_1_defence_position = (hockey_position_list[i][1] + hockey_position_list[i+1][1])/2
                #print('go to defence position (%0.2f,%0.2f)'%(0,self.strategy_1_defence_position))
                self.setPusher1_position(self.strategy_1_defence_position)
                self.setTrack1_position(0)
    def strategy_2(self):
        #print(2)
        hockey_pos = list(self.predict_data.values())
        self.strategy_2_defence_position_x,self.strategy_2_defence_position_y = hockey_pos[0][0],hockey_pos[0][1]
        self.setPusher1_position(self.strategy_2_defence_position_y)
        self.setTrack1_position(self.strategy_2_defence_position_x-0.05)
        # for pre_time,hockey_pos in self.predict_data.items():
        #     if hockey_pos[0]<=0.86:
        #         self.strategy_2_defence_position_x,self.strategy_2_defence_position_y = hockey_pos[0],hockey_pos[1]
        #         #print('go to defence position (%0.2f,%0.2f)'%(self.strategy_2_defence_position_x,self.strategy_2_defence_position_y))
        #         self.setPusher1_position(self.strategy_2_defence_position_y)
        #         self.setTrack1_position(self.strategy_2_defence_position_x-0.05)


    def strategy_3(self,same_path):
        #print(3)
        if not same_path:
            hockey_position_list = list(self.predict_data.values())
            for i in range(len(hockey_position_list)-1):
                if hockey_position_list[i][0] > STRATEGY_3_SAFE_LINE and hockey_position_list[i+1][0] <= STRATEGY_3_SAFE_LINE:
                    self.strategy_3_defence_position = (hockey_position_list[i][1] + hockey_position_list[i+1][1])/2
                    if i <= STRATEGY_3_PREPARE_RANGE:
                        self.strategy_1()
                        return
                    else:
                        #print('go to prepare position (%0.2f,%0.2f)'%(0,self.strategy_3_defence_position))
                        self.setPusher1_position(self.strategy_3_defence_position)
                        self.setTrack1_position(STRATEGY_3_ATTACK_LINE)
                        return
            return
        else:
            hockey_position_list = list(self.predict_data.values())
            for i in range(len(hockey_position_list)-1):
                if hockey_position_list[i][0] > STRATEGY_3_SAFE_LINE and hockey_position_list[i+1][0] <= STRATEGY_3_SAFE_LINE:
                    if i <= STRATEGY_3_PREPARE_RANGE:
                        #x/(x+0.03)=(y/y+?) ## x == (0.86*2-STRATEGY_3_SAFE_LINE), y+? == self.strategy_3_attact_position, y == self.strategy_3_defence_position
                        self.strategy_3_attact_position = self.strategy_3_defence_position + 0.03*self.strategy_3_defence_position/(0.86*2-STRATEGY_3_SAFE_LINE)
                        self.setPusher1_position(self.strategy_3_attact_position)
                        self.setTrack1_position(STRATEGY_3_SAFE_LINE+0.05)
                        return
                    else:
                        return
    # def strategy_4(self,same_path,Jointinfo):
    #     prefer = 0
    #     if Jointinfo[1] < 0:
    #         prefer = -0.05
    #     elif Jointinfo[1] > 0:
    #         prefer = 0.05
    #     if not same_path:
    #         hockey_position_list = list(self.predict_data.values())
    #         for i in range(len(hockey_position_list)-1):
    #             if hockey_position_list[i][0] > STRATEGY_3_SAFE_LINE and hockey_position_list[i+1][0] <= STRATEGY_3_SAFE_LINE:
    #                 self.strategy_3_defence_position = (hockey_position_list[i][1] + hockey_position_list[i+1][1])/2
    #                 if i <= STRATEGY_3_PREPARE_RANGE:
    #                     self.strategy_1()
    #                     return
    #                 else:
    #                     #print('go to prepare position (%0.2f,%0.2f)'%(0,self.strategy_3_defence_position))
    #                     self.setPusher1_position(self.strategy_3_defence_position+prefer)
    #                     self.setTrack1_position(STRATEGY_3_ATTACK_LINE-0.05)
    #                     return
    #         return
    #     else:
    #         hockey_position_list = list(self.predict_data.values())
    #         for i in range(len(hockey_position_list)-1):
    #             if hockey_position_list[i][0] > STRATEGY_3_SAFE_LINE and hockey_position_list[i+1][0] <= STRATEGY_3_SAFE_LINE:
    #                 if i <= STRATEGY_3_PREPARE_RANGE:
    #                     self.setPusher1_position(self.strategy_3_defence_position+prefer)
    #                     self.setTrack1_position(STRATEGY_3_SAFE_LINE-0.05)
    #                     return
    #                 else:
    #                     return

    def check_hockey_static(self):
        hockey_position_list = list(self.predict_data.values())
        hockey_x_list=[]
        for i in hockey_position_list:
            hockey_x_list.append(i[0])
        move_dis=0
        for i in range(1,len(hockey_x_list)):
            move_dis += abs(hockey_x_list[i]-hockey_x_list[i-1])
        if move_dis <=0.1:
            if hockey_position_list[0][0] >= 0.86:
                return False
            return True
        return False

    def check_hockey_direction(self):
        hockey_position_list = self.predict_data.values()
        hockey_x_list=[]
        for i in hockey_position_list:
            hockey_x_list.append(i[0])
        if len(hockey_x_list) == 0:
            return False
        if hockey_x_list[-1]>hockey_x_list[0]:
            return False
        return True
    
    def update_hockey_path(self,hockey_path):
        #print("******************************")
        #print(hockey_path)
        if len(hockey_path) != HOCKEY_PATH_LENGTH:
            return False
        path_difference = 0
        for i in range(2):
            path_difference += abs(self.hockey_path[i]-hockey_path[i])
        if path_difference <= allowed_path_mistake:
            return True
        else:
            self.hockey_path = hockey_path
            return False
    
    def update_prediction(self,prediction,hockey_path,Jointinfo = None):
        self.predict_data = prediction
        same_path = self.update_hockey_path(hockey_path)
        self.plan_strategy(same_path,Jointinfo)
    def plan_strategy(self,same_path,Jointinfo = None):

        #step 1:check if need plan:
        # try:
        #print('plan start')
        if not self.check_hockey_direction():
            #print('No need to plan')
            self.reset_position()
            return
        
        if self.check_hockey_static():
            #print('strategy1 is not suit')
            #check_hockey_position:
            self.strategy = 2
            #print('plan start strategy_2')
            self.strategy_2()
            return
        else:
            self.strategy = default_strategy
        
        #step 2:strategy choice:
        # while not rospy.is_shutdown():

        if self.strategy <= 1:
            if same_path:
                return
            if self.check_strategy_1():
                #print('plan start strategy_1')
                self.strategy_1()
                return
            else:
                # strategy_1 can not use
                #print ('Error:Defence strategy(strategy1) is not suit')
                return
        # elif self.strategy  == 2:
        #     #print('plan start strategy_2')
        #     self.strategy_2()
        #     return
        elif self.strategy == 3:
            #print('plan start strategy_3')
            self.strategy_3(same_path)
            return
        elif self.strategy == 4:
            #print('plan start strategy_4')
            self.strategy_4(same_path,Jointinfo)
            return
        else:
            #print("error: wrong strategy code, pls report")
            return
        # except:
        #     #print('planner unknown error')

    # rospy.Subscriber('/hockey_robot/joint_states', JointState, callback)

## end region



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
        # else:
        #     #print(self.last_time_stamp)
        #     #print(time_stamp)
        #     #print("#########################")
        # #print("start_setting_points")
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
        return self.prediction,[self.x_speed,self.y_speed,self.x_pending,self.y_pending]


default_predicter = predicter()
default_planner = planner()
                

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
    # #print(circles)
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
        coordinates[1] = 0.0025445*y - 1.3096175
    return prediction

def callback(data : Image):#, JointData:JointState):
    # #print(1)
    current_time = data.header.seq
    if current_time % microsecond_between_each_frame == 0:
        #print(current_time)
        # #print(time.time())
        # current_time = microsecond_between_each_frame*(current_time//microsecond_between_each_frame)
        # #print(current_time)
        img = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        # filename = '{}{}.jpg'.format(save_dir,data.header.seq)
        # cv2.imwrite(filename, img)
        # # img = cv2.
        # filename = '{}{}.jpg'.format(save_dir,data.header.seq)
        # cv2.imwrite(filename, img)
        detected,x,y = detect_coordinates_of_red_balls(img)
        if not detected:
            #print("cannot detect")
            return
        # #print(data.header.seq)
        prediction, hockey_path = default_predicter.set_current_status(x,y,current_time)
        # # pred_res = str(pickle.dumps(prediction))
        # pred_res = json.dumps(prediction)
        # pred_publisher.publish(pred_res)
        # #print(prediction)
        # #print("++++++++++++++++++++++++++++++++")
        # Joint_x, Joint_y = JointData.position[0], JointData.position[2]
        # Jointinfo = [Joint_x, Joint_y]
        prediction = convert_image_coordinate_into_actual(prediction)
        #print(prediction)
        #print("++++++++++++++++++++++++++++++++")
        default_planner.update_prediction(prediction,hockey_path)#,Jointinfo)
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
    # image_sub = message_filters.Subscriber('/hockey_robot/camera1/image_raw', Image)
    # joint_sub = message_filters.Subscriber('/hockey_robot/joint_states', JointState)
    # ts = message_filters.TimeSynchronizer([image_sub, joint_sub], 1)
    # ts.registerCallback(callback)
    rospy.Subscriber('/hockey_robot/camera1/image_raw', Image, callback)
    # spin() simply keeps python from exiting until this node is stopped
    
    rospy.spin()

if __name__ == '__main__':
    # f = open("demofile3.txt", "w")
    # save_dir = '/home/futong/catkin_ws/src/air_hockey_robot/hockey_robot_gazebo/scripts'
    bridge = CvBridge()
    listener()
    # f.close()