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

# from sympy import false
import rospy
from std_msgs.msg import Float64
# from sensor_msgs.msg import JointState
# import json
import time

def check_strategy_1(predict_data:dict):
    hockey_position_list = list(predict_data.values())
    if hockey_position_list[-1][0]<0:
        return True
    elif hockey_position_list[-1][0]>0:
        return False


def strategy_1(predict_data:dict):
    hockey_position_list = list(predict_data.values())
    for i in range(len(hockey_position_list)-1):
        if hockey_position_list[i][0] >0 and hockey_position_list[i+1][0] <=0 :
            defence_position = (hockey_position_list[i][1] + hockey_position_list[i+1][1])/2
            print('go to defence position (%0.2f,%0.2f)'%(0,defence_position))
            setPusher1_position(defence_position)
            setTrack1_position(0)

    
def strategy_2(predict_data:dict):
    for pre_time,hockey_pos in predict_data:
        if hockey_pos[0]<=0.50:
            defence_position_x,defence_position_y = hockey_pos[0],hockey_pos[1]
            setPusher1_position(defence_position_y)
            setTrack1_position(defence_position_x)


def strategy_3(predict_data:dict):
    for pre_time,hockey_pos in predict_data:
        if hockey_pos[0]<=0.50:
            cur_time = time.time()
            defence_position_x,defence_position_y = hockey_pos[0],hockey_pos[1]
            duration = cur_time - pre_time - 0.5
            rospy.sleep(duration)
            setPusher1_position(defence_position_y)
            setTrack1_position(defence_position_x)

def setPusher1_position(data):
    pusher_pub.publish(data)

def setTrack1_position(data):
    tracker_pub.publish(data)

def reset_position():
    pusher_pub.publish(0)
    tracker_pub.publish(0)

def check_plan(predict_data):

    hockey_position_list = predict_data.values()
    hocky_x_list=[]
    for i in hockey_position_list:
        hocky_x_list.append(i[0])

    if hocky_x_list[-1]>=hocky_x_list[0]:
        return False
    
    move_dis=0
    for i in range(1,len(hocky_x_list)):
        move_dis += abs(hocky_x_list[i]-hocky_x_list[i-1])

    if move_dis <=0.1:
        return False
    return True



# def callback(data: JointState):
#     # #step 1: get current joints position:

#     # track1_position = data.position[0]
#     # pusher1_position = data.position[2]

#     # #step 2:strategy choice:

#     # if default_strategy <= 1:
#     #     strategy_1(predict_data)
#     # elif default_strategy == 2:
#     #     strategy_2(predict_data,track1_position,pusher1_position)
#     # else:
#     #     strategy_3(predict_data,track1_position,pusher1_position)
#     return


def plan_strategy(data:dict):

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    # 0-0.86 is from left to mid for joint_1.
    # 0.5~0 is from top to mid for joint_2.
    # 0~-0.5 is from top to mid for joint_2.

    global tracker_pub
    global pusher_pub
    global predict_data
    global default_strategy 
    # global timestamp_list
    # global postion_list
    # global track1_position
    # global pusher1_position


    default_strategy = 1

    
    predict_data = data


    tracker_pub = rospy.Publisher('hockey_robot/joint1_position_controller/command', Float64, queue_size=10)
    pusher_pub = rospy.Publisher('hockey_robot/joint2_position_controller/command', Float64, queue_size=10)

    rospy.init_node('planner', anonymous=True)

    #step 1:check if need plan:
    try:
        if not check_plan(predict_data):
            print('No need to plan')
            reset_position()
            return
        
        #step 2:strategy choice:
        while not rospy.is_shutdown():

            if default_strategy <= 1:
                if check_strategy_1(predict_data):
                    strategy_1(predict_data)
                else:
                    # strategy_1 can not use
                    print ('Defence strategy(strategy1) is not suit')
            elif default_strategy == 2:
                strategy_2(predict_data)
            else:
                strategy_3(predict_data)
    except:
        print('planner unknown error')


    # rospy.Subscriber('/hockey_robot/joint_states', JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()



# if __name__ == '__main__':
#     data = {1500000:[0.3,0.5],1600000:[0.2,0.4],1700000:[0.1,0.3],1800000:[-0.1,0.2],1900000:[-0.2,0.1]}
#     plan_strategy(data)