#!/usr/bin/env python

from ast import Str
from cmath import sin
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String, Int16

import os
import cv2 as cv
import copy
import itertools
import mediapipe as mp
import math
import json
import sys
# print(sys.path)

import numpy as np
import tensorflow as tf

from gesture_recognition.keypoint_classifier import KeyPointClassifier

ORIGIN = (0.5,0.5)

def draw_bounding_rect(use_brect, image, brect, label):
    if use_brect:

        cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[3]),
                     (0, 0, 0), 1)

    return image

def calc_bounding_rect(image_shape, landmarks):
    image_width, image_height = image_shape

    landmark_array = np.empty((0, 2), int)

    for _, landmark in enumerate(landmarks.landmark):
        landmark_x = min(int(landmark.x * image_width), image_width - 1)
        landmark_y = min(int(landmark.y * image_height), image_height - 1)

        landmark_point = [np.array((landmark_x, landmark_y))]

        landmark_array = np.append(landmark_array, landmark_point, axis=0)

    x, y, w, h = cv.boundingRect(landmark_array)

    return [x, y, x + w, y + h]

def calc_landmark_list(image, landmarks):
    image_width, image_height = image

    landmark_point = []


    for _, landmark in enumerate(landmarks):
        landmark_x = min(int(landmark['x'] * image_width), image_width - 1)
        landmark_y = min(int(landmark['y'] * image_height), image_height - 1)
        # landmark_z = landmark.z

        landmark_point.append([landmark_x, landmark_y])
    return landmark_point

def pre_process_landmark(landmark_list):
    temp_landmark_list = copy.deepcopy(landmark_list)


    base_x, base_y = 0, 0
    for index, landmark_point in enumerate(temp_landmark_list):
        if index == 0:
            base_x, base_y = landmark_point[0], landmark_point[1]

        temp_landmark_list[index][0] = temp_landmark_list[index][0] - base_x
        temp_landmark_list[index][1] = temp_landmark_list[index][1] - base_y


    temp_landmark_list = list(
        itertools.chain.from_iterable(temp_landmark_list))


    max_value = max(list(map(abs, temp_landmark_list)))

    def normalize_(n):
        return n / max_value

    temp_landmark_list = list(map(normalize_, temp_landmark_list))

    return temp_landmark_list


def pre_process_point_history(image, point_history):
    image_width, image_height = image.shape[1], image.shape[0]

    temp_point_history = copy.deepcopy(point_history)


    base_x, base_y = 0, 0
    for index, point in enumerate(temp_point_history):
        if index == 0:
            base_x, base_y = point[0], point[1]

        temp_point_history[index][0] = (temp_point_history[index][0] -
                                        base_x) / image_width
        temp_point_history[index][1] = (temp_point_history[index][1] -
                                        base_y) / image_height


    temp_point_history = list(
        itertools.chain.from_iterable(temp_point_history))

    return temp_point_history

def callback(data:String):
    landmarks = json.loads(data.data)
    landmark_list = calc_landmark_list((1280,720), landmarks)
    pre_processed_landmark_list = pre_process_landmark(landmark_list)
    hand_sign_id = keypoint_classifier(pre_processed_landmark_list)

    x = landmarks[0]['x']-ORIGIN[0]
    y = landmarks[0]['y']-ORIGIN[1]
    
    # print(x, y , hand_sign_id)
    if hand_sign_id == 1:
        tracker_pub.publish(-y*4)
        pusher_pub.publish(-x*4)
    elif hand_sign_id == 0:
        tracker_pub.publish(10000)
    else:
        hand_sign_pub.publish(hand_sign_id)
    

def talker():
    # global tracker_pub
    # global pusher_pub
    global tracker_pub
    global pusher_pub
    global hand_sign_pub
    tracker_pub = rospy.Publisher('hockey_robot/joint3_position_controller/command', Float64, queue_size=10)
    pusher_pub = rospy.Publisher('hockey_robot/joint4_position_controller/command', Float64, queue_size=10)
    # tracker_vel_pub = rospy.Publisher('hockey_robot/joint5_position_controller/command', Float64, queue_size=10)
    hand_sign_pub = rospy.Publisher('/hockey_robot/gest_controller/hand_sign', Int16, queue_size=10)
    rospy.init_node('web_gest_controller', anonymous=True)
    rospy.Subscriber('/landmark', String, callback)
    rospy.spin()
    # cap = cv.VideoCapture(0)
    # # pos = get_hand_position(cap,(0,0))
    
    # # while not rospy.is_shutdown():
    # while True:
    #     hand,hand_sign_id, image_shape  = get_hand_position(cap,orgin)
    #     if hand:
    #         x = hand.landmark[mp_hands.HandLandmark.WRIST].x-orgin[0]
    #         y = (hand.landmark[mp_hands.HandLandmark.WRIST].y-orgin[1])
    #     else:
    #         x,y = x,y
    #     if hand_sign_id == 1:
    #         tracker_pub.publish(-y*4)
    #         pusher_pub.publish(-x*4)
    #     elif hand_sign_id == 0:
    #         tracker_pub.publish(10000)
    #         # pusher_pub.publish(-x*4)

if __name__ == '__main__':

    DIRECTION = []
    VELECITY = [50, 100, 500]

    model_save_path = os.path.join('/home/xumingjie/catkin_ws/src/air_hockey_robot/hockey_robot_gazebo/scripts/model/keypoint_classifier/keypoint_classifier.tflite') 
    keypoint_classifier = KeyPointClassifier(model_path=model_save_path)

    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    mp_hands = mp.solutions.hands
    br = CvBridge()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass