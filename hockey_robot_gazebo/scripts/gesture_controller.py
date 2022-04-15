#!/usr/bin/env python

from cmath import sin
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

import os
import cv2 as cv
import copy
import itertools
import mediapipe as mp
import math

import sys
# print(sys.path)

import numpy as np
import tensorflow as tf

from gesture_recognition.keypoint_classifier import KeyPointClassifier

def draw_bounding_rect(use_brect, image, brect, label):
    if use_brect:

        cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[3]),
                     (0, 0, 0), 1)

    return image

def calc_bounding_rect(image, landmarks):
    image_width, image_height = image.shape[1], image.shape[0]

    landmark_array = np.empty((0, 2), int)

    for _, landmark in enumerate(landmarks.landmark):
        landmark_x = min(int(landmark.x * image_width), image_width - 1)
        landmark_y = min(int(landmark.y * image_height), image_height - 1)

        landmark_point = [np.array((landmark_x, landmark_y))]

        landmark_array = np.append(landmark_array, landmark_point, axis=0)

    x, y, w, h = cv.boundingRect(landmark_array)

    return [x, y, x + w, y + h]

def calc_landmark_list(image, landmarks):
    image_width, image_height = image.shape[1], image.shape[0]

    landmark_point = []


    for _, landmark in enumerate(landmarks.landmark):
        landmark_x = min(int(landmark.x * image_width), image_width - 1)
        landmark_y = min(int(landmark.y * image_height), image_height - 1)
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

def get_hand_position(cap,orgin,
                        radius = 0.1,
                        color = (0, 0, 255),
                        thickness =0
):
    with mp_hands.Hands(
        model_complexity=0,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as hands:
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            # If loading a video, use 'break' instead of 'continue'.

        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        
        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        
        results = hands.process(image)

        # Draw the hand annotations on the image.
        image.flags.writeable = True
        image = cv.cvtColor(image, cv.COLOR_RGB2BGR)
        image_hight, image_width, _ = image.shape
        
        orgin = (int(image_width*orgin[0]), int(image_hight*orgin[1]))
        radius = int( min(image_hight, image_width) * radius)

        x,y = -1,-1
        hand_0 = None
        hand_sign_id = None
        if results.multi_hand_landmarks:
            hand_0 = results.multi_hand_landmarks[0]
            x = hand_0.landmark[mp_hands.HandLandmark.WRIST].x * image_width
            y = hand_0.landmark[mp_hands.HandLandmark.WRIST].y * image_hight



            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    image,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style())



            brect = calc_bounding_rect(image, hand_landmarks)

            landmark_list = calc_landmark_list(image, hand_landmarks)


            pre_processed_landmark_list = pre_process_landmark(
                landmark_list)
            hand_sign_id = keypoint_classifier(pre_processed_landmark_list)
            # rospy.loginfo('hand_sign:'+hand_sign_id)

            radians = math.atan2(y-orgin[1], x-orgin[0])

            x_pivot, y_pivot = (math.cos(radius), math.sin(radius))
            # print(radians)

            image = cv.arrowedLine(image, orgin, (int(x),int(y)),
                                     (0, 255, 0), 2)
            # image = cv.arrowedLine(image, orgin, (int(x_pivot),int(y_pivot)),
            #                 (0, 0, 255), 2)
            image = cv.circle(image, orgin, radius, color, thickness)
            image = draw_bounding_rect(True, image, brect,hand_sign_id)
        image = cv.flip(image, 1)


        hand_pub.publish(br.cv2_to_imgmsg(image,"bgr8"))
        return hand_0,hand_sign_id ,(image_hight, image_width)
        # Flip the image horizontally for a selfie-view display.
        # cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))

def talker():
    # global tracker_pub
    # global pusher_pub
    global hand_pub
    tracker_pub = rospy.Publisher('hockey_robot/joint3_position_controller/command', Float64, queue_size=10)
    pusher_pub = rospy.Publisher('hockey_robot/joint4_position_controller/command', Float64, queue_size=10)
    status_pub = rospy.Publisher('hockey_robot/joint4_position_controller/command', Float64, queue_size=10)
    # tracker_vel_pub = rospy.Publisher('hockey_robot/joint5_position_controller/command', Float64, queue_size=10)
    hand_pub = rospy.Publisher('/hockey_robot/gest_controller/img', Image, queue_size=10)

    rospy.init_node('gest_controller', anonymous=True)

    cap = cv.VideoCapture(0)
    # pos = get_hand_position(cap,(0,0))
    orgin = (0.5,0.5)
    x,y = (0,0)
    # while not rospy.is_shutdown():
    while True:
        hand,hand_sign_id, image_shape  = get_hand_position(cap,orgin)
        if hand:
            x = hand.landmark[mp_hands.HandLandmark.WRIST].x-orgin[0]
            y = (hand.landmark[mp_hands.HandLandmark.WRIST].y-orgin[1])
        else:
            x,y = x,y
        if hand_sign_id == 1:
            tracker_pub.publish(-y*4)
            pusher_pub.publish(-x*4)
        elif hand_sign_id == 0:
            tracker_pub.publish(10000)
            # pusher_pub.publish(-x*4)

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