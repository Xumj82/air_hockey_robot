#!/usr/bin/env python


from sympy import im
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

import cv2
import mediapipe as mp


def get_hand_position(cap):
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
        
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        results = hands.process(image)

        # Draw the hand annotations on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        image_hight, image_width, _ = image.shape
        
        x,y = -1,-1
        hand_0 = None
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

            # for hand_landmarks in results.multi_hand_landmarks:
            #     print(
            #         f'Index finger tip coordinate: (',
            #         f'{hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x * image_width}, '
            #         f'{hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y * image_hight})'
            #     )
        hand_pub.publish(br.cv2_to_imgmsg(image,"bgr8"))
        return hand_0
        # Flip the image horizontally for a selfie-view display.
        # cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))

def talker():
    # global tracker_pub
    # global pusher_pub
    global hand_pub
    tracker_pub = rospy.Publisher('hockey_robot/joint3_position_controller/command', Float64, queue_size=10)
    pusher_pub = rospy.Publisher('hockey_robot/joint4_position_controller/command', Float64, queue_size=10)
    hand_pub = rospy.Publisher('/hockey_robot/hand_controller/hand', Image, queue_size=10)

    rospy.init_node('hand_controller', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    cap = cv2.VideoCapture(0)
    # pos = get_hand_position(cap,(0,0))
    orgin = (0,0.5)
    x,y = (0,0)
    while not rospy.is_shutdown():
        hand = get_hand_position(cap)
        if hand:
            x = hand.landmark[mp_hands.HandLandmark.WRIST].x-orgin[0]
            y = (hand.landmark[mp_hands.HandLandmark.WRIST].y-orgin[1])*2
        else:
            x,y = x,y
        tracker_pub.publish(x)
        pusher_pub.publish(y)
        # if pos != (0,0):
        #     x,y = get_hand_position(cap,pos)
        #     if (x,y) != (0,0):
        #         tracker_pub.publish(y)
        #         pusher_pub.publish(x)
        #     rospy.loginfo( 'pos:{} {}'.format(x,y))
        # else:
        #     pos = get_hand_position(cap,(0,0))
        rospy.loginfo((x,y))

        # pub.publish(hello_str)
        rate.sleep()
    cap.release()

# def talker():
#     # global tracker_pub
#     # global pusher_pub
#     tracker_pub = rospy.Publisher('hockey_robot/joint1_position_controller/command', Float64, queue_size=10)
#     pusher_pub = rospy.Publisher('hockey_robot/joint2_position_controller/command', Float64, queue_size=10)
#     hand_pub = rospy.Publisher('/hockey_robot/hand_controller/hand', Image, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz

#     cap = cv2.VideoCapture(0)
#     # pos = get_hand_position(cap,(0,0))
#     orgin = (0,0.5)
#     x,y = (0,0)


#     cap = cv2.VideoCapture(0)
#     with mp_hands.Hands(
#         model_complexity=0,
#         min_detection_confidence=0.5,
#         min_tracking_confidence=0.5) as hands:
#         while cap.isOpened():
#             success, image = cap.read()
#             if not success:
#                 print("Ignoring empty camera frame.")
#                 # If loading a video, use 'break' instead of 'continue'.
#                 continue

#             # To improve performance, optionally mark the image as not writeable to
#             # pass by reference.
#             image.flags.writeable = False
#             image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
#             results = hands.process(image)

#             # Draw the hand annotations on the image.
#             image.flags.writeable = True
#             image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
#             if results.multi_hand_landmarks:
#                 hand = results.multi_hand_landmarks[0]
#                 x = hand.landmark[mp_hands.HandLandmark.WRIST].x-orgin[0]
#                 y = hand.landmark[mp_hands.HandLandmark.WRIST].y-orgin[1]

#             tracker_pub.publish(x)
#             pusher_pub.publish(y)

#             hand_pub.publish(br.cv2_to_imgmsg(image))

#             rospy.loginfo((x,y))
            
#                 # Flip the image horizontally for a selfie-view display.
#             # cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))

#     cap.release()

if __name__ == '__main__':

    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    mp_hands = mp.solutions.hands
    br = CvBridge()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass