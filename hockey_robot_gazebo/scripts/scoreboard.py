#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
from sensor_msgs.msg import Range

def reset_world():
    rospy.wait_for_service('/gazebo/reset_simulation')
    reset_world = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    reset_world()

def callback_1(data:Range):
    # score[1] +=1
    if cur_range[0] > thr and data.range <= thr:
        score[1] += 1
        reset_world()
    cur_range[0] = data.range
    rospy.loginfo(score)

def callback_2(data:Range):
    if cur_range[1] > thr and data.range <= thr:
        score[0] +=1
        reset_world()
    cur_range[1] = data.range
    rospy.loginfo(score)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global score
    global cur_range
    global thr
    thr = 0.2
    score = [0,0]
    cur_range = [0.3,0.3]

    rospy.init_node('scoreboard', anonymous=True)

    # rospy.Subscriber("/hockey_robot/joint_states",JointState, callback_1)
    rospy.Subscriber("/hockey_robot/laser/sonar_1", Range, callback_1)
    rospy.Subscriber("/hockey_robot/laser/sonar_2", Range, callback_2)
    # spin() simply keeps python from exiting until this node is stopped
    
    rospy.spin()

if __name__ == '__main__':
    listener()