#!/usr/bin/env python

from grpc import Status
from numpy import int16
import rospy
import random
from std_srvs.srv import Empty, EmptyRequest
from sensor_msgs.msg import Range
from std_msgs.msg import String, Int16
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

class Referee():
    def __init__(self):
        self.status = 0

def pause(hand_sign:Int16):
    if hand_sign.data == 2 and referee.status == 0:
        rospy.wait_for_service("/gazebo/unpause_physics")
        pause_physics_client=rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
        pause_physics_client(EmptyRequest())
        

def reset_puck(side):
    print(side)
    state_msg = ModelState()
    state_msg.model_name = 'puck'
    state_msg.pose.position.x = 0
    state_msg.pose.position.y = 0.5 if side == 'red' else -0.5
    state_msg.pose.position.z = 0.805
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

    except rospy.ServiceException as e:
        print("Service call failed: ",e)

def callback_blue(data:Range):
    side = 'blue'
    if cur_range[0] > thr and data.range <= thr:
        score[1] += 1
        reset_puck(side)
        # goal_pub.publish('1')
    cur_range[0] = data.range
    score_pub.publish('red vs blue --- {}:{}'.format(score[0],score[1]))
    
    
    

def callback_red(data:Range):
    side = 'red'
    if cur_range[1] > thr and data.range <= thr:
        score[0] +=1
        reset_puck(side)
        # goal_pub.publish('-1')
    cur_range[1] = data.range
    score_pub.publish('red vs blue --- {}:{}'.format(score[0],score[1]))
    
    
def callback_reset(data):
    side = random.choice(["red", "blue"]) 
    reset_puck(side)

def referee():

    global score
    global cur_range
    global thr
    thr = 0.5
    score = [0,0]
    cur_range = [0.3,0.3]

    global score_pub
    global goal_pub
    global side
    global referee
    referee = Referee()

    side = random.choice(["red", "blue"]) 

    score_pub = rospy.Publisher('hockey_robot/referee/score', String, queue_size=10)
    # goal_pub = rospy.Publisher('hockey_robot/referee/goal', String, queue_size=10)
    # serve_pub = rospy.Publisher('hockey_robot/referee/side', String, queue_size=10)
    

    
    rospy.init_node('referee', anonymous=True)
    # serve_pub.publish(side)
    # reset_puck(side)
    # rospy.Subscriber("/hockey_robot/joint_states",JointState, callback_1)
    rospy.Subscriber("/hockey_robot/laser/sonar_1", Range, callback_blue)
    rospy.Subscriber("/hockey_robot/laser/sonar_2", Range, callback_red)
    rospy.Subscriber("/hockey_robot/puck/reset_pose", String, callback_reset)
    # rospy.Subscriber("/hockey_robot/gest_controller/hand_sign",Int16, pause)
    
    # spin() simply keeps python from exiting until this node is stopped
    
    rospy.spin()
    

if __name__ == '__main__':
    referee()