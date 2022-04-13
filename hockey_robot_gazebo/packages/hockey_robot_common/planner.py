import rospy
from std_msgs.msg import Float64
# from sensor_msgs.msg import JointState
# import json
import time

### global config
HOCKEY_PATH_LENGTH = 4

STRATEGY_3_SAFE_LINE = 0.4
STRATEGY_3_ATTACK_LINE = 0.3
STRATEGY_3_PREPARE_RANGE = 5

microsecond_between_each_frame = 10

default_strategy = 3

class Planner():
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
        hockey_position_list = list(self.predict_data.values())
        for i in range(len(hockey_position_list)-1):
            if hockey_position_list[i][0] >0 and hockey_position_list[i+1][0] <=0 :
                self.strategy_1_defence_position = (hockey_position_list[i][1] + hockey_position_list[i+1][1])/2
                #print('go to defence position (%0.2f,%0.2f)'%(0,self.strategy_1_defence_position))
                self.setPusher1_position(self.strategy_1_defence_position)
                self.setTrack1_position(0)
    def strategy_2(self):
        for pre_time,hockey_pos in self.predict_data.items():
            if hockey_pos[0]<=0.86:
                self.strategy_2_defence_position_x,self.strategy_2_defence_position_y = hockey_pos[0],hockey_pos[1]
                #print('go to defence position (%0.2f,%0.2f)'%(self.strategy_2_defence_position_x,self.strategy_2_defence_position_y))
                self.setPusher1_position(self.strategy_2_defence_position_y)
                self.setTrack1_position(self.strategy_2_defence_position_x-0.05)


    def strategy_3(self,same_path):
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
                        self.setPusher1_position(self.strategy_3_defence_position)
                        self.setTrack1_position(STRATEGY_3_SAFE_LINE)
                        return
                    else:
                        return

    def check_hockey_static(self):
        hockey_position_list = self.predict_data.values()
        hockey_x_list=[]
        for i in hockey_position_list:
            hockey_x_list.append(i[0])
        move_dis=0
        for i in range(1,len(hockey_x_list)):
            move_dis += abs(hockey_x_list[i]-hockey_x_list[i-1])
        if move_dis <=0.1:
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
        if len(hockey_path) != HOCKEY_PATH_LENGTH:
            return False
        path_difference = 0
        for i in range(len(hockey_path)):
            path_difference += abs(self.hockey_path[i]-hockey_path[i])
        if path_difference <= 1:
            return True
        else:
            self.hockey_path = hockey_path
            return False
    
    def update_prediction(self,prediction,hockey_path):
        self.predict_data = prediction
        same_path = self.update_hockey_path(hockey_path)
        self.plan_strategy(same_path)
    def plan_strategy(self,same_path):

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
        else:
            self.strategy = default_strategy
        
        #step 2:strategy choice:
        while not rospy.is_shutdown():

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
            elif self.strategy  == 2:
                #print('plan start strategy_2')
                self.strategy_2()
                return
            else:
                #print('plan start strategy_3')
                self.strategy_3(same_path)
                return
        # except:
        #     #print('planner unknown error')

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
            print('go to defence position (%0.2f,%0.2f)'%(defence_position_x,defence_position_y))
            setPusher1_position(defence_position_y)
            setTrack1_position(defence_position_x)


def strategy_3(predict_data:dict):
    for pre_time,hockey_pos in predict_data:
        if hockey_pos[0]<=0.50:
            cur_time = time.time()*100
            defence_position_x,defence_position_y = hockey_pos[0],hockey_pos[1]
            duration = cur_time - pre_time - 50
            rospy.sleep(duration)
            print('go to defence position (%0.2f,%0.2f)'%(defence_position_x,defence_position_y))
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
    # -0.5~0 is from top to mid for joint_2.
    # 0~0.5 is from mid to bot for joint_2.

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

    # rospy.init_node('planner', anonymous=True)

    #step 1:check if need plan:
    try:
        print('plan start')
        if not check_plan(predict_data):
            print('No need to plan')
            reset_position()
            return
        
        #step 2:strategy choice:
        while not rospy.is_shutdown():

            if default_strategy <= 1:
                if check_strategy_1(predict_data):
                    print('plan start strategy_1')
                    strategy_1(predict_data)
                    return
                else:
                    # strategy_1 can not use
                    print ('Defence strategy(strategy1) is not suit')
                    return
            elif default_strategy == 2:
                print('plan start strategy_2')
                strategy_2(predict_data)
                return
            else:
                print('plan start strategy_3')
                strategy_3(predict_data)
                return
    except:
        print('planner unknown error')


    # rospy.Subscriber('/hockey_robot/joint_states', JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()



# if __name__ == '__main__':
#     data = {1500000:[0.3,0.5],1600000:[0.2,0.4],1700000:[0.1,0.3],1800000:[-0.1,0.2],1900000:[-0.2,0.1]}
#     plan_strategy(data)