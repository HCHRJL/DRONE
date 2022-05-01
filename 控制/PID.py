#!/usr/bin/env python3
import rospy
import time
import math
import signal
import sys

from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import String, Float64
from drone.msg import PID
from TDK_function import Ideal_PID
import threading


PITCH = 1500
ROLL = 1500
THROTTLE = 1500

THETA = 38, 54

##### 高度 #####
HIGH = 0
HIGH_KP = 0
HIGH_KI = 0
HIGH_KD = 0
HIGH_TARGET = 100
HIGH_ERROR_LIST = []
HIGH_ERROR_SUM = 0
HIGH_ERROR_PAST = 0
HIGH_T_PAST = time.time()
HIGH_FIRST = True
##### 座標 #####      x -> roll   y -> pitch
DISTANCE_KP = 0 ,0
DISTANCE_KI = 0, 0
DISTANCE_KD = 0 ,0
CENTER_TARGET = 320, 240
DISTANCE_ERROR_LIST = [[], []]
DISTANCE_ERROR_SUM = [0, 0]
DISTANCE_ERROR_PAST = [0, 0]
DISTANCE_T_PAST = time.time()
DISTANCE_FIRST = True
##### 速度 #####
SPEED = 0,0
SPEED_KP = 0,0
SPEED_KI = 0,0
SPEED_KD = 0,0
SPEED_TARGET = 0,0
SPEED_ERROR_LIST = [[], []]
SPEED_ERROR_SUM = [0,0]
SPEED_ERROR_PAST = [0,0]
SPEED_PAST = time.time()
SPEED_FIRST = True


########################################
S = 1
#a = 0
#######################################

STATE = "wait for other messages"
STATE_BUFFER = ""
COUNT = 0
TOLERANCE = 20

def limit():
    if MSG_COMMAND.x >= 1600:
        MSG_COMMAND.x = 1600
    elif MSG_COMMAND.x <= 1400:
        MSG_COMMAND.x = 1400
    if MSG_COMMAND.y >= 1600:
        MSG_COMMAND.y = 1600
    elif MSG_COMMAND.y <= 1400:
        MSG_COMMAND.y = 1400


def pub(): # 發佈訊息
    limit()
    PUB_COMMAND.publish(MSG_COMMAND)
    PUB_DISTANCE.publish(MSG_DISTANCE)
    PUB_PID.publish(MSG_PID)
    PUB_ROI.publish(MSG_ROI)
    print("h= %.1f\t" %HIGH, "thr= %.2f\t" %MSG_COMMAND.z, "pitch= %.1f\t" %MSG_COMMAND.y, "roll= %.1f" %MSG_COMMAND.x)
    print("tp= %0.2f\t" %MSG_PID.throttle.x, "ti= %0.2f\t" %MSG_PID.throttle.y, "td= %.2f\n" %MSG_PID.throttle.z)
    print("dp= %0.2f\t" %MSG_PID.roll.x, "dd= %0.2f\t" %MSG_PID.roll.z)
    print("%0.2f\t"%MSG_DISTANCE.x)
    print("\n")



def signal_handler(signum, frame):
    global S
    if signum == signal.SIGINT.value:  # Ctrl-C
       S = 0
       for i in range(20):
           MSG_COMMAND.z = 1000
           PUB_COMMAND.publish(MSG_COMMAND)
           print("STOP")
           time.sleep(0.01)
       rospy.signal_shutdown("stop program")
       sys.exit("stop program")

def callback_TDK_state(msg):
    global STATE_BUFFER
    STATE_BUFFER = msg.data

############################################################################################################################################
#座標

def callback_center(center): # 座標處理
    global DISTANCE_T_PAST, DISTANCE_ERROR_PAST, DISTANCE_ERROR_LIST, DISTANCE_ERROR_SUM, DISTANCE_FIRST, MSG_COMMAND, MSG_DISTANCE, STATE
    global COUNT, TOLERANCE
    ##### 時間差 #####
    distance_t_now = time.time()
    distance_dt = distance_t_now - DISTANCE_T_PAST
    DISTANCE_T_PAST = distance_t_now
    if HIGH > 15:

        ##### 誤差計算 #####
        center_error = [CENTER_TARGET[0] - center.x, CENTER_TARGET[1] - center.y]
        ''' 座標差2距離'''
        distance_per_pixel = HIGH/math.tan(math.pi*THETA[0]/180)/320, HIGH/math.tan(math.pi*THETA[1]/180)/240
        distance_partial_error = [distance_per_pixel[0]*center_error[0], distance_per_pixel[1]*center_error[1]]
        MSG_ROI.x = 10/distance_per_pixel[0]
        MSG_ROI.y = 10/distance_per_pixel[1]

        distance_error = math.sqrt(math.pow(distance_partial_error[0], 2)+math.pow(distance_partial_error[1], 2))
        if distance_error <= 10:
            distance_error = 0
        elif distance_error>10:
            distance_error -= 10

        if STATE_BUFFER == "A Point":
            if distance_error > TOLERANCE:
                change_t0 = True
            else:
                change_t0 = False
                TOLERANCE = 60
            
            if change_t0:
                COUNT = 0
                t0 = time.time()
                timer = t0
            else:
                COUNT += 1
                timer = time.time() - t0
            if COUNT > 50 or timer > 5:
                PUB_STATE.publish("B Point")
                COUNT = 0
                TOLERANCE = 20

        elif STATE_BUFFER == "B Point":
            if distance_error > TOLERANCE:
                change_t0 = True
            else:
                change_t0 = False
                TOLERANCE = 60
            
            if change_t0:
                COUNT = 0
            else:
                COUNT += 1

            if COUNT > 40:
                STATE = "land"
                #t0 = 0
                COUNT = 0
        print(COUNT)
        
        ##### 向量拆解 #####
        try:
            theta_to_separate_vector = math.atan(abs(distance_partial_error[1])/abs(distance_partial_error[0]))
        except ZeroDivisionError:
            theta_to_separate_vector = math.pi/2
        vector_x = distance_error*math.cos(theta_to_separate_vector)*math.copysign(1,distance_partial_error[0])
        vector_y = distance_error*math.sin(theta_to_separate_vector)*math.copysign(-1,distance_partial_error[1])
        
        ##### 微分 #####
        if DISTANCE_FIRST:
            distance_derror = 0, 0
            DISTANCE_FIRST = False
        else:
            distance_derror = (vector_x - DISTANCE_ERROR_PAST[0])/distance_dt, (vector_y - DISTANCE_ERROR_PAST[1])/distance_dt
        DISTANCE_ERROR_PAST = vector_x, vector_y

        ##### PID結果 #####
        distance_p = DISTANCE_KP[0]*vector_x, DISTANCE_KP[1]*vector_y
        distance_d = DISTANCE_KD[0]*distance_derror[0], DISTANCE_KD[1]*distance_derror[1]
        pid_output = distance_p[0] + distance_d[0], distance_p[1] + distance_d[1]
        ##### 訊息內容 #####
        MSG_COMMAND.x = ROLL + pid_output[0]
        MSG_COMMAND.y = PITCH - pid_output[1]
        if STATE == "land":
            MSG_COMMAND.x = ROLL
            MSG_COMMAND.y = PITCH
        MSG_DISTANCE.x = distance_partial_error[0]
        MSG_DISTANCE.y = distance_partial_error[1]
        MSG_DISTANCE.z = distance_error
        MSG_PID.roll.x = distance_p[0]
        MSG_PID.roll.y = 0
        MSG_PID.roll.z = distance_d[0]
        MSG_PID.pitch.x = distance_p[1]
        MSG_PID.pitch.y = 0
        MSG_PID.pitch.z = distance_d[1]

#########################################################################################################################################
#高度

def callback_high(h): # 高度處理
    global HIGH_T_PAST, HIGH_ERROR_PAST, HIGH_ERROR_LIST, HIGH_ERROR_SUM, HIGH_FIRST, MSG_COMMAND, HIGH, HIGH_TARGET,THROTTLE,S
#    a += 1
    HIGH = h.z
    ##### 時間差 #####
    high_t_now = time.time()
    high_dt = high_t_now - HIGH_T_PAST
    HIGH_T_PAST = high_t_now

    ##### 誤差計算 #####
    done = True
    if rospy.is_shutdown():
        HIGH_TARGET = HIGH
        THROTTLE -= 10
    elif STATE == "land" and MSG_COMMAND.z > 1000:
        HIGH_TARGET = HIGH
        THROTTLE -= 8
    high_error = HIGH_TARGET - HIGH

    ##### 積分與數量限制 #####
    if HIGH > 4:
        HIGH_ERROR_LIST.append(high_error*high_dt)
        HIGH_ERROR_SUM += HIGH_ERROR_LIST[-1]
    if len(HIGH_ERROR_LIST) > 3000:
        HIGH_ERROR_SUM -= HIGH_ERROR_LIST[0]
        del HIGH_ERROR_LIST[0]

    ##### 微分 #####
    if HIGH_FIRST:
        high_derror = 0
        HIGH_FIRST = False
    else:
        high_derror = (high_error - HIGH_ERROR_PAST)/high_dt
    HIGH_ERROR_PAST = high_error

    ##### PID結果 #####
    high_p = HIGH_KP*high_error
    high_i = HIGH_KI*HIGH_ERROR_SUM
    high_d = HIGH_KD*high_derror

    ##### 訊息內容 #####
    MSG_COMMAND.z = THROTTLE + (high_p + high_i + high_d)
#   if (STATE == "land" or rospy.is_shutdown()) and HIGH < 10:
#       MSG_COMMAND.z = 1000
#       rospy.signal_shutdown("done")

    if S == 0:
           MSG_COMMAND.z = 1000
#          rospy.signal_shutdown("done")

#    if a >= 100 and a <=200:
#       time.sleep(0.3)
    MSG_PID.throttle.x = high_p
    MSG_PID.throttle.y = high_i
    MSG_PID.throttle.z = high_d
    pub() # 發佈

###################################################################################################################################
#速度

def callback_speed(SP):
    global SPEED_T_PAST, SPEED_ERROR_PAST, SPEED_ERROR_SUM, SPEED_FIRST, MSG_COMMAND, SPEED, SPEED_TARGET,THROTTLE

###################################################################################################################################
#時間差

    SPEED_t_now = time.time()
    SPEED_dt = SPEED_t_now - SPEED_T_PAST
    SPEED_T_PAST = SPEED_t_now

###################################################################################################################################
#誤差

    if HIGH > 10:
        if rospy.is_shutdown():
            SPEED_TARGET = SPEED
            THROTTLE -= 10
        SPEED_error = SPEED_TARGET - SP #!!! SPEEND -> SP

        if SPEED_error <= TOLERANCE: #!!!
            SPEED_error = 0
        elif SPEED_error > 2:
            SPEED_error -= 2

###################################################################################################################################
#微分

            if SPEED_FIRST:
                SPEED_derror = 0,0
                SPEED_FIRST = False
            else:
                try:
                    SPEED_derror = (SPEED_X - SPEED_ERROR_PAST)/SPEED_dt,(SPEED_Y - SPEED_ERROR_PAST)/SPEED_dt
                except ZeroDivisionError:
                    SPEED_derror = 0 #!!! ?
            SPEED_ERROR_PAST = SPEED_X, SPEED_Y

###################################################################################################################################
#PID

        SPEED_p = SPEED_KP[0]*SPEED_error,PEED_KP[1]*SPEED_error
        SPEEED_d = SPEED_KD[0]*SPEED_derror[0], SPEED_KD[1]*SPEED_derror[1]
        pid_output = SPEED_p[0] + SPEED_d[0], SPEED_p[1] + SPEED_d[1]

###################################################################################################################################
#訊息內容

        MSG_COMMAND.x = ROLL + pid_output[0]
        MSG_COMMAND.y = PITCH - pid_output[1]
#        if STATE == "land":
#            MSG_COMMAND.x = ROLL
#            MSG_COMMAND.y = PITCH
        MSG_DISTANCE.x = SPEED_partial_error[0]
        MSG_DISTANCE.y = SPEED_partial_error[1]
        MSG_DISTANCE.z = SPEED_error
        MSG_PID.roll.x = SPEED_p[0]
        MSG_PID.roll.y = 0
        MSG_PID.roll.z = SPEED_d[0]
        MSG_PID.pitch.x = SPEED_p[1]
        MSG_PID.pitch.y = 0
        MSG_PID.pitch.z = SPEED_d[1]

###################################################################################################################################
##### ROS #####

if __name__ == "__main__":

   rospy.init_node("pid", anonymous=True)

   MSG_ROI = Point()
   MSG_COMMAND = Vector3()
   MSG_DISTANCE = Point()
   MSG_PID = PID()
   MSG_STATE = String()

   MSG_COMMAND.x = ROLL
   MSG_COMMAND.y = PITCH
   MSG_COMMAND.z = THROTTLE

   rospy.Subscriber("lidar", Vector3, callback_high, queue_size=1)
   rospy.Subscriber("center", Point, callback_center, queue_size=1)
   rospy.Subscriber("speed",Vector3,callback_speed,queue_size=1)
   rospy.Subscriber("/state_van_img", String, callback_TDK_state)

   signal.signal(signal.SIGINT, signal_handler)


   PUB_COMMAND = rospy.Publisher("throttle", Vector3, queue_size=1) # 飛行控制
   PUB_DISTANCE = rospy.Publisher("distance", Point, queue_size=1)
   PUB_PID = rospy.Publisher("pid", PID, queue_size=1) # pid各個結果
   PUB_STATE = rospy.Publisher("/state_van_pid", String, queue_size=1)
   PUB_ROI = rospy.Publisher("roi", Point, queue_size=1)


   rospy.spin()
