#!/bin/env python3.8
import cv2
import mediapipe as mp
import copy
import math

import numpy
numpy.random.BitGenerator = numpy.random.bit_generator.BitGenerator

import pandas as pd

TEMP_ORIGINAL = {}

##############################################################################################

import numpy as np
import pybullet as p
import time
import pybullet_data
import inspect
#from sensor_msgs.msg import JointState
import threading
import datetime
import csv


FILENAME = 'TRACE_NOSLIP.csv'
TRACE_HEADER = ['Event','Timestamp','J0_position', 'J0_velocity', 'J0_Fx', 'J0_Fy', 'J0_Fz', 'J0_Mx', 'J0_My', 'J0_Mz', 'J0_torque', 'J1_position', 'J1_velocity', 'J1_Fx', 'J1_Fy', 'J1_Fz', 'J1_Mx', 'J1_My', 'J1_Mz', 'J1_torque', 'J2_position', 'J2_velocity', 'J2_Fx', 'J2_Fy', 'J2_Fz', 'J2_Mx', 'J2_My', 'J2_Mz', 'J2_torque', 'J3_position', 'J3_velocity', 'J3_Fx', 'J3_Fy', 'J3_Fz', 'J3_Mx', 'J3_My', 'J3_Mz', 'J3_torque', 'J4_position', 'J4_velocity', 'J4_Fx', 'J4_Fy', 'J4_Fz', 'J4_Mx', 'J4_My', 'J4_Mz', 'J4_torque', 'J5_position', 'J5_velocity', 'J5_Fx', 'J5_Fy', 'J5_Fz', 'J5_Mx', 'J5_My', 'J5_Mz', 'J5_torque', 'J6_position', 'J6_velocity', 'J6_Fx', 'J6_Fy', 'J6_Fz', 'J6_Mx', 'J6_My', 'J6_Mz', 'J6_torque', 'J7_position', 'J7_velocity', 'J7_Fx', 'J7_Fy', 'J7_Fz', 'J7_Mx', 'J7_My', 'J7_Mz', 'J7_torque', 'J8_position', 'J8_velocity', 'J8_Fx', 'J8_Fy', 'J8_Fz', 'J8_Mx', 'J8_My', 'J8_Mz', 'J8_torque', 'J9_position', 'J9_velocity', 'J9_Fx', 'J9_Fy', 'J9_Fz', 'J9_Mx', 'J9_My', 'J9_Mz', 'J9_torque', 'J10_position', 'J10_velocity', 'J10_Fx', 'J10_Fy', 'J10_Fz', 'J10_Mx', 'J10_My', 'J10_Mz', 'J10_torque', 'J11_position', 'J11_velocity', 'J11_Fx', 'J11_Fy', 'J11_Fz', 'J11_Mx', 'J11_My', 'J11_Mz', 'J11_torque', 'J12_position', 'J12_velocity', 'J12_Fx', 'J12_Fy', 'J12_Fz', 'J12_Mx', 'J12_My', 'J12_Mz', 'J12_torque', 'J13_position', 'J13_velocity', 'J13_Fx', 'J13_Fy', 'J13_Fz', 'J13_Mx', 'J13_My', 'J13_Mz', 'J13_torque', 'J14_position', 'J14_velocity', 'J14_Fx', 'J14_Fy', 'J14_Fz', 'J14_Mx', 'J14_My', 'J14_Mz', 'J14_torque', 'J15_position', 'J15_velocity', 'J15_Fx', 'J15_Fy', 'J15_Fz', 'J15_Mx', 'J15_My', 'J15_Mz', 'J15_torque', 'J16_position', 'J16_velocity', 'J16_Fx', 'J16_Fy', 'J16_Fz', 'J16_Mx', 'J16_My', 'J16_Mz', 'J16_torque', 'J17_position', 'J17_velocity', 'J17_Fx', 'J17_Fy', 'J17_Fz', 'J17_Mx', 'J17_My', 'J17_Mz', 'J17_torque', 'J18_position', 'J18_velocity', 'J18_Fx', 'J18_Fy', 'J18_Fz', 'J18_Mx', 'J18_My', 'J18_Mz', 'J18_torque', 'J19_position', 'J19_velocity', 'J19_Fx', 'J19_Fy', 'J19_Fz', 'J19_Mx', 'J19_My', 'J19_Mz', 'J19_torque' ]
def save_to_csv(FILENAME, TRACE_CSV, type_open='a'):    
    with open(FILENAME,type_open,newline="") as trace_file:
        writer = csv.writer(trace_file, )
        writer.writerow(TRACE_CSV)
save_to_csv(FILENAME, TRACE_HEADER, 'w')

clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
	physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally

def get_joint_states(robot, numJoints):
    DATUM = []
    joint_states = p.getJointStates(robot,range(numJoints),physicsClientId=physicsClient)   #0 to 20::: 4 are fixed
    for j in joint_states:
        for quadruple, k in enumerate(j):
            if quadruple == 2:
                for l in k:
                    DATUM.append(l)
            else:
                DATUM.append(k)
    print(DATUM)
    input('#$#$#$#$#$#$#$#$#')
    return DATUM

def get_joint_angles(robot, numJoints):
    DATUM=[]
    joint_states = p.getJointStates(robot,range(numJoints),physicsClientId=physicsClient)   #0 to 20::: 4 are fixed
    for j in joint_states:
        for quadruple, k in enumerate(j):
            if quadruple ==1 : #Just the joint angle
                DATUM.append(k)
    return DATUM

def get_joint_states_hand_only(robot, numJoints):
    DATUM = []
    joint_states = p.getJointStates(robot,range(numJoints),physicsClientId=physicsClient)   #0 to 20::: 4 are fixed
    for jno, j in enumerate(joint_states):
        if jno in [16,17,18,19,20, 25,26,27,28,29, 34,35,36,37,38, 43,44,45,46,47]:
            #print(jno)
            for quadruple, k in enumerate(j):
                if quadruple == 2:
                    for l in k:
                        DATUM.append(l)
                else:
                    DATUM.append(k)
    return DATUM

planeId = p.loadURDF("plane.urdf")

kuka_allegro_hand_biotac = p.loadURDF("ll4ma_robots_description/robots/kuka-allegro-biotac.urdf")
for j in range(53):
    p.enableJointForceTorqueSensor(kuka_allegro_hand_biotac, j, enableSensor=1)

numJoints = p.getNumJoints(kuka_allegro_hand_biotac)

p.setRealTimeSimulation(1)

from datetime import datetime
p.setGravity(0,0,-9.8)

p.changeDynamics(kuka_allegro_hand_biotac,-1, lateralFriction=0.5)
p.changeDynamics(kuka_allegro_hand_biotac,-1, rollingFriction=0.5)
joint_cmd = [0 for _ in range(53)]
p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)
#RECORD 1ST FROM HERE
time.sleep(1)
save_to_csv(FILENAME,['RESET_POSITION',datetime.now().time() ]+get_joint_states_hand_only( kuka_allegro_hand_biotac, numJoints),'a')

time.sleep(5)
for _ in range(100):
    save_to_csv(FILENAME,['STEADY_AFTER_ACHIEVING_RESET_POSITION',datetime.now().time() ]+get_joint_states_hand_only( kuka_allegro_hand_biotac, numJoints),'a')

# time.sleep(5)

print('&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&')

import random



#############################################################################################

mp_hands = mp.solutions.hands

columns__=['Good_FrameNumber']
for i in range(21):
    for j in ['x', 'y', 'z']:
        columns__.append(str(i)+'_'+j)

df = pd.DataFrame(columns = columns__)

cap = cv2.VideoCapture(0)
# cap = cv2.VideoCapture("Fist.mp4")

mpHands = mp.solutions.hands
mpDraw = mp.solutions.drawing_utils

pTime = 0
cTime = 0
ctr=0

S0 = {}
S0_prime = {}

Angles={
    'TI':-999,
    'IM':-999,
    'MR':-999,
    'RP':-999
}

def AnglesCalc__(State):
    v1 = (State['THUMB_TIP'][0]-State['W2T'][0], State['THUMB_TIP'][1]-State['W2T'][1])
    v2 = (State['WRIST'][0]-State['W2T'][0], State['WRIST'][1]-State['W2T'][1])
    
    Angles['WT'] = math.acos((v1[0]*v2[0]+v1[1]*v2[1])/(0.0001+((v1[0]**2+v1[1]**2)*(v2[0]**2+v2[1]**2))**0.5))

    v1 = (State['MIDDLE_FINGER_TIP'][0]-State['W2M'][0], State['MIDDLE_FINGER_TIP'][1]-State['W2M'][1])
    v2 = (State['WRIST'][0]-State['W2M'][0], State['WRIST'][1]-State['W2M'][1])
    
    Angles['WM'] = math.acos((v1[0]*v2[0]+v1[1]*v2[1])/(0.0001+((v1[0]**2+v1[1]**2)*(v2[0]**2+v2[1]**2))**0.5))

    v1 = (State['RING_FINGER_TIP'][0]-State['W2R'][0], State['RING_FINGER_TIP'][1]-State['W2R'][1])
    v2 = (State['WRIST'][0]-State['W2R'][0], State['WRIST'][1]-State['W2R'][1])
    
    Angles['WR'] = math.acos((v1[0]*v2[0]+v1[1]*v2[1])/(0.0001+((v1[0]**2+v1[1]**2)*(v2[0]**2+v2[1]**2))**0.5))

    v1 = (State['INDEX_FINGER_TIP'][0]-State['W2I'][0], State['INDEX_FINGER_TIP'][1]-State['W2I'][1])
    v2 = (State['WRIST'][0]-State['W2I'][0], State['WRIST'][1]-State['W2I'][1])
    
    Angles['WI'] = math.acos((v1[0]*v2[0]+v1[1]*v2[1])/(0.0001+((v1[0]**2+v1[1]**2)*(v2[0]**2+v2[1]**2))**0.5))



with mp_hands.Hands(
    model_complexity=0,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as hands:
    while True:
        
        while(True):
            success, img = cap.read()
            if(success==False):
                continue

            imgRGB = cv2.flip(img,1)
            results = hands.process(imgRGB)

            cv2.imshow("Image", imgRGB)
            signal = cv2.waitKey(1)
            if(signal==99):
                break
        
        y__ =1
        img_height, img_width, _ = img.shape

        print('SHAPE : ', img_height, img_width, _)
        print(results)


        # Assumption : Only 1 hand under observation
        try:
            coordinates_info = results.multi_hand_landmarks[0]
        except:
            continue

        if(True):
            S0_prime['WRIST'] = (coordinates_info.landmark[mp_hands.HandLandmark.WRIST].x * img_width, coordinates_info.landmark[mp_hands.HandLandmark.WRIST].y * img_height)
            S0_prime['W2I'] = (coordinates_info.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].x * img_width, coordinates_info.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].y * img_height)
            S0_prime['INDEX_FINGER_TIP'] = (coordinates_info.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x * img_width, coordinates_info.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y * img_height)
            S0_prime['W2T'] = (coordinates_info.landmark[mp_hands.HandLandmark.THUMB_CMC].x * img_width, coordinates_info.landmark[mp_hands.HandLandmark.THUMB_CMC].y * img_height)
            S0_prime['THUMB_TIP'] = (coordinates_info.landmark[mp_hands.HandLandmark.THUMB_TIP].x * img_width, coordinates_info.landmark[mp_hands.HandLandmark.THUMB_TIP].y * img_height)
            S0_prime['W2M'] = (coordinates_info.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].x * img_width, coordinates_info.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y * img_height)
            S0_prime['MIDDLE_FINGER_TIP'] = (coordinates_info.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].x * img_width, coordinates_info.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y * img_height)
            S0_prime['W2R'] = (coordinates_info.landmark[mp_hands.HandLandmark.RING_FINGER_MCP].x * img_width, coordinates_info.landmark[mp_hands.HandLandmark.RING_FINGER_MCP].y * img_height)
            S0_prime['RING_FINGER_TIP'] = (coordinates_info.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].x * img_width, coordinates_info.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].y * img_height)
            S0_prime['W2P'] = (coordinates_info.landmark[mp_hands.HandLandmark.PINKY_MCP].x * img_width, coordinates_info.landmark[mp_hands.HandLandmark.PINKY_MCP].y * img_height)
            S0_prime['PINKY_TIP'] = (coordinates_info.landmark[mp_hands.HandLandmark.PINKY_TIP].x * img_width, coordinates_info.landmark[mp_hands.HandLandmark.PINKY_TIP].y * img_height)

            
            S0_prime['WRIST'] = ((S0_prime['INDEX_FINGER_TIP'][0]+S0_prime['THUMB_TIP'][0]+S0_prime['MIDDLE_FINGER_TIP'][0]+S0_prime['RING_FINGER_TIP'][0]+S0_prime['PINKY_TIP'][0])/5, (S0_prime['INDEX_FINGER_TIP'][1]+S0_prime['THUMB_TIP'][1]+S0_prime['MIDDLE_FINGER_TIP'][1]+S0_prime['RING_FINGER_TIP'][1]+S0_prime['PINKY_TIP'][1])/5)


            file = open('S0_Prime.txt', 'wt')
            file.write(str(S0_prime))
            file.close()
            # break

        def StateChange(Original, Target):

            #########################################################################################################################
            joint_cmd = [0 for _ in range(53)]
            # joint_cmd[52] = random.randint(-180,180)/60
            # #THUMB
            joint_cmd[45]= Angles['WT']
            # joint_cmd[43]= random.randint(0,180)/180
            # joint_cmd[44]= random.randint(0,180)/180
            # # joint_cmd[45]= random.randint(0,180)/180
            # joint_cmd[46]= random.randint(0,180)/180

            # # #INDEX_UNGERS
            joint_cmd[17]= Angles['WI']
            # joint_cmd[17]= (1-Angles['WI']/(0.5*3.142))
            # # joint_cmd[17]= random.randint(0,180)/180    
            # joint_cmd[18]= random.randint(0,180)/180    
            # joint_cmd[19]= random.randint(0,180)/180 
            # ##### joint_cmd[19]= 5 * np.pi/180 

            # # joint_cmd[20:26]=[random.randint(0,180)/180 for _ in range(6)]
            # # MIDDLE_FINGER
            joint_cmd[26]= Angles['WM']
            # joint_cmd[26]= (1-Angles['WM']/(0.5*3.142))
            # # joint_cmd[26]= random.randint(0,180)/180
            # joint_cmd[27]= random.randint(0,180)/180
            # joint_cmd[28]= random.randint(0,180)/180 
            # # #joint_cmd[28]= 5 * np.pi/180

            # # joint_cmd[29:35]=[np.pi*random.randint(0,180)/180 for _ in range(6)]

            # # # PINKY FINGER
            joint_cmd[35]= Angles['WR']
            # joint_cmd[35]= (1-Angles['WR']/(0.5*3.142))
            # # joint_cmd[35]= random.randint(0,180)/180
            # joint_cmd[36]= random.randint(0,180)/180
            # joint_cmd[37]= random.randint(0,180)/180

            p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)
            #RECORD 1ST FROM HERE
            time.sleep(0.2)
            #############################################################################################################################

            Original=copy.deepcopy(Target)
            return Original

        def AnglesCalc(State):
            v1 = (State['THUMB_TIP'][0]-State['WRIST'][0], State['THUMB_TIP'][1]-State['WRIST'][1])
            v2 = (State['INDEX_FINGER_TIP'][0]-State['WRIST'][0], State['INDEX_FINGER_TIP'][1]-State['WRIST'][1])
            
            Angles['TI'] = math.acos((v1[0]*v2[0]+v1[1]*v2[1])/(0.0001+((v1[0]**2+v1[1]**2)*(v2[0]**2+v2[1]**2))**0.5))

            v1 = (State['MIDDLE_FINGER_TIP'][0]-State['WRIST'][0], State['MIDDLE_FINGER_TIP'][1]-State['WRIST'][1])
            v2 = (State['INDEX_FINGER_TIP'][0]-State['WRIST'][0], State['INDEX_FINGER_TIP'][1]-State['WRIST'][1])
            
            Angles['IM'] = math.acos((v1[0]*v2[0]+v1[1]*v2[1])/(0.0001+((v1[0]**2+v1[1]**2)*(v2[0]**2+v2[1]**2))**0.5))

            v1 = (State['MIDDLE_FINGER_TIP'][0]-State['WRIST'][0], State['MIDDLE_FINGER_TIP'][1]-State['WRIST'][1])
            v2 = (State['RING_FINGER_TIP'][0]-State['WRIST'][0], State['RING_FINGER_TIP'][1]-State['WRIST'][1])
            
            Angles['MR'] = math.acos((v1[0]*v2[0]+v1[1]*v2[1])/(0.0001+((v1[0]**2+v1[1]**2)*(v2[0]**2+v2[1]**2))**0.5))

            v1 = (State['RING_FINGER_TIP'][0]-State['WRIST'][0], State['RING_FINGER_TIP'][1]-State['WRIST'][1])
            v2 = (State['PINKY_TIP'][0]-State['WRIST'][0], State['PINKY_TIP'][1]-State['WRIST'][1])
            
            Angles['RP'] = math.acos((v1[0]*v2[0]+v1[1]*v2[1])/(0.0001+((v1[0]**2+v1[1]**2)*(v2[0]**2+v2[1]**2))**0.5))

        # AnglesCalc(S0_prime)
        # print('Angles = ', Angles)
        Angles = {}
        AnglesCalc__(S0_prime)
        print('Angles_ = ', Angles)

        StateChange(TEMP_ORIGINAL, Angles)

        print('\nBefoe: ')
        # print('S0 = ', S0, '\nS0_prime = ', S0_prime)
        S0 = StateChange(S0, S0_prime)
        # print('\nS0 = ', S0, '\nS0_prime = ', S0_prime)

