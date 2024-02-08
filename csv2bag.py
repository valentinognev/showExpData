#!/usr/bin/python3
# set: conda activate foxglove
import rosbag 
import rospy
import pandas as pd
import glob
from os.path import exists

from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Quaternion
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import Marker

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import Time, ColorRGBA, Int16, Int32, Float32
import numpy as np

from matplotlib import pyplot as plt

import argparse
import os
import sys
from tqdm import tqdm

def str2bool(v):
    if isinstance(v, bool):
       return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

# Create the parser
my_parser = argparse.ArgumentParser(description='csv to bag converter for zvuv')
# Add the arguments
my_parser.add_argument('inFile',
                       metavar='path',
                       type=str,
                       help='the file to convert')

# my_parser.add_argument('--posSigX',
#                        metavar='varName', nargs='?',
#                        const='stateEstimate.x', default='stateEstimate.x',
#                        type=str,
#                        help='Position signal X')

# my_parser.add_argument('--posSigY', nargs='?',
#                        metavar='varName', 
#                        const='stateEstimate.y', default='stateEstimate.y',
#                        type=str,
#                        help='Position signal Y')


# Add the arguments
my_parser.add_argument("--LD", type=str2bool, nargs='?', metavar='BOOL',
                        const=True, default=False,
                        help="looking down mode.")

# Add the arguments
my_parser.add_argument("--ZO", type=str2bool, nargs='?', metavar='BOOL',
                        const=False, default=False,
                        help="Zero odometry.")

# Add the arguments
my_parser.add_argument("--ODOM", metavar='N', type=int, nargs='?',
                        const=0, default=0,
                        help="stateEstimate.x (0, default), kalman.stateX (1), kalman.stateX_f (2)")
                  
LD = True
ZO = True
ODOM = 0
dirName = '/home/valentin/crazyflie/Recordings/tube/'

fileNameList = glob.glob(dirName+'sd_*')
outInds=[]
for i in range(len(fileNameList)):
    res = [j for j in fileNameList if fileNameList[i].split('/')[-1].split('.')[0]+'.bag' in j]
    if fileNameList[i].find('.csv')==-1 or len(res)!=0:
        outInds.append(fileNameList[i])

for value in outInds:
    fileNameList.remove(value)

for inFile in fileNameList:
   
    print('csv2bag.py -- infile = {}'.format(inFile))
    file_name = inFile

    assert(ODOM in list(range(3)))
    odomSigX = ['stateEstimate.x', 'kalman.stateX', 'kalman.stateX_f']
    odomSigY = ['stateEstimate.y', 'kalman.stateY', 'kalman.stateY_f']
    odomSigYaw = ['stateEstimate.yaw', 'stateEstimate.yaw', 'kalman.stateYaw_f']
    yawSf = [np.deg2rad(1), np.deg2rad(1), 1.0]
    print('odometry signals (x,y,yaw)=('+odomSigX[ODOM]+','+odomSigY[ODOM]+','+ odomSigYaw[ODOM] +')')


    if LD:
        print('LD =True')
    else:
        print('LD =False')


    data = pd.read_csv(file_name, delimiter = ',')
    data = data.iloc[::3]
    L, W = data.shape

    col_head = list(data.columns)

    if LD:
        mrOrd = [4, 5, 6, 7, 8, 9, 10 , 11, 12, 13, 14, 15, 0, 1, 2, 3]
    else:
        mrOrd = [4, 3, 2, 1, 0, 15, 14 , 13, 12, 11, 10, 9, 8, 7, 6, 5]

    mr = ['mr18.m'+str(mrOrd_i) for mrOrd_i in mrOrd]

    file_name_out_zo = file_name.strip('.csv')+'_ZO.bag'
    file_name_out = file_name.strip('.csv')+'.bag'

    print('csv2bag.py -- file_name_out = {}'.format(file_name_out))
    print('csv2bag.py -- file_name_out_zo = {}'.format(file_name_out_zo))


    Freq = 50.0  # Hz
    T_ = 1/Freq

    ls_msg = LaserScan()
    ls_msg.header.seq = 0
    ls_msg.header.frame_id = 'mr18_frame'
    ls_msg.angle_min = 0.0
    ls_msg.angle_max = np.deg2rad(337.5)
    ls_msg.angle_increment = np.deg2rad(22.5)
    ls_msg.range_min= 0.05
    ls_msg.range_max= 4.0

    tf_msg = TFMessage()
    t = TransformStamped()
    t.header.seq = 0
    t.header.frame_id = 'world'
    t.child_frame_id = 'mr18_frame'
    tf_msg.transforms.append(t)

    from arrow_marker import arw_mrkr_class
    arw_mrkr = arw_mrkr_class()

    clock_msg = Clock()
    time = 0.0

    firstRun = True
    pbar = tqdm(total=L)
    with rosbag.Bag(file_name_out_zo, 'w') as bag_zo:
        with rosbag.Bag(file_name_out, 'w') as bag:
            for index, row in data.iterrows():
                pbar.update(1)
                t_curr = row[['tick']].values*0.001

                if firstRun:
                    time = t_curr
                    firstRun = False

                while t_curr > time:
                    clock_msg.clock = rospy.Time(time)
                    bag.write('/clock', clock_msg, t=rospy.Time(time))
                    if ZO:
                        bag_zo.write('/clock', clock_msg, t=rospy.Time(time))
                    time += T_

                ls_msg.ranges = list(row[mr].values*0.001)
                ls_msg.header.stamp = rospy.Time(t_curr)

                t.header.stamp = ls_msg.header.stamp
                t.transform.rotation = Quaternion(*quaternion_from_euler(0, 0, yawSf[ODOM]*(row[[odomSigYaw[ODOM]]].values), 'rxyz'))
                t.transform.translation.x = row[[odomSigX[ODOM]]].values
                t.transform.translation.y = row[[odomSigY[ODOM]]].values
                
                bag.write('/fa_node/mr18',ls_msg, t=ls_msg.header.stamp)
                bag.write('/tf', tf_msg, t=t.header.stamp)

                vxy_cmd = row[['state_machine.vx_sp', 'state_machine.vy_sp']].values
                arw_mrkr.update_arrow_mrkr(vxy_cmd[0], vxy_cmd[1], rospy.Time(t_curr))
                bag.write('/v_cmd', arw_mrkr.arroy_mrkr, t=rospy.Time(t_curr))

                bag.write('/squal',Int16(int(row['motion.squal'])), t=rospy.Time(t_curr))
                bag.write('/motor1',Int32(int(row['motor.m1'])), t=rospy.Time(t_curr))
                bag.write('/HL_ratio',Float32(row['FF_Angle.HL_ratio']), t=rospy.Time(t_curr))
                bag.write('/shutter',Float32(row['motion.shutter']), t=rospy.Time(t_curr))
                bag.write('/range_zrange',Float32(row['range.zrange']*0.001), t=rospy.Time(t_curr))
                bag.write('/mr18_m16_zrange',Float32(-row['mr18.m16']*0.001), t=rospy.Time(t_curr))
                bag.write('/pm_vbat',Float32(row['pm.vbat']), t=rospy.Time(t_curr))
                

                if ZO:
                    t.transform.translation.x = 0.0
                    t.transform.translation.y = 0.0
                    bag_zo.write('/fa_node/mr18',ls_msg, t=ls_msg.header.stamp)
                    bag_zo.write('/tf', tf_msg, t=t.header.stamp)


                t.header.seq += 1
                ls_msg.header.seq += 1
                



