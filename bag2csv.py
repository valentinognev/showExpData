#!/usr/bin/python
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import rosbag
import pandas as pd
import numpy as np
from tf.transformations import euler_from_quaternion
from tqdm import tqdm
import csv
import io

buffer_core = tf2_ros.BufferCore()

import argparse

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

# Add the arguments
my_parser.add_argument("--LD", type=str2bool, nargs='?', metavar='BOOL',
                        const=True, default=False,
                        help="looking down mode.")                        

def convert2csv(bag_file, LD=True):
    file_out = bag_file.strip('.bag')+'.csv'
    print('processing file : {}. LD={}'.format(bag_file, "True" if LD else "False"))
    print('output file : {}'.format(file_out))
    with io.open(file_out,'wb') as fid:
    # with open(file_out, 'w', encoding='utf8') as fid:
        writer = csv.writer(fid)
        if LD:
            mrOrd = [4, 5, 6, 7, 8, 9, 10 , 11, 12, 13, 14, 15, 0, 1, 2, 3]
        else:
            mrOrd = [4, 3, 2, 1, 0, 15, 14 , 13, 12, 11, 10, 9, 8, 7, 6, 5]

        columns = ['tick']
        columns = columns+[b'mr18.m'+str(mrOrd_i) for mrOrd_i in mrOrd]
        # columns = list(mr)
        columns.append(b"stateEstimate.x")
        columns.append(b"stateEstimate.y")
        columns.append(b"stateEstimate.yaw")
        writer.writerow(columns)

        i=0
        prev_ls_time = None
        with rosbag.Bag(bag_file, 'r') as bag:

            L = bag.get_message_count()
            pbar = tqdm(total=L)
            for topic, msg, t in bag.read_messages(['/fa_node/mr18', '/tf']):
                pbar.update(1)
                if '/tf' == topic:
                    for T in msg.transforms:
                        buffer_core.set_transform(T, "default_authority")
                        i += 1
                elif '/fa_node/mr18' == topic:
                    if i<2:
                        prev_ls_time = msg.header.stamp
                        continue
                    elif prev_ls_time is None:
                        prev_ls_time = msg.header.stamp
                        continue
                    
                    try:
                        tfSpamped = buffer_core.lookup_transform_core('world', 'mr18_frame', prev_ls_time)
                    except Exception as e:
                        print('----- tf {} not found ------'.format(i))
                        continue

                    _, _, yaw = euler_from_quaternion([tfSpamped.transform.rotation.x, tfSpamped.transform.rotation.y, tfSpamped.transform.rotation.z, tfSpamped.transform.rotation.w])

                    row = [int(prev_ls_time.to_sec()*1000.0)]+[round(round(r*1000.0)) for r in msg.ranges]+[tfSpamped.transform.translation.x, tfSpamped.transform.translation.y, np.rad2deg(yaw)]
                    writer.writerow([b"{}".format(s) for s in row])
                    prev_ls_time = msg.header.stamp

if __name__=="__main__":
    args = my_parser.parse_args()
    inFile = args.inFile
    convert2csv(inFile, LD=args.LD)

