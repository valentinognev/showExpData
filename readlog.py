# -*- coding: utf-8 -*-
"""
Helper to decode binary logged sensor data from crazyflie2 with uSD-Card-Deck
"""
import argparse
from zlib import crc32
import struct
import numpy as np
import matplotlib.pyplot as plt

# extract null-terminated string
def _get_name(data, idx):
    endIdx = idx
    while data[endIdx] != 0:
        endIdx = endIdx + 1
    return data[idx:endIdx].decode("utf-8"), endIdx + 1

def decode(filename):
    # read file as binary
    with open(filename, 'rb') as f:
        data = f.read()

    # check magic header
    if data[0] != 0xBC:
        print("Unsupported format!")
        return

    # check CRC
    crc = crc32(data[0:-4])
    expected_crc, = struct.unpack('I', data[-4:])
    if crc != expected_crc:
        print("WARNING: CRC does not match!")

    # check version
    version, num_event_types = struct.unpack('HH', data[1:5])
    if version != 1 and version != 2:
        print("Unsupported version!", version)
        return

    result = dict()
    event_by_id = dict()

    # read header with data types
    idx = 5
    for _ in range(num_event_types):
        event_id, = struct.unpack('H', data[idx:idx+2])
        idx += 2
        event_name, idx = _get_name(data, idx)
        result[event_name] = dict()
        result[event_name]["timestamp"] = []
        num_variables, = struct.unpack('H', data[idx:idx+2])
        idx += 2
        fmtStr = "<"
        variables = []
        for _ in range(num_variables):
            var_name_and_type, idx = _get_name(data, idx)
            var_name = var_name_and_type[0:-3]
            var_type = var_name_and_type[-2]
            result[event_name][var_name] = []
            fmtStr += var_type
            variables.append(var_name)
        event_by_id[event_id] = {
            'name': event_name,
            'fmtStr': fmtStr,
            'numBytes': struct.calcsize(fmtStr),
            'variables': variables,
            }

    while idx < len(data) - 4:
        if version == 1:
            event_id, timestamp, = struct.unpack('<HI', data[idx:idx+6])
            idx += 6
        elif version == 2:
            event_id, timestamp, = struct.unpack('<HQ', data[idx:idx+10])
            timestamp = timestamp / 1000.0
            idx += 10
        event = event_by_id[event_id]
        fmtStr = event['fmtStr']
        eventData = struct.unpack(fmtStr, data[idx:idx+event['numBytes']])
        idx += event['numBytes']
        for v,d in zip(event['variables'], eventData):
            result[event['name']][v].append(d)
        result[event['name']]["timestamp"].append(timestamp)

    # remove keys that had no data
    for event_name in list(result.keys()):
        if len(result[event_name]['timestamp']) == 0:
            del result[event_name]

    # convert to numpy arrays
    for event_name in result.keys():
        for var_name in result[event_name]:
            result[event_name][var_name] = np.array(result[event_name][var_name])

    return result


if __name__ == "__main__":
    filename = '/home/valentin/robotics/crazyflie/log/log03'
    data = decode(filename)
    
    tim=data['fixedFrequency']['timestamp']
    tim = (tim-tim[0])/1000

    # plt.plot(tim,data['fixedFrequency']['ctrltarget.ax'])
    # plt.xlabel('Time [s]')
    # plt.show()
    # plt.plot(tim,data['fixedFrequency']['ctrltarget.ay'])
    # plt.xlabel('Time [s]')
    # plt.show()
    # plt.plot(tim,data['fixedFrequency']['ctrltarget.az'])
    # plt.xlabel('Time [s]')
    # plt.show()

    # plt.plot(tim,data['fixedFrequency']['controller.pitchRate']/100)
    # plt.plot(tim,data['fixedFrequency']['controller.r_pitch'])
    # plt.legend(['ctrl pitch rate ft', 'ctrl pitch rate'])
    # plt.xlabel('Time [s]')
    # plt.show()
    # plt.plot(tim,data['fixedFrequency']['controller.rollRate']/100)
    # plt.plot(tim,data['fixedFrequency']['controller.r_roll'])
    # plt.legend(['ctrl roll rate ft', 'ctrl roll rate'])
    # plt.xlabel('Time [s]')
    # plt.show()
    
    # plt.plot(tim,data['fixedFrequency']['controller.cmd_pitch']/1000)
    # plt.plot(tim,data['fixedFrequency']['controller.pitch'])
    # plt.plot(tim,data['fixedFrequency']['stateEstimate.pitch'])
    # plt.plot(tim,data['fixedFrequency']['ctrltarget.pitch'])
    # plt.legend(['cmd pitch', 'ctrl pitch', 'est pitch', 'targ pitch'])
    # plt.xlabel('Time [s]')
    # plt.show()
    # plt.plot(tim,data['fixedFrequency']['controller.cmd_roll']/1000)
    # plt.plot(tim,data['fixedFrequency']['controller.roll'])
    # plt.plot(tim,data['fixedFrequency']['stateEstimate.roll'])
    # plt.plot(tim,data['fixedFrequency']['ctrltarget.roll'])
    # plt.legend(['cmd roll', 'ctrl roll', 'est roll', 'targ roll'])
    # plt.xlabel('Time [s]')
    # plt.show()


    # plt.plot(tim,data['fixedFrequency']['ctrltarget.x'])
    # plt.plot(tim,data['fixedFrequency']['stateEstimate.x'])
    # plt.xlabel('Time [s]')
    # plt.show()
    # plt.plot(tim,data['fixedFrequency']['ctrltarget.y'])
    # plt.plot(tim,data['fixedFrequency']['stateEstimate.y'])
    # plt.xlabel('Time [s]')
    # plt.show()
    plt.plot(tim,data['fixedFrequency']['ctrltarget.z'])
    plt.plot(tim,data['fixedFrequency']['stateEstimate.z'])
    plt.xlabel('Time [s]')
    plt.show()
    plt.plot(tim,data['fixedFrequency']['posCtl.Zd'])
    plt.plot(tim,data['fixedFrequency']['posCtl.Zi'])
    plt.plot(tim,data['fixedFrequency']['posCtl.Zp'])
    plt.legend(['Zd', 'Zi', 'Zp'])
    plt.xlabel('Time [s]')
    plt.show()
   
    # plt.plot(tim,data['fixedFrequency']['posCtl.targetX'])
    # plt.plot(tim,data['fixedFrequency']['posCtl.bodyX'])
    # plt.xlabel('Time [s]')
    # plt.show()
    # plt.plot(tim,data['fixedFrequency']['posCtl.targetY'])
    # plt.plot(tim,data['fixedFrequency']['posCtl.bodyY'])
    # plt.plot(tim,data['fixedFrequency']['posCtl.Yd'])
    # plt.plot(tim,data['fixedFrequency']['posCtl.Yi'])
    # plt.plot(tim,data['fixedFrequency']['posCtl.Yp'])
    # plt.legend(['Yd', 'Yi', 'Yp'])
    # plt.xlabel('Time [s]')
    # plt.show()

    # plt.plot(tim,data['fixedFrequency']['ctrltarget.vx'])
    # plt.plot(tim,data['fixedFrequency']['stateEstimate.vx'])
    # plt.xlabel('Time [s]')
    # plt.show()
    # plt.plot(tim,data['fixedFrequency']['ctrltarget.vy'])
    # plt.plot(tim,data['fixedFrequency']['stateEstimate.vy'])
    # plt.xlabel('Time [s]')
    # plt.show()
    # plt.plot(tim,data['fixedFrequency']['ctrltarget.vz'])
    # plt.plot(tim,data['fixedFrequency']['stateEstimate.vz'])
    # plt.xlabel('Time [s]')
    # plt.show()


    # parser = argparse.ArgumentParser()
    # parser.add_argument("filename")
    # args = parser.parse_args()
    # data = decode(args.filename)
    print(data)