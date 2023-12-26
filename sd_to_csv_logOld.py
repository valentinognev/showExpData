#!/usr/bin/env python3

import argparse
import csv
import logging
import os
import struct
from zlib import crc32

import coloredlogs
import numpy as np


# ATTENTION: This file needs to be run with Python 3. However, the file is run automatically when plotting recording
# from microSD card connected to the computer.

def decode(filename):
    # read file as binary
    fil_obj = open(filename, 'rb')
    fil_con = fil_obj.read()
    fil_obj.close()

    # get file size to forecast output array
    stat_info = os.stat(filename)

    # process file header
    set_width = struct.unpack('B', fil_con[:1])
    set_names = []
    idx = 1
    for ii in range(0, set_width[0]):
        start_idx = idx
        while True:
            if fil_con[idx] == ','.encode('ascii')[0]:
                break
            idx += 1
        # print(fil_con[start_idx:idx], start_idx, idx)
        set_names.append(fil_con[start_idx:idx])
        idx += 1
    # print("[CRC] of file header:", end="")
    crc_val = crc32(fil_con[0:idx + 4]) & 0xffffffff
    crc_errors = 0
    if crc_val == 0xffffffff:
        pass  # print("\tOK\t["+hex(crc_val)+"]")
    else:
        crc_errors += 1
    #    print("\tERROR\t["+hex(crc_val)+"]")
    # crc_errors += 1
    offset = idx + 4

    # process data sets
    set_con = np.zeros(stat_info.st_size)  # upper bound...
    idx = 0
    fmt_str = "<"
    for setName in set_names:
        fmt_str += chr(setName[-2])
    set_bytes = struct.calcsize(fmt_str)
    while offset < len(fil_con):
        set_number = struct.unpack('B', fil_con[offset:offset + 1])
        offset += 1
        for ii in range(set_number[0]):
            set_con[idx:idx + set_width[0]] = np.array(struct.unpack(fmt_str, fil_con[offset:set_bytes + offset]))
            offset += set_bytes
            idx += set_width[0]
        crc_val = crc32(fil_con[offset - set_bytes * set_number[0] - 1:offset + 4]) & 0xffffffff
        # print("[CRC] of data set:", end="")
        if crc_val == 0xffffffff:
            pass
        #       print("\tOK\t["+hex(crc_val)+"]")
        else:
            #      print("\tERROR\t["+hex(crc_val)+"]")
            crc_errors += 1
        offset += 4
    if not crc_errors:
        pass
    #   print("[CRC] no errors occurred:\tOK")
    else:
        pass
    #  print("[CRC] {0} errors occurred:\tERROR".format(crc_errors))

    # remove not required elements and reshape as matrix
    set_con = np.reshape(set_con[0:idx], (set_width[0], idx // set_width[0]), 'f')

    # create output dictionary
    output = {}
    for ii in range(set_width[0]):
        output[set_names[ii][0:-3].decode("utf-8").strip()] = set_con[ii]
    return output


def main():
    # parser = argparse.ArgumentParser()
    # parser.add_argument('file')
    # args = parser.parse_args()
    # file_path = args.file
    file_path = '/home/valentin/crazyflie/Recordings/testStep/logOLD06'
    try:
        d = decode(file_path)
    except IndexError:
        logging.fatal('Something went wrong during file decoding. Check whether you\'re actually decoding a log file.')
        exit(1)
    file_dir, filename = os.path.split(file_path)
    os.chdir(file_dir)
    with open(file_path + '.csv', 'w') as csv_file:
        keys = d.keys()
        writer = csv.DictWriter(csv_file, fieldnames=keys)
        writer.writeheader()
        longest_log_length = len(max(d.values(), key=len))
        for i in range(longest_log_length):
            writer.writerow({k: d[k][i] for k in keys})
    logging.info('Successfully converted log file into CSV format.')


if __name__ == '__main__':
    coloredlogs.install()

    main()

    # ls /home/roee/records/Zvuv/ZVUV1_second_batch/ | grep -v txt | xargs -I{} bash -c 'FILE={} && ./sd_log_to_csv.py  /home/roee/records/Zvuv/ZVUV1_second_batch/$FILE'