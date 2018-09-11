#!/usr/bin/python

import argparse
import numpy
import rosbag
import rospy
import sensor_msgs.msg
import sys
import os
import tf
from pprint import pprint

def main():
    rospy.init_node('playlive')

    parser = argparse.ArgumentParser(description='Check file.')
    parser.add_argument('-b', '--bag', help='name of input bag file')
    args, unkown = parser.parse_known_args()

    if args.bag is None: 
        print("Need a bag file.")
        exit(0)

    print('Input files:  ' + str(args.bag))

    in_bag  = rosbag.Bag(args.bag)
    in_msgs = in_bag.read_messages()
    in_msgs_count = in_bag.get_message_count()

    max_time = None
    min_time = None

    pubs = {}
    for topic, msg, time in in_msgs:
        if hasattr(msg, 'header'):
            if max_time is None:
                max_time = msg.header.stamp
            if min_time is None:
                min_time = msg.header.stamp
            
            if msg.header.stamp < min_time:
                min_time = msg.header.stamp
            if msg.header.stamp > max_time:
                max_time = msg.header.stamp

        if topic == "/tf":
            for t in msg.transforms:
                if max_time is None:
                    max_time = t.header.stamp
                if min_time is None:
                    min_time = t.header.stamp
                
                if t.header.stamp < min_time:
                    min_time = t.header.stamp
                if t.header.stamp > max_time:
                    max_time = t.header.stamp
        if topic not in pubs:
            pubs[topic] = rospy.Publisher(topic, type(msg), latch='map' in topic, queue_size=0)


    print("min_time : " + str(min_time))
    print("max_time : " + str(max_time))

    # get the time offsets 
    ts = []
    last = None
    in_msgs = in_bag.read_messages()
    for topic, msg, time in in_msgs:
        if hasattr(msg, 'header'):
            if last is None:
                last = msg.header.stamp

            ts.append(msg.header.stamp - last)

        if topic == "/tf":
            min_time = None

            for t in msg.transforms:
                if min_time is None:
                    min_time = t.header.stamp
                if t.header.stamp < min_time:
                    min_time = t.header.stamp

                if last is None:
                    last = t.header.stamp
            
            ts.append(min_time - last)
            last = min_time

    start_time = rospy.Time.now()
    index = 0
    in_msgs = in_bag.read_messages()
    print("Start playing!")
    for topic, msg, time in in_msgs:
        rospy.sleep(ts[index])
        now = rospy.Time.now()

        if hasattr(msg, 'header'):
            msg.header.stamp = now;

        if topic == "/tf":
            for t in msg.transforms:
                diff = t.header.stamp - min_time
                t.header.stamp = now + diff

        pubs[topic].publish(msg)
        if rospy.is_shutdown():
                break
        
        state = 'Progress: ' + '{0:.2f}'.format((index + 1) / (1.0 * in_msgs_count) * 100) + '%'
        sys.stdout.write('%s\r' % state)
        index += 1

if __name__ == "__main__":
    main()
