#!/usr/bin/python
import argparse
import numpy
import rosbag
import rospy
import sensor_msgs.msg


def main():
    parser = argparse.ArgumentParser(description='merge multiple bag files into one')
    parser.add_argument('-o', '--output', help='name of output bag file')
    parser.add_argument('-i', '--input', nargs='+', help='list of input bag files')
    args = parser.parse_args()

    print('Input files:  ' + str(args.input))
    print('Output file: ' + args.output)

    data = []

    for in_file in args.input:
        print('reading: ' + in_file)
        for topic, msg, t in rosbag.Bag(in_file).read_messages():
            data.append([topic, msg, t])

        print('done reading: ' + in_file)

    print('sorting...')
    data_sorted = sorted(data, key=lambda x: x[2]) # sort by time
    print('done sorting.')

    print('writing...')
    with rosbag.Bag(args.output, 'w') as out_bag:
        for t in data_sorted:
            out_bag.write(t[0], t[1], t[2])
    print('done writing.')


if __name__ == "__main__":
    main()