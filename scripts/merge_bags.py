#!/usr/bin/python
import argparse
import numpy
import rosbag
import rospy
import sensor_msgs.msg
import sys


def find_next_index(next):
    t_next = rospy.Time.from_sec(sys.maxsize)
    i_next = -1

    for idx, t in enumerate(next):
        if t[2] < t_next:
            t_next = t[2]
            i_next = idx
    return i_next


def main():
    parser = argparse.ArgumentParser(description='merge multiple bag files into one')
    parser.add_argument('-o', '--output', help='name of output bag file')
    parser.add_argument('-i', '--input', nargs='+', help='list of input bag files')

    args = parser.parse_args()

    print(args.input)

    print('Input files:  ' + str(args.input))
    print('Output file: ' + args.output)

    data = []
    next = []

    for idx, in_file in enumerate(args.input):
        print('creating generators for: ' + in_file)
        messages = rosbag.Bag(in_file).read_messages()
        next.append(messages.next())
        data.append(messages)
        print('done creating generators for: ' + in_file)

    print('starting to merge...')
    n_messages = 0
    with rosbag.Bag(args.output, 'w') as out_bag:
        while True:
            if n_messages % 10000 == 0:
                print(str(n_messages) + " messages merged")

            i = find_next_index(next)
            if i == -1:
                break

            out_bag.write(next[i][0], next[i][1], next[i][2])
            n_messages += 1

            try:
                next[i] = data[i].next()
            except StopIteration:
                # there is no next element... remove this input source
                print('end of input file: ' + args.input[i])
                del args.input[i]
                del next[i]
                del data[i]

    print(str(n_messages) + " messages merged in total")

if __name__ == "__main__":
    main()