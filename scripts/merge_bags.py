#!/usr/bin/python
import argparse
import numpy
import rosbag
import rospy
import sensor_msgs.msg
import sys
import os


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
    parser.add_argument('-s', '--split_size', help='split size if bag should be splitted')

    args = parser.parse_args()

    print('Input files:  ' + str(args.input))
    print('Output file: ' + args.output)
    if args.split_size:
        print("Splitting every " + str(int(args.split_size) / (1024.0 * 1024.0))) + "MB"

    data = []
    next = []
    message_count = 0
    split_size = 0
    if args.split_size:
        split_size = int(args.split_size)

    for idx, in_file in enumerate(args.input):
        print('Creating generators for: ' + in_file)
        bagfile = rosbag.Bag(in_file)
        messages = bagfile.read_messages()
        message_count += bagfile.get_message_count()
        next.append(messages.next())
        data.append(messages)
        print('Created generators for: ' + in_file)

    print('Starting to merge...')
    n_messages = 0

    filename, file_extension = os.path.splitext(args.output)
    path = filename
    split = 0
    if split_size != 0:
        path += "_" + str(split)



    print('Writing to file ' + path + file_extension + '.')
    out_bag = rosbag.Bag(path + file_extension, 'w')
    while True:
        if out_bag.size >= split_size and split_size != 0:
            out_bag.close()
            split += 1
            path = filename + "_" + str(split)  
            out_bag = rosbag.Bag(path + file_extension, 'w')
            print('Writing to file ' + path + file_extension + '.')

        state = 'Progress: ' + '{0:.2f}'.format((n_messages + 1) / (1.0 * message_count) * 100) + '%'
        sys.stdout.write('%s\r' % state)

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

    out_bag.close()



    print(str(n_messages) + " messages merged in total")

if __name__ == "__main__":
    main()
