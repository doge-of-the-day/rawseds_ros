#!/usr/bin/python
import argparse
import numpy
import rosbag
import rospy
import sensor_msgs.msg
import sys

def main():
    parser = argparse.ArgumentParser(description='convert rawseeds sick data to ROS bag file')
    parser.add_argument('input', help='input rawseeds file')
    parser.add_argument('output', help='name of output bag file')
    parser.add_argument('--frame_id', type=str, default="sick", help='frame_id for ROS message')
    parser.add_argument('--topic', type=str, default="sick", help='topic name for ROS message')

    args = parser.parse_args()

    print('Input file:  ' + args.input)
    print('Output file: ' + args.output)
    print('frame-id:    ' + args.frame_id)
    print('topic:       ' + args.topic)

    n_messages = 0

    max_range = 0.
    min_range = 100.

    with rosbag.Bag(args.output, 'w') as out_bag:
        with open(args.input) as in_file:
            count = len(in_file.readlines())
            in_file.seek(0)
            for i, line in enumerate(in_file):
                # read line:
                row = line.split(',')

                t = float(row[0])  # timestamp in SECONDS (doc incorrectly says microseconds)
                n_ranges = int(row[1])
                angular_offset = float(row[2]) # in 1/4 degree
                angular_offset *= numpy.pi/(4.*180.)
                ranges = map(float, row[3:])  # ranges in meters
                assert n_ranges == len(ranges)

                max_range = max(max_range, max(ranges))
                min_range = min(min_range, min(ranges))

                # create message:
                msg = sensor_msgs.msg.LaserScan()
                msg.header.frame_id = args.frame_id
                msg.header.stamp = rospy.Time.from_sec(t)

                msg.angle_min = -numpy.pi/2 + angular_offset
                msg.angle_max = +numpy.pi/2 + angular_offset
                msg.angle_increment = numpy.pi/180
                msg.time_increment = 0.  # don't know, actually :/
                msg.scan_time = 0.1    # time between consecutive scans in seconds
                msg.range_min = 0.0    # in meters
                msg.range_max = 81.91  # in meters

                msg.ranges = ranges

                out_bag.write(args.topic, msg, rospy.Time.from_sec(t))

                n_messages += 1

                str = 'Progress: ' + '{0:.2f}'.format(i / (1.0 * count) * 100) + '%'
                sys.stdout.write('%s\r' % str)

    print
    print("Conversion of %d messages done." % n_messages)
    print("Min range seen: %f" % min_range)
    print("Max range seen: %f" % max_range)

if __name__ == "__main__":
    main()