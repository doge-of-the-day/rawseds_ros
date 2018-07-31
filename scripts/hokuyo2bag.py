#!/usr/bin/python
import argparse
import numpy
import rosbag
import rospy
import sensor_msgs.msg
import sys

def main():
    parser = argparse.ArgumentParser(description='convert rawseeds hokuyo data to ROS bag file')
    parser.add_argument('input', help='input rawseeds file')
    parser.add_argument('output', help='name of output bag file')
    parser.add_argument('--frame_id', type=str, default="hokuyo", help='frame_id for ROS message')
    parser.add_argument('--topic', type=str, default="hokuyo", help='topic name for ROS message')

    args = parser.parse_args()

    print('Input file:  ' + args.input)
    print('Output file: ' + args.output)
    print('frame-id:    ' + args.frame_id)
    print('topic:       ' + args.topic)

    n_messages = 0

    max_range = 0.
    min_range = 100.

    last_t = 0.0

    n_ranges = 681
    with rosbag.Bag(args.output, 'w') as out_bag:
        with open(args.input) as input_file:
            count = len(input_file.readlines())
            input_file.seek(0)
            for i, line in enumerate(input_file):
                # read line:
                row = line.split(',')

                t = float(row[0])  # timestamp in SECONDS (doc incorrectly says microseconds)
                ranges = map(float, row[1:])  # ranges in meters

                max_range = max(max_range, max(ranges))
                min_range = min(min_range, min(ranges))

                # create message:
                msg = sensor_msgs.msg.LaserScan()
                msg.header.frame_id = args.frame_id
                msg.header.stamp = rospy.Time.from_sec(t)

                d_angle = (2*numpy.pi)/1024

                msg.angle_min = -340*d_angle
                msg.angle_max = +340*d_angle
                msg.angle_increment = d_angle
                msg.time_increment = 0.  # don't know, actually :/
                msg.scan_time = 0.1  # time between consecutive scans in seconds
                msg.range_min = 0.0 # in meters
                msg.range_max = 5.6 # in meters
               
                if(last_t == 0.0):
                    msg.scan_time    = 1. / 10.    # time between consecutive scans in seconds
                else:
                    msg.scan_time = (t - last_t)

                msg.time_increment  = msg.scan_time / (n_ranges - 1)  # this should be fine now
 
                msg.ranges = ranges

                out_bag.write(args.topic, msg, rospy.Time.from_sec(t))

                last_t = t

                n_messages += 1

                state = 'Progress: ' + '{0:.2f}'.format((i + 1) / (1.0 * count) * 100) + '%'
                sys.stdout.write('%s\r' % state)

    print
    print("Conversion of %d messages done." % n_messages)
    print("Min range seen: %f" % min_range)
    print("Max range seen: %f" % max_range)


if __name__ == "__main__":
    main()