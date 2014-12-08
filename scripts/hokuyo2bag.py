#!/usr/bin/python
import argparse
import numpy
import rosbag
import rospy
import sensor_msgs.msg


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

    with rosbag.Bag(args.output, 'w') as out_bag:
        with open(args.input) as input_file:
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

                msg.ranges = ranges

                out_bag.write(args.topic, msg, rospy.Time.from_sec(t))

                n_messages += 1

    print("Conversion of %d messages done." % n_messages)
    print("Min range seen: %f" % min_range)
    print("Max range seen: %f" % max_range)


if __name__ == "__main__":
    main()