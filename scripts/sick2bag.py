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

    next_t = 0.0

    with rosbag.Bag(args.output, 'w') as out_bag:
        with open(args.input) as in_file:
            count = len(in_file.readlines())
            in_file.seek(0)
            
            content = in_file.readlines()
            for i in range(count):
                row = content[i].split(',')
                if i < count - 1:
                    next_t = float((content[i+1].split(','))[0])
                else:
                    next_t = 1. / 75.

                t        = float(row[0])  # timestamp in SECONDS (doc incorrectly says microseconds)
                n_ranges = int(row[1])

                ranges = map(float, row[3:])  # ranges in meters
                assert n_ranges == len(ranges)

                max_range = max(max_range, max(ranges))
                min_range = min(min_range, min(ranges))

                # create message:
                msg = sensor_msgs.msg.LaserScan()
                msg.header.frame_id = args.frame_id
                msg.header.stamp    = rospy.Time.from_sec(t)

                msg.angle_min       = -numpy.pi/2.0
                msg.angle_max       = +numpy.pi/2.0
                msg.angle_increment =  numpy.pi/(n_ranges - 1)
                
                msg.scan_time       = (next_t - t)
 
                msg.time_increment  = msg.scan_time / float(n_ranges)  # this should be fine now
                msg.range_min       = 0.015                            # in meters, assuming this goes ok with the systematic error of the sensor
                msg.range_max       = 80.0                             # in meters

                msg.ranges = ranges

                out_bag.write(args.topic, msg, rospy.Time.from_sec(t))

                n_messages += 1

                state = 'Progress: ' + '{0:.2f}'.format(i / (1.0 * count) * 100) + '%'
                sys.stdout.write('%s\r' % state)



    print
    print("Conversion of %d messages done." % n_messages)
    print("Min range seen: %f" % min_range)
    print("Max range seen: %f" % max_range)

if __name__ == "__main__":
    main()