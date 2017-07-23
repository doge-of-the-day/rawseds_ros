#!/usr/bin/python
import argparse
import numpy
import rosbag
import rospy
import geometry_msgs.msg


def main():
    parser = argparse.ArgumentParser(description='convert rawseeds ground truth odometry to ROS bag file')
    parser.add_argument('input', help='input rawseeds ground truth file')
    parser.add_argument('output', help='name of output bag file')
    parser.add_argument('--frame_id', type=str, default="world", help='frame_id for gt data')
    parser.add_argument('--topic', type=str, default="groundtruth", help='topic name for gt data')

    args = parser.parse_args()

    print('Input file: ' + args.input)
    print('Output file: ' + args.output)

    n_messages = 0

    with rosbag.Bag(args.output, 'w') as out_bag:
        with open(args.input) as in_file:
            for i, line in enumerate(in_file):
                print(line)

                # read line:
                row = line.split(',')

                t = float(row[0])   # timestamp in SECONDS (doc incorrectly says microseconds)
                x = float(row[1])   # in m
                y = float(row[2])   # in m
                th = float(row[3])  # in rad

                c_xx = float(row[4])
                c_xy = float(row[5])
                c_xt = float(row[6])
                c_yx = float(row[7])
                c_yy = float(row[8])
                c_yt = float(row[9])
                c_tx = float(row[10])
                c_ty = float(row[11])
                c_tt = float(row[12])

                # create message:
                msg = geometry_msgs.msg.PoseWithCovarianceStamped()
                msg.header.frame_id = args.frame_id
                msg.header.stamp = rospy.Time.from_sec(t)

                msg.pose.pose.position = geometry_msgs.msg.Point(x, y, 0.0)
                msg.pose.pose.orientation = geometry_msgs.msg.Quaternion(0, 0, numpy.sin(th/2), numpy.cos(th/2))

                msg.pose.covariance[0] = c_xx
                msg.pose.covariance[1] = c_yx
                msg.pose.covariance[5] = c_tx

                msg.pose.covariance[6] = c_xy
                msg.pose.covariance[7] = c_yy
                msg.pose.covariance[11] = c_ty

                msg.pose.covariance[12] = c_xt
                msg.pose.covariance[13] = c_yt
                msg.pose.covariance[17] = c_tt

                out_bag.write(args.topic, msg, rospy.Time.from_sec(t))

                n_messages += 1

    print("Conversion of %d messages done." % n_messages)
    return

if __name__ == "__main__":
    main()
