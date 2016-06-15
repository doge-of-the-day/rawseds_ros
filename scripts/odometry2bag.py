#!/usr/bin/python
import argparse
import numpy
import rosbag
import rospy
import geometry_msgs.msg
import nav_msgs.msg
import tf
import tf.msg
import sys

def main():
    parser = argparse.ArgumentParser(description='convert rawseeds odometry data to ROS bag file')
    parser.add_argument('input', help='input rawseeds file')
    parser.add_argument('output', help='name of output bag file')

    args = parser.parse_args()

    print('Input file:  ' + args.input)
    print('Output file: ' + args.output)

    frame_id = '/odom'
    topic = '/odom'

    n_messages = 0

    with rosbag.Bag(args.output, 'w') as out_bag:
        with open(args.input) as in_file:
            count = len(in_file.readlines())
            in_file.seek(0)
            for i, line in enumerate(in_file):
                # read line:
                row = line.split(',')

                t = float(row[0])  # timestamp in SECONDS
                counter = int(row[1])
                ticks_right = int(row[2])
                ticks_left = int(row[3])
                x = float(row[4])
                y = float(row[5])
                th = float(row[6])

                # create odometry message:
                msg = nav_msgs.msg.Odometry()
                msg.header.frame_id = frame_id
                msg.header.stamp = rospy.Time.from_sec(t)

                msg.pose.pose.position = geometry_msgs.msg.Point(x, y, 0)
                msg.pose.pose.orientation = geometry_msgs.msg.Quaternion(0, 0, numpy.sin(th/2), numpy.cos(th/2))
                out_bag.write(topic, msg, rospy.Time.from_sec(t))

                odom_trans = geometry_msgs.msg.TransformStamped()
                odom_trans.header = msg.header
                odom_trans.child_frame_id = "base_link"

                odom_trans.transform.translation = geometry_msgs.msg.Vector3(x, y, 0)
                odom_trans.transform.rotation = msg.pose.pose.orientation

                tf_msg = tf.msg.tfMessage()
                tf_msg.transforms.append(odom_trans)

                out_bag.write("/tf", tf_msg, rospy.Time.from_sec(t))

                n_messages += 1

                state = 'Progress: ' + '{0:.2f}'.format(i / (1.0 * count) * 100) + '%'
                sys.stdout.write('%s\r' % state)

    print
    print("Conversion of %d messages done." % n_messages)
    return


if __name__ == "__main__":
    main()