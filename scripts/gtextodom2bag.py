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
    parser.add_argument('--frame_id', type=str, default="/odom_gt", help='frame_id for ROS message')
    parser.add_argument('--topic', type=str, default="/odom_gt", help='topic name for ROS message')


    args = parser.parse_args()

    print('Input file:  ' + args.input)
    print('Output file: ' + args.output)
    print('frame_id:    ' + args.frame_id)
    print('topic:       ' + args.topic)

    frame_id = '/odom_gt'
    topic = '/odom_gt'

    n_messages = 0

    with rosbag.Bag(args.output, 'w') as out_bag:
        with open(args.input) as in_file:
            count = len(in_file.readlines())
            in_file.seek(0)
            for i, line in enumerate(in_file):
                # read line:
                row = line.split(',')

                t = float(row[0])  # timestamp in SECONDS
                x = float(row[1])
                y = float(row[2])
                th = float(row[3])

                # create odometry message:
                msg = nav_msgs.msg.Odometry()
                msg.header.frame_id = args.frame_id
                msg.header.stamp = rospy.Time.from_sec(t)

                msg.pose.pose.position = geometry_msgs.msg.Point(x, y, 0)
                msg.pose.pose.orientation = geometry_msgs.msg.Quaternion(0, 0, numpy.sin(th/2), numpy.cos(th/2))
                out_bag.write(args.topic, msg, rospy.Time.from_sec(t))

                odom_trans = geometry_msgs.msg.TransformStamped()
                odom_trans.header = msg.header
                odom_trans.child_frame_id = "base_link"

                odom_trans.transform.translation = geometry_msgs.msg.Vector3(x, y, 0)
                odom_trans.transform.rotation = msg.pose.pose.orientation

                tf_msg = tf.msg.tfMessage()
                tf_msg.transforms.append(odom_trans)

                out_bag.write("/tf", tf_msg, rospy.Time.from_sec(t))

                n_messages += 1

                state = 'Progress: ' + '{0:.2f}'.format((i + 1) / (1.0 * count) * 100) + '%'
                sys.stdout.write('%s\r' % state)



    print("Conversion of %d messages done." % n_messages)
    return


if __name__ == "__main__":
    main()