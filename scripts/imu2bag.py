#!/usr/bin/python
import argparse
import numpy
import rosbag
import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import tf
import sys

def main():
    parser = argparse.ArgumentParser(description='convert rawseeds imu data to ROS bag file')
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
            count = len(input_file.readlines())
            input_file.seek(0)
            for i, line in enumerate(input_file):
                # read line:
                row = line.split(',')

                t = float(row[0])           # 0: timestamp in SECONDS (doc incorrectly says microseconds)
                s = int(row[1])             # 1: sample count
                acc = map(float, row[2:5])  # 2,3,4: acceleration
                vel = map(float, row[5:8])  # 5,6,7: velocity
                mag = map(float, row[8:11]) # 8,9,10: magnetic field
                rot = map(float, row[11:20])# 11,...,19: rotation matrix
                rot_mat = numpy.array([[rot[0], rot[1], rot[2], 0.],
                                       [rot[3], rot[4], rot[5], 0.],
                                       [rot[6], rot[7], rot[8], 0.],
                                       [0.,0.,0.,1.]])


                # create message:
                av_msg = sensor_msgs.msg.Imu()
                av_msg.header.frame_id = args.frame_id
                av_msg.header.stamp = rospy.Time.from_sec(t)
                av_msg.angular_velocity.x = vel[0]
                av_msg.angular_velocity.y = vel[1]
                av_msg.angular_velocity.z = vel[2]
                av_msg.linear_acceleration.x = acc[0]
                av_msg.linear_acceleration.y = acc[1]
                av_msg.linear_acceleration.z = acc[2]
                quat = tf.transformations.quaternion_from_matrix(rot_mat)
                av_msg.orientation = geometry_msgs.msg.Quaternion(quat[0], quat[1], quat[2], quat[3])

                m_msg = sensor_msgs.msg.MagneticField()
                m_msg.header = av_msg.header
                m_msg.magnetic_field.x = mag[0]
                m_msg.magnetic_field.y = mag[1]
                m_msg.magnetic_field.z = mag[2]

                out_bag.write(args.topic, av_msg, rospy.Time.from_sec(t))
                out_bag.write(args.topic, m_msg, rospy.Time.from_sec(t))

                n_messages += 1

                state = 'Progress: ' + '{0:.2f}'.format((i + 1) / (1.0 * count) * 100) + '%'
                sys.stdout.write('%s\r' % state)

    print
    print("Conversion of %d messages done." % n_messages)
    print("Min range seen: %f" % min_range)
    print("Max range seen: %f" % max_range)


if __name__ == "__main__":
    main()