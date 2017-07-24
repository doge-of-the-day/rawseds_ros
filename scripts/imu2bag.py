#!/usr/bin/python
import argparse
import numpy
import rosbag
import rospy
import geometry_msgs.msg
import rawseeds_ros.msg
import tf
import sys

def main():
    parser = argparse.ArgumentParser(description='convert rawseeds imu data to ROS bag file')
    parser.add_argument('input', help='input rawseeds file')
    parser.add_argument('output', help='name of output bag file')
    parser.add_argument('--frame_id', type=str, default="imu", help='frame_id for ROS message')
    parser.add_argument('--topic', type=str, default="rawseeds_imu", help='topic name for ROS message')

    args = parser.parse_args()

    print('Input file:  ' + args.input)
    print('Output file: ' + args.output)
    print('frame-id:    ' + args.frame_id)
    print('topic:       ' + args.topic)

    n_messages = 0

    with rosbag.Bag(args.output, 'w') as out_bag:
        with open(args.input) as input_file:
            count = len(input_file.readlines())
            input_file.seek(0)
            for i, line in enumerate(input_file):
                # read line:
                row = line.split(',')

                t = float(row[0])           # 0: timestamp
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
                msg = rawseeds_ros.msg.RawseedsIMU()
                msg.header.frame_id = args.frame_id
                msg.header.stamp = rospy.Time.from_sec(t)
                msg.angular_velocity.x = vel[0]
                msg.angular_velocity.y = vel[1]
                msg.angular_velocity.z = vel[2]
                msg.linear_acceleration.x = acc[0]
                msg.linear_acceleration.y = acc[1]
                msg.linear_acceleration.z = acc[2]
                quat = tf.transformations.quaternion_from_matrix(rot_mat)
                msg.orientation = geometry_msgs.msg.Quaternion(quat[0], quat[1], quat[2], quat[3])
                msg.magnetic_field.x = mag[0]
                msg.magnetic_field.y = mag[1]
                msg.magnetic_field.z = mag[2]

                out_bag.write(args.topic, msg, rospy.Time.from_sec(t))

                n_messages += 1

                state = 'Progress: ' + '{0:.2f}'.format((i + 1) / (1.0 * count) * 100) + '%'
                sys.stdout.write('%s\r' % state)

    print("Conversion of %d messages done." % n_messages)

if __name__ == "__main__":
    main()