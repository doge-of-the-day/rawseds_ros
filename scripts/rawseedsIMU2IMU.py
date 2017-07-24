#!/usr/bin/python
import argparse
import numpy
import rosbag
import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import rawseeds_ros.msg
import tf
import sys

pub_imu = None
pub_mag = None

def convert(data):
    msg_imu = sensor_msgs.msg.Imu()
    msg_mag = sensor_msgs.msg.MagneticField()
    msg_imu.header = data.header
    msg_mag.header = data.header
    msg_imu.orientation = data.orientation
    msg_imu.angular_velocity = data.angular_velocity
    msg_imu.linear_acceleration = data.linear_acceleration
    msg_mag.magnetic_field = data.magnetic_field

    pub_imu.publish(msg_imu)
    pub_mag.publish(msg_mag)

def main():
    parser = argparse.ArgumentParser(description='online conversion from rawseeds imu messages to ros imu messages')
    parser.add_argument('--input_topic', type=str, default="imu_rawseeds", help='input rawseeds imu topic')
    parser.add_argument('--output_topic_imu', type=str, default="imu", help='output imu topic')
    parser.add_argument('--output_topic_mag', type=str, default="mag", help='output magneto meter topic')

    args = parser.parse_args()

    print('input_topic:    ' + args.input_topic)
    print('output_topic_imu:   ' + args.output_topic_imu)
    print('output_topic_mag:   ' + args.output_topic_mag)

    rospy.init_node('rawseedsIMU2IMU')
    rospy.Subscriber(args.input_topic, rawseeds_ros.msg.RawseedsIMU, convert)

    global pub_imu
    global pub_mag
    pub_imu = rospy.Publisher(args.output_topic_imu, sensor_msgs.msg.Imu, queue_size='1')
    pub_mag = rospy.Publisher(args.output_topic_mag, sensor_msgs.msg.MagneticField, queue_size='1')

    rospy.spin()


if __name__ == "__main__":
    main()