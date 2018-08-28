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
    parser = argparse.ArgumentParser(description='convert rawseeds .csv trajectory to relations')
    parser.add_argument('input', help='input trajectory file')
    parser.add_argument('output', help='output relations file')
 
    args = parser.parse_args()

    print('Input file:  ' + args.input)
    print('Output file: ' + args.output)
 
    in_file = open(args.input, 'r')
    out_file = open(arg.output, 'w')

    last_t  = 0.0;
    last_tf = np.identity(3);

    count = 0

    for l in in_file:
        row = line.split(',')

        t  = float(row[0])  # timestamp in SECONDS
        x  = float(row[1])
        y  = float(row[2])
        th = float(row[3])

        tf      = numpy.identiy(3)
        tf[0,0] =  numpy.cos(th)
        tf[0,1] = -numpy.sin(th)
        tf[1,0] =  numpy.sin(th)
        tf[1,1] =  numpy.cos(th)
        tf[0,2] =  x
        tf[1,2] =  y



        if last_t is not 0.0:
            tf_rel = last_tf.inverse() * tf

            # write out
            out = str(last_t) + "," + 
                  str(t) + "," + 
                  str(tf_rel[0,2]) + "," +
                  str(tf_rel[1,2]) + "," +
                  str(0.0) + ","
                  str(0.0) + ","
                  str(0.0) + ","
                  str(numpy.acos(tf_rel[0,0]))
            out_file.write(out)
            count += 1

        last_tf = tf
        last_t = t


    print("Calculated " + str(count) + " relative transformations.")
    return


if __name__ == "__main__":
    main()
