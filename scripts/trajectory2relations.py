#!/usr/bin/python
import argparse
import numpy as np
import sys 

def main():
    parser = argparse.ArgumentParser(description='convert rawseeds .csv trajectory to relations')
    parser.add_argument('--input', help='input trajectory file')
    parser.add_argument('--output', help='output relations file')
 
    args = parser.parse_args()

    print('Input file:  ' + args.input)
    print('Output file: ' + args.output)
 
    in_file = open(args.input, 'r')
    out_file = open(args.output, 'w')

    last_t  = 0.0;
    last_tf = np.identity(3);

    count = 0

    for l in in_file:
        row = l.split(',')

        t  = float(row[0])  # timestamp in SECONDS
        x  = float(row[1])
        y  = float(row[2])
        th = float(row[3])

        tf      =  np.identity(3)
        tf[0,0] =  np.cos(th)
        tf[0,1] = -np.sin(th)
        tf[1,0] =  np.sin(th)
        tf[1,1] =  np.cos(th)
        tf[0,2] =  x
        tf[1,2] =  y



        if last_t is not 0.0:
            tf_rel = np.dot(np.linalg.inv(last_tf),tf)

            # write out
            out = str(last_t) + "," + \
                  str(t) + "," +  \
                  str(tf_rel[0,2]) + "," + \
                  str(tf_rel[1,2]) + "," + \
                  str(0.0) + "," + \
                  str(0.0) + "," + \
                  str(0.0) + "," + \
                  str(np.arccos(tf_rel[0,0])) + "\n"
            out_file.write(out)
            count += 1

        last_tf = tf
        last_t = t



    print("Calculated " + str(count) + " relative transformations.")
    return


if __name__ == "__main__":
    main()
