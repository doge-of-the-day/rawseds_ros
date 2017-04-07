#!/usr/bin/python
import argparse
import numpy
import cv2
import yaml

class Calibration:


    def __init__(self):
        things = ""

    def load(self, path):
        f = open(path,'r')
        data_map = yaml.safe_load(f)
        f.close()

        return

def main():
    parser = argparse.ArgumentParser(description='convert two rawseeds svs streams to ROS bag file')
    parser.add_argument('folder_a', help='first camera stream')
    parser.add_argument('folder_b', help='second camera stream')
    parser.add_argument('calibration_a', help='calibration file for the first camera')
    parser.add_argument('calibration_b', help='calibration file for the second camera')
    parser.add_argument('extrinsic')

    # http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/example5.html
    # http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html