#!/bin/sh

rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "header: 
  seq: 14
  stamp: 
    secs: 0
    nsecs: 0
  frame_id: map
pose: 
  pose: 
    position: 
      x: 46.578250885
      y: 2.42099475861
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.679567344219
      w: 0.733613130111
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]"
