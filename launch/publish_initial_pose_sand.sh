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
      x: 43.278
      y: 19.046
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 1.0
      w: 0.0
  covariance: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01]" -1
