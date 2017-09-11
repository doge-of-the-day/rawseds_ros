#!/bin/bash
rosrun rawseeds_ros gtextodom2bag.py --frame_id odom --topic /odom *GROUNDTRUTH.csv odom_gt.bag
rosrun rawseeds_ros imu2bag.py --frame_id imu --topic /rawseeds_imu *IMU_STRETCHED.csv imu.bag
rosrun rawseeds_ros sick2bag.py --frame_id sick_rear --topic /sick/rear *SICK_REAR.csv sick_rear.bag
rosrun rawseeds_ros sick2bag.py --frame_id sick_front --topic /sick/front *SICK_REAR.csv sick_front.bag
rosrun rawseeds_ros hokuyo2bag.py --frame_id hokyuo_rear --topic /hokuyo/rear *HOKUYO_REAR.csv hokuyo_rear.bag
rosrun rawseeds_ros hokuyo2bag.py --frame_id hokyuo_front --topic /hokuyo/front *HOKUYO_FRONT.csv hokuyo_front.bag

