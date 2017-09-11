#!/bin/bash
bag=${PWD##*/}-groundtruth.bag
if [ ! -f odom_gt.bag ] || [ "$1" = "--clean" ]; then
    echo "Converting ground truth odometry!"
    rosrun rawseeds_ros gtextodom2bag.py --frame_id odom --topic /odom *GROUNDTRUTH.csv odom_gt.bag
fi
if [ ! -f imu.bag ] || [ "$1" = "--clean" ] ; then
    echo "Converting IMU!"
    rosrun rawseeds_ros imu2bag.py --frame_id imu --topic /rawseeds_imu *IMU_STRETCHED.csv imu.bag
fi
if [ ! -f sick_rear.bag ] || [ "$1" = "--clean" ] ; then
    echo "Converting SICK laserscanner rear!"
    rosrun rawseeds_ros sick2bag.py --frame_id sick_rear --topic /sick/rear *SICK_REAR.csv sick_rear.bag
fi
if [ ! -f sick_front.bag ] || [ "$1" = "--clean" ] ; then
    echo "Converting SICK laserscanner front!"
    rosrun rawseeds_ros sick2bag.py --frame_id sick_front --topic /sick/front *SICK_FRONT.csv sick_front.bag
fi
if [ ! -f hokuyo_rear.bag ] || [ "$1" = "--clean" ] ; then
    echo "Converting Hokuyo laserscanner rear!"
    rosrun rawseeds_ros hokuyo2bag.py --frame_id hokuyo_rear --topic /hokuyo/rear *HOKUYO_REAR.csv hokuyo_rear.bag
fi
if [ ! -f hokuyo_front.bag ] || [ "$1" = "--clean" ] ; then
    echo "Converting Hokuyo laserscanner rear!"
    rosrun rawseeds_ros hokuyo2bag.py --frame_id hokuyo_front --topic /hokuyo/front *HOKUYO_FRONT.csv hokuyo_front.bag
fi
rosrun rawseeds_ros merge_bags.py -i odom_gt.bag imu.bag sick_rear.bag sick_front.bag hokuyo_front.bag hokuyo_rear.bag -o $bag


