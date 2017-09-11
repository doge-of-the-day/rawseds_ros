#!/bin/bash
bag=${PWD##*/}.bag
if [ ! -f odom.bag ] || [ "$1" = "--clean" ]; then
    echo "Converting Odometry!"
    rosrun rawseeds_ros odometry2bag.py *ODOMETRY_XYT.csv odom.bag
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
rosrun rawseeds_ros merge_bags.py -i odom_gt.bag imu.bag sick_rear.bag sick_front.bag hokuyo_front.bag hokuyo_back.bag -o $bag



