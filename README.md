# rawseeds_ros: Tools for using the rawseeds benchmark datasets with ROS

## Basic information:

The rawseeds benchmark datasets consists of odometry, IMU, 4 laser scanners, one monocular camera, one trinocular camera, one omnidirectional camera and ground truth data for both localization and mapping.

http://www.rawseeds.org/home/

## How to use:

Since creating one bagfile containing all sensor data leads to unsuably big bag files, you can 

* convert all the sensor sources you need to one bag file each
* combine bagfiles of sensor sources you want to use to one single bag file.

Example usage:

Convert some sensor streams to bag files:

    rosrun rawseeds_ros odometry2bag.py Bicocca_2009-02-25a-ODOMETRY_XYT.csv odom.bag

    rosrun rawseeds_ros sick2bag.py Bicocca_2009-02-25a-SICK_FRONT.csv sick_front.bag --frame_id sick_front --topic sick_front
    rosrun rawseeds_ros sick2bag.py Bicocca_2009-02-25a-SICK_REAR.csv sick_rear.bag --frame_id sick_rear --topic sick_rear
    
    rosrun rawseeds_ros hokuyo2bag.py Bicocca_2009-02-25a-HOKUYO_FRONT.csv hokuyo_front.bag --frame_id hokuyo_front --topic hokuyo_front
    rosrun rawseeds_ros hokuyo2bag.py Bicocca_2009-02-25a-HOKUYO_REAR.csv hokuyo_rear.bag --frame_id hokuyo_rear --topic hokuyo_rear
    
Merge bag files:
    
    rosrun rawseeds_ros merge_bags.py -i sick_front.bag hokuyo_front.bag sick_rear.bag hokuyo_rear.bag odom.bag -o combined.bag
    
Run bag file:

    rosparam set use_sim_time true
    rosbag play --clock combined.bag
    
Launch tf nodes containing the extrinsic sensor calibration
    
    roslaunch rawseeds_ros transforms.launch
    
Visualization with rviz:

    roslaunch rawseeds_ros rviz.launch


Log a trajectory to a csv file

     rosrun rawseeds_ros log_trajectory.py test.csv /map  /base_link
