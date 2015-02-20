#!/usr/bin/python
import argparse
import rospy
import tf
import tf.msg

from nav_msgs.msg import Odometry

class LogTrajectory:

    def __init__(self,args):
        self.args = args
        print ("Log odom to %s started with")%self.args.filename        
        print('Fixed frame:  ' + self.args.fixed_frame)
        print('Moving frame: ' + self.args.moving_frame)

        #init ros
        rospy.init_node('log_odom', anonymous=True) #make node 

        #self.old_time = rospy.Time.now()        
        self.old_time = 0.0
        self.old_x = 0
        self.old_y = 0
        self.old_yaw = 0
        
        rospy.Subscriber("/odom/truth", Odometry, self.odomCallback) 

    
        #Read Parameters
        # topic_odom_1 = rospy.get_param('topic_odom_1', '/robot_0/odom/truth')
        # topic_odom_2 = rospy.get_param('topic_odom_2', '/robot_0/odom')    
        
        self.file = open(self.args.filename, 'w')                    
        self.file_relation = open(self.args.filename+".relations", 'w')                    
                           
        r = rospy.Rate(75)         
        while not rospy.is_shutdown():                       
            r.sleep()
            
        rospy.spin()    


    def odomCallback(self,data):        
        time = data.header.stamp.to_sec()
        

        
        quat = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        
        (r, p, yaw) = tf.transformations.euler_from_quaternion(quat)
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        p_string = "%f, %f, %f, %f\n"%(time, x, y, yaw)

        #time_diff = time.to_sec() - self.old_time.to_sec()
        time_diff = time - self.old_time        
        # filter out the same times to avoid problems with interpolation
        if (time_diff > 0):                
            self.file.write(p_string)
            p_string_relations = "%f, %f, %f, %f, 0, 0, 0, %f\n"%(self.old_time, time, x-self.old_x, y-self.old_y, yaw-self.old_yaw)
            self.file_relation.write(p_string_relations)
        else:
            print "Error time diff = 0 at ", time
        self.old_time = time    
        

    
    

if __name__ == "__main__":
 
    parser = argparse.ArgumentParser(description='logs the trajectory of an object by recording tf poses')
    parser.add_argument('--filename',  type=str, default="logged_trajectory.csv", help='Name of the file to be logged')
    parser.add_argument('--fixed_frame',  type=str, default="/map", help='A fixed frame e.g. /map')
    parser.add_argument('--moving_frame',  type=str, default="/base_link", help='A moving frame e.g. /base_link')
    
    args, unkown = parser.parse_known_args()
     
    
    LogTrajectory(args)
