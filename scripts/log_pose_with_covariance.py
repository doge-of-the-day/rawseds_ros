#!/usr/bin/python
import argparse
import rospy
import tf
import tf.msg
import numpy as np

from geometry_msgs.msg import PoseWithCovariance

class LogTrajectory:

    def __init__(self,args):
        self.args = args
        print ("Log odom to %s started with")%self.args.filename        

        #init ros
        rospy.init_node('log_pose_with_covariance', anonymous=True) #make node 

        #self.old_time = rospy.Time.now()        
        self.old_time = 0.0
        self.old_x = 0
        self.old_y = 0
        self.old_yaw = 0
        
        rospy.Subscriber("/pose", PoseWithCovariance, self.poseWitchCovarianceCallback) 

    
        #Read Parameters
        # topic_odom_1 = rospy.get_param('topic_odom_1', '/robot_0/odom/truth')
        # topic_odom_2 = rospy.get_param('topic_odom_2', '/robot_0/odom')    
        
        self.file = open(self.args.filename, 'w')                    
        self.file_relation = open(self.args.filename+".relations", 'w')                    
                           
        r = rospy.Rate(75)         
        while not rospy.is_shutdown():                       
            r.sleep()
            
        rospy.spin()    


# Taken from the matlab function of the benchmar toolbox
#s = sin(t1(3,:));
#c = cos(t1(3,:));
#dx = t2(1,:)-t1(1,:);
#dy = t2(2,:)-t1(2,:);
#tac =  [ c .* dx + s .* dy
#        -s .* dx + c .* dy
#        normalize_ang(t2(3,:)-t1(3,:)) ];


    def poseWitchCovarianceCallback(self,data):        
        time = rospy.Time.now().to_sec()   
           
        quat = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
        
        (r, p, yaw) = tf.transformations.euler_from_quaternion(quat)
        yaw = yaw
        x = data.pose.position.x
        y = data.pose.position.y
        p_string = "%f, %f, %f, %f\n"%(time, x, y, yaw)

  
        c = np.cos(self.old_yaw)
        s = np.sin(self.old_yaw)
#            dx = -c * self.old_x -s * self.old_y + x
#            dy =  s * self.old_x -c * self.old_y + y
#            dw = (-self.old_yaw + yaw) % (2*np.pi)
        Dx = x - self.old_x
        Dy = y - self.old_y
        
        dx =  c * Dx + s * Dy
        dy = -s * Dx + c * Dy
        dw = (yaw-self.old_yaw) % (2*np.pi)
        
        self.file.write(p_string)
        p_string_relations = "%f, %f, %f, %f, 0, 0, 0, %f\n"%(self.old_time, time, dx, dy, dw)
        self.file_relation.write(p_string_relations)
        
        self.old_time = time  
        self.old_x = x
        self.old_y = y
        self.old_yaw = yaw
#        else:
#            print "Error time diff = 0 at ", time

        

    
    

if __name__ == "__main__":
 
    parser = argparse.ArgumentParser(description='logs the trajectory of an object by recording tf poses')
    parser.add_argument('--filename',  type=str, default="logged_trajectory.csv", help='Name of the file to be logged')
    
    args, unkown = parser.parse_known_args()
     
    
    LogTrajectory(args)
