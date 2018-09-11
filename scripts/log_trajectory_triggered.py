#!/usr/bin/python
import argparse
import rospy
import tf
import tf.msg

class LogTrajectory:

    def __init__(self,args):
        self.args = args
        print ("Log trajectory to %s started with")%self.args.filename        
        print('Fixed frame:  ' + self.args.fixed_frame)
        print('Moving frame: ' + self.args.moving_frame)

        #init ros
        rospy.init_node('log_trajectory_triggered', anonymous=True) #make node 
        rospy.Subscribe('tf', tf.msg.tfMessage, self.callback)
        self.tf_listener = tf.TransformListener()
        self.old_time = 0

        #Read Parameters
        # topic_odom_1 = rospy.get_param('topic_odom_1', '/robot_0/odom/truth')
        # topic_odom_2 = rospy.get_param('topic_odom_2', '/robot_0/odom')    
        
        self.file = open(self.args.filename, 'w')                    
        rospy.spin()    

    def __del__(self):
        file.close()

    def callback(self, data):
        updateTf()

    def updateTf(self):
        
        time_now = rospy.Time.now()        
        try:
            now = rospy.Time.now()
            self.tf_listener.waitForTransform(self.args.fixed_frame, self.args.moving_frame, now, rospy.Duration(0.1))
            (trans,rot) = self.tf_listener.lookupTransform(self.args.fixed_frame, self.args.moving_frame, now)
            (r, p, yaw) = tf.transformations.euler_from_quaternion(rot)
            p_string = "%f, %f, %f, %f\n"%(now.to_sec(),trans[0], trans[1], yaw)
            time_diff = now.to_sec() - self.old_time 
            self.old_time = now.to_sec()
            if (time_diff > 0):                
                self.file.write(p_string)
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception):
            print "Error: Could not get transform from %s to %s at time %f"%(self.args.moving_frame, self.args.fixed_frame, time_now.to_sec())
            return
    

if __name__ == "__main__":
 
    parser = argparse.ArgumentParser(description='logs the trajectory of an object by recording tf poses')
    parser.add_argument('--filename',  type=str, default="logged_trajectory.csv", help='Name of the file to be logged')
    parser.add_argument('--fixed_frame',  type=str, default="/map", help='A fixed frame e.g. /map')
    parser.add_argument('--moving_frame',  type=str, default="/base_link", help='A moving frame e.g. /base_link')
    
    args, unkown = parser.parse_known_args()
     
    
    LogTrajectory(args)
