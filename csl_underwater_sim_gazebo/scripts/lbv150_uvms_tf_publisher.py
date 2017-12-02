#!/usr/bin/env python

# ROS imports
import roslib
import rospy

# Msgs imports
from gazebo_msgs.srv import GetLinkState
from geometry_msgs.msg import *
import tf
import tf2_ros
                        

def Get_LinkState():
    	rospy.wait_for_service('/gazebo/get_link_state')
        try:
            get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
            
            resp1 = get_link_state('base_link','world')
            
            br = tf2_ros.TransformBroadcaster()
             
            tr = geometry_msgs.msg.TransformStamped()
            tr.header.stamp = rospy.Time.now()
    	    tr.header.frame_id = "world"
            tr.child_frame_id = "base_link"
            tr.transform.translation.x = resp1.link_state.pose.position.x
            tr.transform.translation.y = resp1.link_state.pose.position.y
            tr.transform.translation.z = resp1.link_state.pose.position.z
    
            tr.transform.rotation.x = resp1.link_state.pose.orientation.x
            tr.transform.rotation.y = resp1.link_state.pose.orientation.y
            tr.transform.rotation.z = resp1.link_state.pose.orientation.z
            tr.transform.rotation.w = resp1.link_state.pose.orientation.w
            
            br.sendTransform(tr)
                     
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e           
 
if __name__ == '__main__':
    try:
        rospy.init_node('lbv150_uvms_world_publish')
        rate_it = rospy.Rate(100)        
        
        while not rospy.is_shutdown():
            Get_LinkState()
            rate_it.sleep()
        rospy.spin()

    except rospy.ROSInterruptException: pass
