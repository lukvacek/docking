"""
testing offboard positon control with a simple takeoff script
"""

import rospy
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
import math
import numpy
from geometry_msgs.msg import TwistStamped
from VelocityController import VelocityController
from LandingController import LandingController


class QuadController:
    sim_ctr = 1

    des_pose = PoseStamped()
    cur_pose = PoseStamped()

    isReadyToFly = True

    target1 = PoseStamped()
    target1.pose.position.x = 0
    target1.pose.position.y = 0
    target1.pose.position.z = 5

    target2 = PoseStamped()
    target2.pose.position.x = 1
    target2.pose.position.y = 3
    target2.pose.position.z = 4.7

    def __init__(self):
        rospy.init_node('f450_swarm_controller', anonymous=True)
        rospy.set_param("/mavros1/conn/heartbeat_rate", '3.0');
        rospy.set_param("/mavros2/conn/heartbeat_rate", '3.0');
        pose_pub1 = rospy.Publisher('/mavros1/setpoint_position/local', PoseStamped, queue_size=10)
	pose_pub2 = rospy.Publisher('/mavros2/setpoint_position/local', PoseStamped, queue_size=10)

        arming_srv1 = rospy.ServiceProxy("/mavros1/cmd/arming", CommandBool)
        mode_srv1 = rospy.ServiceProxy("/mavros1/set_mode", SetMode)
	
	arming_srv2 = rospy.ServiceProxy("/mavros2/cmd/arming", CommandBool)
        mode_srv2 = rospy.ServiceProxy("/mavros2/set_mode", SetMode)

        rate = rospy.Rate(10)  # Hz
        
        # self.des_vel.twist.linear.x = 0
        # self.des_vel.twist.linear.y = 0
        for i in range(0,10):
            pose_pub1.publish(self.target1);
            pose_pub2.publish(self.target2);
            rate.sleep()

        print "Setting Offboard Mode"
        result = mode_srv1(custom_mode="OFFBOARD")
        print result
        result = mode_srv2(custom_mode="OFFBOARD")
        print result
        print "Arming"
        result = arming_srv1(value=True)
        print result
        result = arming_srv2(value=True)
        print result


        while not rospy.is_shutdown():
            pose_pub1.publish(self.target1);
            pose_pub2.publish(self.target2);
            rate.sleep()

if __name__ == "__main__":
    QuadController()
