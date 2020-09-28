#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty, EmptyResponse
import numpy as np 
import math 


class VelCmds(object):
    def __init__(self):
        self._cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        self._vel = Twist()
        self.lspeed = .45
        self.aspeed = .78

    def move_sphero(self, direction):
        if direction == "Right_Turn":
            self._vel.linear.x = -self.lspeed
            self._vel.angular.z = self.aspeed
        elif direction == "Left_Turn":
            self._vel.linear.x = -self.lspeed
            self._vel.angular.z = -self.aspeed
        elif direction == "rot_z_cw":
            self._vel.linear.x = 0
            self._vel.angular.z = self.aspeed
        elif direction == "rot_z_ccw":
            self._vel.linear.x = 0
            self._vel.angular.z = -self.aspeed
        elif direction == "exit_maze":
            self._vel.linear.x = -self.lspeed
            self._vel.angular.z = 0 
        elif direction == "stop":
            self._vel.linear.x = 0
            self._vel.angular.z = 0
      
        print('Action: {}'.format(direction))
        self._cmd_vel_pub.publish(self._vel)
    
class SubWrapper(object):
    def __init__(self, topic, msgType):
        self._topic_name = topic
        self.sub = rospy.Subscriber(topic, msgType, self.callback)
        self.data = msgType()
    def callback(self, msg):
        self.data = msg
    def get_data(self):
        return self.data
        
    
def shutdownhook():
    global shutdown
    shutdown = True
    velCmds.move_sphero("stop")


def srv_callback(request):
    
    global odom_wrapper, imu_wrapper
    global velCmds, direction, rate, shutdown
    
    
    pos = odom_wrapper.get_data()
    pos = pos.pose.pose.position
    ori = imu_wrapper.get_data()
    ori = ori.orientation

    Waypoint1 = -0.9
    Waypoint2 = -1.5
    Waypoint3 = -2.1
    
    while ori.z < .0105-.01 and not shutdown:
        direction = "rot_z_ccw"
        velCmds.move_sphero(direction)
        rate.sleep()
        ori = imu_wrapper.get_data()
        ori = ori.orientation
        print(np.degrees(ori.z))

    while pos.y > Waypoint1 and not shutdown:
        direction = "Left_Turn"
        velCmds.move_sphero(direction)
        rate.sleep()
        pos = odom_wrapper.get_data()
        pos = pos.pose.pose.position
        ori = imu_wrapper.get_data()
        ori = ori.orientation
        print(pos)

    while pos.y > Waypoint2 and not shutdown:
        direction = "Right_Turn"
        velCmds.move_sphero(direction)
        rate.sleep()
        pos = odom_wrapper.get_data()
        pos = pos.pose.pose.position
        ori = imu_wrapper.get_data()
        ori = ori.orientation
        print(pos)    

    while pos.y > Waypoint3 and not shutdown:
        direction = "Left_Turn"
        velCmds.move_sphero(direction)
        rate.sleep()
        pos = odom_wrapper.get_data()
        pos = pos.pose.pose.position
        ori = imu_wrapper.get_data()
        ori = ori.orientation
        print(pos)    

    direction = "stop"
    velCmds.move_sphero(direction)
    print('Maze exited')
    return EmptyResponse()
    
if __name__ == "__main__":
    
    rospy.init_node('sphero_move_server')
    velCmds = VelCmds()
    rate = rospy.Rate(6)

    imu_wrapper = SubWrapper("/sphero/imu/data3", Imu) 
    odom_wrapper = SubWrapper("/odom", Odometry)
   
    
    shutdown = False
    rospy.on_shutdown(shutdownhook)
    
    sphero_service = rospy.Service('sphero_service', Empty, srv_callback)
    rospy.spin()