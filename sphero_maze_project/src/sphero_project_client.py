#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty, EmptyRequest

if __name__ == '__main__':
    rospy.init_node('sphero_move_client')
    rospy.wait_for_service('/sphero_service')
    service = rospy.ServiceProxy('/sphero_service', Empty)
    req = EmptyRequest()
    result = service(req)
    print(result)