#!/usr/bin/env python

from __future__ import print_function

import rospy
import time, threading
from std_msgs.msg import String
from ros_status_cli.srv import *

NODE_NAME="dummy_node"

class DummyNode(object):

    def periodic_publish(self):
        self.pub.publish("test")
        threading.Timer(0.01, self.periodic_publish).start()

    def do_nothing(self):
        pass

    def start_dummy_service(self):
        rospy.Service(NODE_NAME + '/dummy_service', DummyNodeAction, self.do_nothing)
        
    def handle_dummy_node_action(self, req):
        action = req.action
        result = True
        if action == "log debug":
            rospy.logdebug("log debug")
        elif action == "log info":
            rospy.loginfo("log info")
        elif action == "log warn":
            rospy.logwarn("log warn")
        elif action == "log err":
            rospy.logerr("log err")
        elif action == "log fatal":
            rospy.logfatal("log fatal")
        elif action == "init publisher":
            self.pub = rospy.Publisher('test_topic', String, queue_size=10)
            self.periodic_publish()           
        elif action == "init service":
            self.start_dummy_service()
        elif action == "shutdown":
            self.shutdown = True
        else: 
            result = False
            print("unknown action")

        print("handle node action called")
        return DummyNodeActionResponse(result)

    def start(self):
        rospy.init_node(NODE_NAME, log_level=rospy.DEBUG, anonymous=False)
        s = rospy.Service('test_service', DummyNodeAction, self.handle_dummy_node_action)
        r = rospy.Rate(10)
        self.shutdown = False
        while not self.shutdown:
            r.sleep()
        print("shutting down")
        rospy.signal_shutdown("requested")      

if __name__ == '__main__':
    node = DummyNode()
    node.start()
