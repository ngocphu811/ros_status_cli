#!/usr/bin/env python

from __future__ import print_function

import unittest
import rostest
import rospy
import time

from ros_status.ros_status import RosStatus
from ros_status.node_stats import *
from ros_status_cli.srv import *
from std_msgs.msg import String

TEST_NAME = 'ros_status'
TEST_SERVICE_NAME = 'test_service'

class Test(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(Test, self).__init__(*args, **kwargs)
        self.test_service = rospy.ServiceProxy(TEST_SERVICE_NAME, DummyNodeAction)
        self.rosstatus = RosStatus()

    def test_node00_does_not_exist(self):
        nodes = self.rosstatus.get_nodes()
        matched = filter(lambda n: n.name == '/dummy_node_non_existent', nodes) 
        self.assertEqual(len(matched), 0, "non existent node is not found")

    def test_node01_exist(self):
        nodes = self.rosstatus.get_nodes()
        matched = filter(lambda n: n.name == '/dummy_node', nodes) 
        self.assertEqual(len(matched), 1, "dummy node is found")
        dummy_node = matched[0]
        self.assertEqual(dummy_node.status, NodeStatus.OK, "dummy node is " + str(NodeStatus.OK) )

    def test_node02_published_topic(self):
        topic_name = "/test_topic"

        topics = self.rosstatus.get_published_topics()
        matched = filter(lambda t: t.name == topic_name, topics) 
        self.assertEqual(len(matched), 0, "test topic is not found when not initialized")

        self.test_service.call("init publisher")
        rospy.wait_for_message(topic_name[1:], String, 0.5)
        
        topics = self.rosstatus.get_published_topics()
        matched = filter(lambda t: t.name == topic_name, topics) 
        self.assertEqual(len(matched), 1, "test topic is found")

    def test_node03_service(self):
        service_name = "/dummy_node/dummy_service"

        services = self.rosstatus.get_services()
        matched = filter(lambda t: t.name == service_name, services) 
        self.assertEqual(len(matched), 0, "service is not found when not initialized")

        self.test_service.call("init service")
        rospy.wait_for_service(service_name, 0.5)
        
        services = self.rosstatus.get_services()
        matched = filter(lambda t: t.name == service_name, services) 
        self.assertEqual(len(matched), 1, "service is found when initialized")

    def test_node04_dead_node(self):
        """This test actually shutdowns the dummy node, so run it last. 
        unittest follows alphabetical order """
        self.test_service.call("shutdown")
        time.sleep(0.2)
        self.rosstatus.reset_nodes()

        nodes = self.rosstatus.get_nodes()
        matched = filter(lambda n: n.name == '/dummy_node', nodes) 
        self.assertEqual(len(matched), 0, "dummy node is not found")
        

if __name__ == '__main__':
    #rospy.init_node(TEST_NAME, anonymous=True)
    rostest.unitrun('ros_status_test', TEST_NAME, Test)

