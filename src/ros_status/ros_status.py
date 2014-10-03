#!/usr/bin/env python

import os
import xmlrpclib
import rospy
import sys
import urwid
from validate import Validator
import subprocess
import rospy
from rosgraph_msgs.msg import Log

from node_stats import NodeStats, NodeStatus
from topic_stats import TopicStats
from service_stats import ServiceStats
from log_stats import LogType

CALLER_ID = '/' + sys.argv[0]

class NoRosConnectionException(Exception):
    pass

class RosStatus(object): 
    def __init__(self):
        self.master = rospy.get_master()
        rospy.init_node('ros_status', anonymous=True, log_level=rospy.FATAL)

        self.DEVNULL = open(os.devnull,"w")
        self._nodes = {}

        self.connect_xmlproxy()
        self.subscribe_to_log_aggr()
        self.rosnode_exec = "rosnode"

    def __del__(self):
        self.DEVNULL.close()

    def connect_xmlproxy(self):
        self.proxy = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI']) 
        try:
            code, msg, val = self.proxy.getSystemState(CALLER_ID)
        except Exception as e:
            print("Unable to connect to ROS master:\n {}".format(str(e)))
            sys.exit(-1)
        if code == 1:
            pubs, subs, srvs = val
        else:
            raise Exception("proxy start failed: {} {}".format(code, msg))

    def subscribe_to_log_aggr(self):
        self.logaggr_sub = rospy.Subscriber("rosout_agg", Log, self.rosout_agg_callback)

    def rosout_agg_callback(self, msg):
        log_type = LogType(msg.level)
        node = self.get_node(msg.name[1:]) # remove first '/'
        self.add_log_to_node(node, log_type)

    def add_log_to_node(self, node, log_type):
        node.logstats[log_type].inc_count()

    def get_nodes(self):
        current_nodes = subprocess.check_output([self.rosnode_exec, 'list'], stderr=self.DEVNULL).split('\n')[:-1]
        # alive nodes will be marked as alive
        self._mark_nodes_as_dead()
        for node_name in current_nodes:
            self.get_node(node_name).status = NodeStatus.OK
        return list(self._nodes.itervalues())

    def get_node(self, name):
        node = self._nodes.get(name)
        if node == None:
            node = NodeStats(name)
            self._nodes[name] = node
        return node

    def _mark_nodes_as_dead(self):
        """Mark all the known node as dead"""
        for node in self._nodes.itervalues():
            node.status = NodeStatus.DEAD

    def get_published_topics(self):
        try:
            code, status_message, topics_and_types = self.proxy.getPublishedTopics(CALLER_ID, "")
        except:
            raise NoRosConnectionException()
        topics = map(lambda topic: TopicStats(topic[0], topic[1]), topics_and_types)
        return topics

    def get_services(self):
        return map(lambda s: ServiceStats(s[0], s[1][0]), self.get_system_state()[2])

    def get_publishers(self):
        return self.get_system_state()[0]
    
    def get_subscribers(self):
        return self.get_system_state()[1]

    def get_system_state(self):
        return self.proxy.getSystemState(CALLER_ID)[2]

    def getUri(self):
        _, _, uri = self.proxy.getUri(CALLER_ID)
        return uri

    def reset_nodes(self):
        self._nodes = {}

    def reset(self):
        self.reset_nodes()

if __name__ == "__main__":
    print("START MAIN")
    a = RosStatus()
    print("\nnodes:\n")
    b = [str(b) for b in a.get_nodes()]
    print (b)
