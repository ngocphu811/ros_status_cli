#!/usr/bin/env python

from enum import Enum
from topic_stats import TopicStats
from log_stats import LogStats, LogStat

class NodeStatus(Enum):
    UNKNOWN = 0
    OK = 1
    DEAD = 2

class NodeStats(object):
    def __init__(self, name):
        self.name = name
        self.status = NodeStatus.UNKNOWN
        self.logstats = LogStats()

    def __eq__(self, obj):
        return isinstance(obj, NodeStats) and self.name == obj.name

    def __hash__(self):
        return hash(self.name)
    
    def __str__(self):
        return '{} [{}]'.format(self.name, self.status)

    def __repr__(self):
        return str(self)
