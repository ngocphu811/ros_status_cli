#!/usr/bin/env python

class TopicStats(object):
    def __init__(self, name, topic_type, subscribers=[], publishers=[]):
        self.name = name
        self.topic_type = topic_type
        self.subscribers = subscribers
        self.publishers = publishers

    def __str__(self):
        return "{} [{}]".format(self.name, self.topic_type)

    def __repr__(self):
        return self.__str__()

