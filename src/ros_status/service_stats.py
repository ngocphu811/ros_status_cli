#!/usr/bin/env python

class ServiceStats(object):
    def __init__(self, name, nodename=""):
        self.name = name
        self.nodename = nodename

    def __str__(self):
        return "{} [{}]".format(self.name, self.nodename)

    def __repr__(self):
        return self.__str__()
