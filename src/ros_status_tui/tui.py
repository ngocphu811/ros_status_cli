#!/usr/bin/env python2

import os
import xmlrpclib
import rospy
import sys
import urwid
import itertools
import random
import logging
import math
import subprocess
from validate import Validator
from enum import Enum
from ros_status.ros_status import RosStatus, NoRosConnectionException
from ros_status.topic_stats import TopicStats
from ros_status.service_stats import ServiceStats

logger = logging.getLogger('RosStatus')
hdlr = logging.FileHandler('RosStatus.log')
formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
hdlr.setFormatter(formatter)
logger.addHandler(hdlr) 
logger.setLevel(logging.DEBUG)


class Tui:
    def __init__(self):
        try:
            self.ros_status = RosStatus()
        except Exception as e: 
            print("unable to connect to ros master:\n" + str(e))
            exit(-1)
        self.update_time = 0.3

    def handle_unhandled_input(self, key):
        pass

    def handle_alarm(self):
        self.update_view()
        self.set_alarm()

    def show_error_msg(self):
        self.titleText.set_text("unable to connect to ros master")

    def update_view(self):
        ros_nodes = None
        ros_nodes = self.ros_status.get_nodes()
        node_widgets = map(RosNodeWidget, ros_nodes)
        self.node_pile.contents = zip(node_widgets, itertools.repeat(('pack', None)))

        ros_topics = []
        try:
            ros_topics = self.ros_status.get_published_topics()
        except NoRosConnectionException as e:
            self.show_error_msg()
            return
        topic_widgets = map(RosTopicWidget, ros_topics)
        self.topic_pile.contents = zip(topic_widgets, itertools.repeat(('pack', None)))

    def start(self):
        self.titleText = urwid.Text(self.ros_status.getUri().decode('unicode-escape'))
        self.node_pile = urwid.Pile([urwid.Text("loading...")])
        self.topic_pile = urwid.Pile([urwid.Text("loading...")])
        self.node_pile_area = urwid.Pile([urwid.Text("nodes"), self.node_pile])
        self.topic_pile_area = urwid.Pile([urwid.Text("topics"), self.topic_pile])
        self.node_pile_area = urwid.LineBox(self.node_pile_area)
        self.topic_pile_area = urwid.LineBox(self.topic_pile_area)
        self.content_columns = urwid.Columns([('weight', 3, self.topic_pile_area),
                                              ('weight', 1, self.node_pile_area)])
        self.main_pile = urwid.Pile([self.titleText, urwid.Divider(), self.content_columns]) 
        self.top = urwid.Filler(self.main_pile, valign='top')
        self.loop = urwid.MainLoop(self.top, 
                unhandled_input=lambda k: self.handle_unhandled_input(k))
        self.set_alarm()
        self.loop.run()

    def set_alarm(self):
        self.loop.set_alarm_in(self.update_time, lambda _, data: self.handle_alarm())


class RosNodeWidget(urwid.WidgetWrap):

    def __init__(self, ros_node):
        self.options = []
        name = urwid.Text(ros_node.name.decode('unicode-escape'), wrap='clip')
        status = urwid.Text(str(ros_node.status.name).decode('unicode-escape'), wrap='clip')
        info = urwid.Columns([status])
        self.display_widget = urwid.Columns([('weight', 3, name), 
            ('pack', VerticalDivider()), ('weight', 1, info)])
        urwid.WidgetWrap.__init__(self, self.display_widget)

    def rows(self, size, focus=False):
        return self.display_widget.pack(size)[1]

class RosTopicWidget(urwid.WidgetWrap):

    def __init__(self, ros_topic):
        self.options = []
        name = urwid.Text(ros_topic.name.decode('unicode-escape'), wrap='clip')
        topic_type = urwid.Text(str(ros_topic.topic_type).decode('unicode-escape'), wrap='clip')
        #topic_publishers = urwid.Text(str(ros_topic.topic_publishers).decode('unicode-escape'), wrap='clip')
        info = urwid.Columns([topic_type])
        self.display_widget = urwid.Columns([('weight', 1, name), 
            ('pack', VerticalDivider()), ('weight', 2, info)])
        urwid.WidgetWrap.__init__(self, self.display_widget)

    def rows(self, size, focus=False):
        return self.display_widget.pack(size)[1]

class VerticalDivider(urwid.WidgetWrap):
    def __init__(self):
        self.options = []
        self.display_widget = urwid.Text(' | ')
        urwid.WidgetWrap.__init__(self, self.display_widget)

    def pack(self, size, focus=False):
        return self.display_widget.pack(size, focus)
