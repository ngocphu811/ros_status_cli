# coding: utf-8

from ros_status import RosStatus
a = RosStatus()
nodes = a.get_nodes()
print(nodes)
