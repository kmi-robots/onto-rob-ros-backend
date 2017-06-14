import rospy
import importlib
import functools
import json
from io import StringIO


class DynamicNode:

    def __init__(self, name):
        rospy.init_node(name, disable_signals=True)
        self.command = dict()

    @staticmethod
    def rsetattr(obj, attr, val):
        pre, _, post = attr.rpartition('.')
        try:
            v = float(val)
        except ValueError:
            v = val
        return setattr(functools.reduce(getattr, [obj] + pre.split('.')) if pre else obj, post, v)

    def gen_message(self, attr):
        fields = self.command['fields']
        tmsg = attr()
        for f in fields:
            DynamicNode.rsetattr(tmsg, f, self.command['param_values'][f])
        return tmsg

    def activate_publisher(self):
        attr = getattr(importlib.import_module(self.command['pkg'] + '.msg'), self.command['name'])
        pub = rospy.Publisher(self.command['topic'], attr, queue_size=1)
        return pub, attr

    def send_command(self, input_d):
        self.command = input_d
        pub, attr = self.activate_publisher()
        msg = self.gen_message(attr)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if pub.get_num_connections() > 0:
                pub.publish(msg)
                break
            rate.sleep()
        pub.unregister()
