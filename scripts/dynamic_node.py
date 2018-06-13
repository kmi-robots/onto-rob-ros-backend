import rospy
import importlib
import functools
from rospy_message_converter import message_converter


class DynamicNode:

    def __init__(self, name):
        rospy.init_node(name, disable_signals=True)
        self.subscribers = dict()
        self.publishers = dict()
        self.last_msg = dict()
        self.rate = rospy.Rate(10)
        print('initialization complete')

    @staticmethod
    def rgetattr(obj, attr):
        return functools.reduce(getattr, [obj] + attr.split('.'))

    @staticmethod
    def rsetattr(obj, attr, val):
        pre, _, post = attr.rpartition('.')
        try:
            v = float(val)
        except ValueError:
            v = val
        return setattr(functools.reduce(getattr, [obj] + pre.split('.')) if pre else obj, post, v)

    @staticmethod
    def flat_to_nested_dict(flat_dict):
        nested_dict = dict()
        for k in flat_dict.keys():
            DynamicNode.filldictionary(nested_dict, k, flat_dict[k])
        return nested_dict

    @staticmethod
    def filldictionary(dictionary, key, value):
        pre, _, post = key.partition('.')
        if post:
            if pre in dictionary:
                dictionary[pre].update(DynamicNode.filldictionary(dictionary[pre], post, value))
            else:
                dictionary[pre] = DynamicNode.filldictionary({}, post, value)
            return dictionary
        else:
            try:
                v = float(value)
            except ValueError:
                v = str(value)
            return {pre: v}

    # def gen_message(self, attr):
    #     fields = self.command['fields']
    #     tmsg = attr()
    #     for f in fields:
    #         DynamicNode.rsetattr(tmsg, f, self.command[f])
    #     return tmsg

    @staticmethod
    def activate_publisher(topic, pkg, name, latched):
        # create a message of the 'pkg'/'name' type
        attr = getattr(importlib.import_module(pkg + '.msg'), name)
        pub = rospy.Publisher(topic, attr, queue_size=1, latch=latched)
        print('publisher active')
        return pub

    def send_command(self, topic, pkg, name, msgd):
        print('command received')
        pub = DynamicNode.activate_publisher(topic, pkg, name, True)
        msg = message_converter.convert_dictionary_to_ros_message(pkg+'/'+name, msgd)
        pub.publish(msg)
        rospy.sleep(3.0)
        # TODO rospy bug, unregister doesn't work correctly
        # pub.unregister()
        print('message delivered')

    def setup_connection(self, topic, pkg, name):
        if topic not in self.publishers:
            self.publishers[topic] = [DynamicNode.activate_publisher(topic, pkg, name, False), pkg, name]
            return True
        else:
            return False

    def stream_command(self, topic, msgd):
        if topic not in self.publishers:
            return False
        msg = message_converter.convert_dictionary_to_ros_message(self.publishers[topic][1] + '/' + self.publishers[topic][2], msgd)
        self.publishers[topic][0].publish(msg)
        print('message delivered')

    # TODO rospy bug, unregister doesn't work correctly
    def tear_down_connection(self, topic):
        self.publishers[topic][0].unregister()
        del self.publishers[topic]

    def read_topic(self, topic, pkg, msg):
        if topic in self.last_msg:
            return self.last_msg[topic]
        else:
            attr = getattr(importlib.import_module(pkg + '.msg'), msg)
            self.subscribers[topic] = rospy.Subscriber(topic, attr, self.reader, callback_args=topic)
            while topic not in self.last_msg:
                self.rate.sleep()
            return self.last_msg[topic]

    def stop_reading(self, topic):
        self.subscribers[topic].unregister()
        print('unsubscribed from '+topic)

    def reader(self, msg, topic):
        self.last_msg[topic] = msg
