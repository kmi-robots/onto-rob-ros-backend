import rospy
import importlib
import functools
from rospy_message_converter import message_converter


class DynamicNode:

    def __init__(self, name):
        rospy.init_node(name, disable_signals=True)
        self.command = dict()
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

    def gen_message(self, attr):
        fields = self.command['fields']
        tmsg = attr()
        for f in fields:
            DynamicNode.rsetattr(tmsg, f, self.command[f])
        return tmsg

    def activate_publisher(self):
        # create a message of the 'pkg'/'name' type
        attr = getattr(importlib.import_module(self.command['pkg'] + '.msg'), self.command['name'])
        pub = rospy.Publisher(self.command['topic'], attr, queue_size=1)
        print('publisher active')
        return pub, attr

    def send_command(self, input_d):
        print('command received')
        self.command = input_d
        pub, attr = self.activate_publisher()
        msg = self.gen_message(attr)
        while not rospy.is_shutdown():
            if pub.get_num_connections() > 0:
                pub.publish(msg)
                print('message delivered')
                break
            self.rate.sleep()
        # TODO rospy bug, unregister doesn't work correctly
        # pub.unregister()

    def stream_command(self, input_d):
        self.command = input_d
        if self.command['topic'] not in self.publishers:
            self.publishers[self.command['topic']], attr = self.activate_publisher()
            self.rate.sleep()
        else:
            attr = getattr(importlib.import_module(self.command['pkg'] + '.msg'), self.command['name'])
        msg = self.gen_message(attr)
        self.publishers[self.command['topic']].publish(msg)
        print('message delivered')

    # TODO rospy bug, unregister doesn't work correctly
    def stop_streaming(self, topic):
        self.publishers[topic].unregister()

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
