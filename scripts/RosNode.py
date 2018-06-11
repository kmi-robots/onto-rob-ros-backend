#! /usr/bin/env python

import rosnode
import rosservice
import rostopic


class RosNode(object):
    def __init__(self, name):
        self.published_topics = []
        self.subscribed_topics = []
        self.services = []
        self.name = name

        nodeinfo = rosnode.get_node_info_description(self.name)
        lines = nodeinfo.split("\n")

        count = 0
        line = lines[count]

        while not line.startswith("Publications:") and count < len(lines):
            count += 1
            line = lines[count]

        count += 1
        line = lines[count]

        while line.strip().startswith("*") and count < len(lines):
            # new published topic
            topic = line.rstrip().split(" ")[2]
            msg = rostopic.get_topic_type(topic)[0]
            pub_topic = {'topic': topic, 'msg': msg}
            self.published_topics.append(pub_topic)
            count += 1
            line = lines[count]

        while not line.startswith("Subscriptions:") and count < len(lines):
            count += 1
            line = lines[count]
        
        count += 1
        line = lines[count]

        while line.strip().startswith("*") and count < len(lines):
            # new subscribed topic
            # topic name
            topic = line.rstrip().split(" ")[2]
            # topic type
            msg = rostopic.get_topic_type(topic)[0]

            sub_topic = {"topic": topic, "msg": msg}
            self.subscribed_topics.append(sub_topic)
            count += 1
            line = lines[count]

        while not line.startswith("Services:") and count < len(lines):
            count += 1
            line = lines[count]
        
        count += 1
        line = lines[count]

        while line.strip().startswith("*") and count < len(lines):
            # get service name
            srv = line.split(" ")[2]
            srv_type = rosservice.get_service_type(srv)
            service = {"name": srv, "msg": srv_type}
            self.services.append(service)
            count += 1
            line = lines[count]

    def get_capability_msg(self):
        ret = []
        for pt in self.published_topics:
            to_add = {"method": "msg", "act": "pub", "node": self.name, "topic": pt["topic"], "message": pt["msg"]}
            ret.append(to_add)

        for st in self.subscribed_topics:
            to_add = {"method": "msg", "act": "sub", "node": self.name, "topic": st["topic"], "message": st["msg"]}
            ret.append(to_add)
            
        for srv in self.services:
            to_add = {"method": "srv", "node": self.name, "topic": srv["name"], "message": srv["msg"]}
            ret.append(to_add)

        return ret
