import json
import urllib2
import pprint


class OntoRobRosInterface:

    def __init__(self, location='localhost', capability=None):
        self.cprx = 'http://data.open.ac.uk/kmi/ontoRob/resource/capability/'
        self.pprx = 'http://data.open.ac.uk/kmi/ontoRob/resource/field/'
        self.location = 'http://'+location+':5000'
        if capability:
            # TODO how does this work?
            contents = urllib2.urlopen(self.location+'/capabilities').read()
        else:
            contents = urllib2.urlopen(self.location+'/capabilities').read()

        self.aug_topics = json.loads(contents)
        return

    def get_topic_list(self, capability=None):
        if capability:
            topics = set()
            for at in self.aug_topics:
                for c in at['capabs']:
                    if c['type'] == self.cprx + capability:
                        topics.add(str(at['topic']))
            return list(topics)
        else:
            return [str(at['topic']) for at in self.aug_topics]

    def get_capability_list(self):
        capablities = set()
        for at in self.aug_topics:
            for c in at['capabs']:
                capablities.add(str(c['type']).replace(self.cprx, ''))
        return list(capablities)

    def topic_in_capability(self, topic, capability):
        topics = self.get_topic_list(capability)
        return topic in topics

    def get_message_fields(self, topic, capability):
        message = list()
        for at in self.aug_topics:
            if at['topic'] == topic:
                for c in at['capabs']:
                    if c['type'] == self.cprx + capability:
                        for p in c['params']:
                            message.append(str(p['p']).replace(self.pprx, ''))
                        break
                break
        return message

    def get_message_type(self, topic):
        for at in self.aug_topics:
            if at['topic'] == topic:
                msg_type = str(at['msg']).split('/')
                return msg_type[0], msg_type[1]

    def publish_on_topic(self, topic, capability, message, repeat=False):
        if not repeat:
            pkg, name = self.get_message_type(topic)
            data = {'capability': self.cprx + capability,
                    'name': name, 'message': message, 'topic': topic, 'pkg': pkg, 'type': 'capability'}
            r = urllib2.Request(self.location+'/run', json.dumps(data), {'Content-Type': 'application/json'})
            urllib2.urlopen(r)

    def read_from_topic(self, topic, capability):
        data = [{'capability': self.cprx + capability, 'topic': topic}]
        r = urllib2.Request(self.location+'/read', json.dumps(data), {'Content-Type': 'application/json'})
        f = urllib2.urlopen(r)
        response = f.read()
        f.close()
        return json.loads(str(response).replace(self.cprx, ''))

    def read_from_topics(self, topics, capabilities):
        data = list()
        for t, c in topics, capabilities:
            data.append({'capability': self.cprx + c, 'topic': t})
        r = urllib2.Request(self.location+'/read', json.dumps(data), {'Content-Type': 'application/json'})
        f = urllib2.urlopen(r)
        response = f.read()
        f.close()
        return json.loads(str(response).replace(self.cprx, ''))
