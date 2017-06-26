#!/usr/bin/env python
import rospy
import json
import importlib
import collections


topic_list = collections.defaultdict(list)
last_value = {}


def send_command(ist):
    rospy.loginfo("%s", str(ist))


def filterd_flatten(l, flt):
    for el in l:
        if isinstance(el, list) and not isinstance(el, basestring):
            for sub in filterd_flatten(el, flt):
                yield sub
        else:
            if flt in el:
                yield el


def manage_while(ist):
    cond_to_test = ''
    for cond in ist['conditions']:
        if 'type' not in cond:
            while cond['id'] not in last_value:
                rate.sleep()
            not_str = 'not ' if cond['not'] else ''
            cond_to_test += not_str + str(last_value[cond['id']]) + str(cond['operator']) + str(cond['val']) + ' '
        else:
            cond_to_test += str(cond['value']) + ' '
    rospy.loginfo('%s', cond_to_test)
    result = eval(cond_to_test)

    while result:
        execute(ist['do'])
        cond_to_test = ''
        for cond in ist['conditions']:
            if 'type' not in cond:
                not_str = 'not ' if cond['not'] else ''
                cond_to_test += not_str + str(last_value[cond['id']]) + str(cond['operator']) + str(cond['val']) + ' '
            else:
                cond_to_test += str(cond['value']) + ' '
        rospy.loginfo('%s', cond_to_test)
        result = eval(cond_to_test)


def manage_if(ist):
    cond_to_test = ''
    for cond in ist['conditions']:
        if 'type' not in cond:
            while cond['id'] not in last_value:
                rate.sleep()
            not_str = 'not ' if cond['not'] else ''
            cond_to_test += not_str + str(last_value[cond['id']]) + str(cond['operator']) + str(cond['val']) + ' '
        else:
            cond_to_test += str(cond['value']) + ' '
    result = eval(cond_to_test)

    if result:
        execute(i['then'])
    else:
        execute(i['else'])


def manage_repeat(ist):
    val = ist['times']
    for _ in range(0, val):
        execute(i['do'])


def callback(msg, topic):
    for t in topic_list[topic]:
        last_value[t[1]] = getattr(msg, t[0])


def execute(instructions):
    if not instructions:
        return
    for ist in instructions:
        if 'type' in ist:
            if ist['type'] == 'if':
                manage_if(ist)
            if ist['type'] == 'while':
                manage_while(ist)
            if ist['type'] == 'repeat':
                manage_repeat(ist)
        else:
            send_command(ist)

if __name__ == '__main__':
    rospy.init_node('dynamic_listener')
    rate = rospy.Rate(1)
    listing = json.load(open('if_example.json'))
    # listing = json.load(open('while_example.json'))
    # listing = json.load(open('/home/gianluca/catkin_ws/src/dynamic_node/scripts/repeat_example.json'))
    # rospy.loginfo("node ready")
    condition_flat_list = filterd_flatten(listing['instructions'], 'conditions')

    for i in condition_flat_list:
        if i['type'] == 'if' or i['type'] == 'while':
            for c in i['conditions']:
                if 'topic' in c:
                    attr = getattr(importlib.import_module(c['pkg'] + '.msg'), c['name'])
                    if c['topic'] not in topic_list:
                        rospy.loginfo("new subscriber on topic %s", c['topic'])
                        rospy.Subscriber(c['topic'], attr, callback, callback_args=c['topic'])
                    topic_list[c['topic']].append([c['field'], c['id']])

    execute(listing['instructions'])
