#!/usr/bin/env python
import rospy
import json
import importlib
import collections


topic_list = collections.defaultdict(list)
last_value = {}


def send_command(i):
    print i


def filterd_flatten(l, flt):
    for el in l:
        if isinstance(el, list) and not isinstance(el, basestring):
            for sub in filterd_flatten(el, flt):
                yield sub
        else:
            if flt in el:
                yield el


def manage_while(i):
    cond = i['condition']
    while i['id'] not in last_value:
        rate.sleep()
    result = eval(str(last_value[i['id']]) + str(cond['operator']) + str(cond['val']))

    while result:
        execute(i['do'])
        result = eval(str(last_value[i['id']]) + str(cond['operator']) + str(cond['val']))


def manage_if(i):
    cond = i['condition']
    while i['id'] not in last_value:
        rate.sleep()
    result = eval(str(last_value[i['id']]) + str(cond['operator']) + str(cond['val']))

    if result:
        execute(i['then'])
    else:
        execute(i['else'])


def manage_repeat(i):
    val = i['times']
    for _ in range(0, val):
        execute(i['do'])


def callback(msg, topic):
    for t in topic_list[topic]:
        last_value[t[1]] = getattr(msg, t[0])


def execute(instructions):
    if not instructions:
        return
    for i in instructions:
        if 'type' in i:
            if i['type'] == 'if':
                manage_if(i)
            if i['type'] == 'while':
                manage_while(i)
            if i['type'] == 'repeat':
                manage_repeat(i)
        else:
            send_command(i)

if __name__ == '__main__':
    rospy.init_node('dynamic_listener')
    rate = rospy.Rate(1)
    # listing = json.load(open('/home/gianluca/catkin_ws/src/dynamic_node/scripts/if_example.json'))
    # listing = json.load(open('/home/gianluca/catkin_ws/src/dynamic_node/scripts/while_example.json'))
    listing = json.load(open('/home/gianluca/catkin_ws/src/dynamic_node/scripts/repeat_example.json'))
    rospy.loginfo("node ready")
    condition_flat_list = filterd_flatten(listing['instructions'], 'condition')

    for ci in condition_flat_list:
        if ci['type'] == 'if' or ci['type'] == 'while':
            c = ci['condition']
            attr = getattr(importlib.import_module(c['pkg'] + '.msg'), c['name'])
            topic_list[c['topic']].append([c['field'], ci['id']])
            rospy.loginfo("new subscriber on topic %s", c['topic'])
            rospy.Subscriber(c['topic'], attr, callback, callback_args=c['topic'])

    execute(listing['instructions'])
