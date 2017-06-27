#!/usr/bin/env python
import rospy
import importlib
import collections
import functools


topic_list = collections.defaultdict(list)
last_value = {}
rate = rospy.Rate(1)


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
        execute(ist['then'])
    else:
        execute(ist['else'])


def manage_repeat(ist):
    val = ist['times']
    for _ in range(0, val):
        execute(ist['do'])


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


def rsetattr(obj, attr, val):
    pre, _, post = attr.rpartition('.')
    try:
        v = float(val)
    except ValueError:
        v = val
    return setattr(functools.reduce(getattr, [obj]+pre.split('.')) if pre else obj, post, v)


def gen_message(jp, attr):
    fields = jp['fields']
    tmsg = attr()
    for f in fields:
        rsetattr(tmsg, f, jp['param_values'][f])
    return tmsg


def activate_publisher(jp):
    attr = getattr(importlib.import_module(jp['pkg'] + '.msg'), jp['name'])
    pub = rospy.Publisher(jp['topic'], attr, queue_size=1)
    return pub, attr


def send_command(input_d):
    pub, attr = activate_publisher(input_d)
    msg = gen_message(input_d, attr)
    while not rospy.is_shutdown():
        if pub.get_num_connections() > 0:
            pub.publish(msg)
            break
        rate.sleep()


def run_program(listing):
    rospy.init_node('dynamic_node', disable_signals=True)
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
