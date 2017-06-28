#!/usr/bin/env python
import rospy
import importlib
import collections
import functools
import uuid

from move_base_msgs.msg import MoveBaseActionResult


topic_list = collections.defaultdict(list)
last_value = {}
last_read = {}
goal_reached = False
subscriber_list = []
publisher_list = []
reader_dict = {}
global rate


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
        if cond['type'] == 'condition':
            while cond['id'] not in last_value:
                rate.sleep()
            not_str = 'not ' if cond['not'] else ''
            cond_to_test += not_str + str(last_value[cond['id']]) + str(cond['operator']) + str(cond['val']) + ' '
        elif cond['type'] == 'logicOperator':
            cond_to_test += str(cond['value']) + ' '
    result = eval(cond_to_test)

    while result:
        execute(ist['do'])
        cond_to_test = ''
        for cond in ist['conditions']:
            if cond['type'] == 'condition':
                not_str = 'not ' if cond['not'] else ''
                cond_to_test += not_str + str(last_value[cond['id']]) + str(cond['operator']) + str(cond['val']) + ' '
            elif cond['type'] == 'logicOperator':
                cond_to_test += str(cond['value']) + ' '
        result = eval(cond_to_test)


def manage_if(ist):
    cond_to_test = ''
    for cond in ist['conditions']:
        if cond['type'] == 'condition':
            while cond['id'] not in last_value:
                rate.sleep()
            not_str = 'not ' if cond['not'] else ''
            cond_to_test += not_str + str(last_value[cond['id']]) + str(cond['operator']) + str(cond['val']) + ' '
        elif cond['type'] == 'logicOperator':
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
        last_value[t[1]] = rgetattr(msg, t[0])


def execute(instructions):
    print instructions
    if not instructions:
        for p in publisher_list:
            p.unregister()
        for s in subscriber_list:
            s.unregister()
        return
    for ist in instructions:
        if ist['type'] == 'if':
            manage_if(ist)
        if ist['type'] == 'while':
            manage_while(ist)
        if ist['type'] == 'repeat':
            manage_repeat(ist)
        if ist['type'] == 'capability':
            send_command(ist)


def rsetattr(obj, attr, val):
    pre, _, post = attr.rpartition('.')
    try:
        v = float(val)
    except ValueError:
        v = val
    return setattr(rgetattr(obj, pre) if pre else obj, post, v)


def rgetattr(obj, attr):
    return functools.reduce(getattr, [obj]+attr.split('.'))


def gen_message(jp, attr):
    fields = jp['fields']
    tmsg = attr()
    for f in fields:
        rsetattr(tmsg, f, jp[f])
    return tmsg


def activate_publisher(jp):
    attr = getattr(importlib.import_module(jp['pkg'] + '.msg'), jp['name'])
    pub = rospy.Publisher(jp['topic'], attr, queue_size=1)
    return pub, attr


def send_command(input_d):
    pub, attr = activate_publisher(input_d)
    publisher_list.append(pub)
    msg = gen_message(input_d, attr)
    while not rospy.is_shutdown():
        if pub.get_num_connections() > 0:
            pub.publish(msg)
            break
        rate.sleep()
    # TODO: this is now an hack, we should work on this
    if input_d['topic'] == '/move_base_simple/goal':
        global goal_reached
        goal_reached = False
        wait_for_response()


def run_program(listing):
    condition_flat_list = filterd_flatten(listing['instructions'], 'conditions')

    for i in condition_flat_list:
        if i['type'] == 'if' or i['type'] == 'while':
            for c in i['conditions']:
                if 'topic' in c:
                    attr = getattr(importlib.import_module(c['pkg'] + '.msg'), c['name'])
                    if c['topic'] not in topic_list:
                        subscriber_list.append(rospy.Subscriber(c['topic'], attr, callback, callback_args=c['topic']))
                    topic_list[c['topic']].append([c['field'], c['id']])

    print listing
    execute(listing['instructions'])


def wait_for_response():
    while not goal_reached:
        rate.sleep()


def status_callback(msg):
    global goal_reached
    if msg.status.status == 3:
        goal_reached = True


def init():
    rospy.init_node('dynamic_node', disable_signals=True)
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, status_callback)
    global rate
    rate = rospy.Rate(1)


def read_topic(topic, pkg, msg):
    attr = getattr(importlib.import_module(pkg + '.msg'), msg)
    uid = uuid.uuid4()
    reader_dict[uid] = rospy.Subscriber(topic, attr, reader, callback_args=topic)
    while topic not in last_read:
        rate.sleep()
    return uid


def get_value(topic):
    return last_read[topic]


def stop_reading(uid):
    reader_dict[uid].unregister()


def reader(msg, topic):
    last_read[topic] = msg
