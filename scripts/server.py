from flask import Flask, make_response, request, current_app, jsonify
import json
import math
from datetime import timedelta
from functools import update_wrapper

import rosnode
from RosNode import RosNode
from OntoRobServer import OntoRobServer
from dynamic_node import DynamicNode

import parse_instructions


def crossdomain(origin=None, methods=None, headers=None, max_age=21600, attach_to_all=True, automatic_options=True):
    """
    Decorator function that allows crossdomain requests.
    Courtesy of
    https://blog.skyred.fi/articles/better-crossdomain-snippet-for-flask.html
    """
    if methods is not None:
        methods = ', '.join(sorted(x.upper() for x in methods))
    if headers is not None and not isinstance(headers, basestring):
        headers = ', '.join(x.upper() for x in headers)
    if not isinstance(origin, basestring):
        origin = ', '.join(origin)
    if isinstance(max_age, timedelta):
        max_age = max_age.total_seconds()

    def get_methods():
        """ Determines which methods are allowed """
        if methods is not None:
            return methods

        options_resp = current_app.make_default_options_response()
        return options_resp.headers['allow']

    def decorator(f):
        """ The decorator function """

        def wrapped_function(*args, **kwargs):
            """ Caries out the actual cross domain code """
            if automatic_options and request.method == 'OPTIONS':
                resp = current_app.make_default_options_response()
            else:
                resp = make_response(f(*args, **kwargs))
            if not attach_to_all and request.method != 'OPTIONS':
                return resp

            h = resp.headers
            h['Access-Control-Allow-Origin'] = origin
            h['Access-Control-Allow-Methods'] = get_methods()
            h['Access-Control-Max-Age'] = str(max_age)
            h['Access-Control-Allow-Credentials'] = 'true'
            h['Access-Control-Allow-Headers'] = \
                'Origin, X-Requested-With, Content-Type, Accept, Authorization'
            if headers is not None:
                h['Access-Control-Allow-Headers'] = headers
            return resp

        f.provide_automatic_options = False
        return update_wrapper(wrapped_function, f)

    return decorator


app = Flask(__name__)
onto_server = OntoRobServer()
dynamic_node = DynamicNode('dynamic_node')


@app.route('/')
def index():
    return 'OntoRobServer ready for listening'


@app.route('/msg/<msg>')
def ask_nodes(msg):
    """
    queries the KB : which capabilities are evoked by this node?
    """
    result = ''
    g = onto_server.get_graph()
    qres = g.query(
        """SELECT DISTINCT ?b
           WHERE {
               <http://data.open.ac.uk/kmi/ontoRob/resource/%s> <http://data.open.ac.uk/kmi/ontoRob/property/evokes> ?b.
           } """ % msg)
    
    for row in qres:
        result += '%s <br/>' % row
    return result, 200


@app.route('/capability/<capa>', methods=['GET', 'OPTIONS'])
@crossdomain(origin='*')
def ask_capability(capa):
    """
    queries the KB : which nodes evoke such capability?
    """
    print capa
    result = ''
    g = onto_server.get_graph()
    qres = g.query(
        """SELECT DISTINCT ?b
           WHERE {
              ?b <http://data.open.ac.uk/kmi/ontoRob/property/evokes> <http://data.open.ac.uk/kmi/ontoRob/resource/%s>.
           } """ % capa)

    for row in qres:
        result += '%s <br/>' % row
    return jsonify(result), 200


@app.route('/capabilities', methods=['GET', 'OPTIONS'])
@crossdomain(origin='*')
def ask_capabilities():
    print 'Received capabilities request'
    parsed_json = get_nodes()
    # add topics dynamically
    # TODO why do we need two of these?
    # can we make query_kb independent from parsed_json?
    # inizialization should be before everything and not triggered from outside
    onto_server.add_topics(parsed_json)
    response_array = onto_server.query_kb(parsed_json)

    for i in reversed(range(0, len(response_array))):
        if len(response_array[i]['capabs']) == 0:
            del response_array[i]

    print 'Capabilities successfully queried'
    return json.dumps(response_array)


@app.route('/execute', methods=['POST', 'OPTIONS'])
@crossdomain(origin='*')
def execute():
    """
    receive program from UI in json format
    execute it directly
    """
    print 'Received execute'
    program = json.loads(request.data)['program']
    onto_server.update_program_with_msg_and_pkg(program)

    execute_on_robot(program)
    return 'OK', 200


@app.route('/run', methods=['POST', 'OPTIONS'])
@crossdomain(origin='*')
def run():
    print 'Execute single command'
    print request.data
    j = json.loads(request.data)
    if check_command_consistency(j):
        dynamic_node.send_command(j['topic'], j['pkg'], j['name'], DynamicNode.flat_to_nested_dict(j['message']))
        return 'OK', 200
    else:
        return 'ERROR', 500


@app.route('/deprecated_read', methods=['GET', 'OPTIONS'])
@crossdomain(origin='*')
def deprecated_read():
    """
    receive a list of topic in json format
    provides the newest messesage from those topics
    """
    try:
        qs = json.loads(request.args['question'])
        ret = {}

        for q in qs:
            topic = q['topic']
            capability = q['capability']
            topic_uri = onto_server.get_ontorob_res_namespace().topic + q['topic']
            # this can be done in the else, by improving the query
            # in the get_params_from_msg method (making it retrievable from the capability)
            q_res = onto_server.get_msg_and_pkg(topic_uri, capability)
            pkg = q_res['pkg']
            msg = q_res['msg']

            parameters_to_read = onto_server.get_params_from_msg(msg)

            # performs topic subscription only once, otherwise read from an existing dictionary
            reading = dynamic_node.read_topic(topic, OntoRobServer.get_name_from_uri(pkg), OntoRobServer.get_name_from_uri(msg))
            ros_msg_dict = ros_msg_2_dict(reading, parameters_to_read)
            key = capability + '/' + topic
            ret[key] = ros_msg_dict

        resp = app.response_class(response=json.dumps(ret), status=200, mimetype='application/json')
 
        return resp
    except Exception, e:
        print str(e)
        return 'ERROR', 500


@app.route('/read', methods=['POST', 'OPTIONS'])
@crossdomain(origin='*')
def read():
    """
    receive a list of topic in json format
    provides the newest messesage from those topics
    """
    try:
        qs = json.loads(request.data)
        ret = list()

        for q in qs:
            topic = q['topic']
            capability = q['capability']
            topic_uri = onto_server.get_ontorob_res_namespace().topic + q['topic']
            # this can be done in the else, by improving the query
            # in the get_params_from_msg method (making it retrievable from the capability)
            q_res = onto_server.get_msg_and_pkg(topic_uri, capability)
            pkg = q_res['pkg']
            msg = q_res['msg']

            parameters_to_read = onto_server.get_params_from_msg(msg)

            # performs topic subscription only once, otherwise read from an existing dictionary
            reading = dynamic_node.read_topic(topic, OntoRobServer.get_name_from_uri(pkg),
                                              OntoRobServer.get_name_from_uri(msg))
            ros_msg_dict = ros_msg_2_dict(reading, parameters_to_read)
            ret.append({'capability': capability, 'topic': topic, 'parameters': ros_msg_dict})

        return app.response_class(response=json.dumps(ret), status=200, mimetype='application/json')
    except Exception, e:
        print str(e)
        return 'ERROR', 500


def check_command_consistency(msgj):
    g = onto_server.get_graph()
    qres = g.query('ASK {'
                   '<http://data.open.ac.uk/kmi/ontoRob/resource/'+msgj['pkg']+'/'+msgj['name']+'>'
                   '<http://data.open.ac.uk/kmi/ontoRob/property/evokes>'
                   '<'+msgj['capability']+'> }')
    qres1 = g.query('ASK {'
                    '<http://data.open.ac.uk/kmi/ontoRob/resource/' + msgj['pkg'] + '/' + msgj['name'] + '>'
                    '<http://data.open.ac.uk/kmi/ontoRob/property/publishedOn>'
                    '<http://data.open.ac.uk/kmi/ontoRob/resource/topic/' + msgj['topic'] + '> }')
    # return qres.askAnswer and qres1.askAnswer
    return qres.askAnswer


def get_nodes():
    nodelist = rosnode.get_node_names()
    msgs_topic_collection = []

    for nodename in nodelist:
        # initialise RosNode object
        node = RosNode(nodename)
        msgs_topic_collection = update_msgs_collection(node.get_capability_msg(), msgs_topic_collection)
        # msgs_topic_collection.extend(node.get_capability_msg())

    # this transform every set in a list in the field 'nodes'
    for item in msgs_topic_collection:
        item['nodes'] = list(item['nodes'])

    return msgs_topic_collection


def update_msgs_collection(cur_node_msgs, msgs_topic_collection):
    for msg in cur_node_msgs:
        topic_message_found = False
        topic = msg['topic']
        message = msg['message']
        node_name = msg['node']

        for msg_topic in msgs_topic_collection:
            if msg_topic['topic'] == topic and msg_topic['message'] == message:
                msg_topic['nodes'].add(node_name)
                topic_message_found = True

        if not topic_message_found:
            new_msg_topic = {}
            new_msg_topic['topic'] = topic
            new_msg_topic['message'] = message
            new_msg_topic['nodes'] = set([node_name])
            new_msg_topic['capabilities'] = []
            msgs_topic_collection.append(new_msg_topic)

    return msgs_topic_collection


def execute_on_robot(program):
    print 'starting execution'
    parse_instructions.run_program(program)


def ros_msg_2_dict(ros_msg_obj, parameters_to_read):
    ret = {}

    for param in parameters_to_read:
        if '__LIST' in param:
            param = param.replace('__LIST', '')

        value = dynamic_node.rgetattr(ros_msg_obj, param)
        if type(value) == list or type(value) == tuple:
            value = list(value)
            if type(value[0]) == float:
                for i in range(0, len(value)):
                    if math.isnan(value[i]):
                        value[i] = None
                    if value[i] == float('inf'):
                        value[i] = 9999999

        ret[param] = value

    return ret


if __name__ == '__main__':
    print 'Starting server'
    # parse_instructions.init()
    # app.run(debug=True, use_reloader=True, threaded=True, host='0.0.0.0')
    app.run(threaded=True, host='0.0.0.0')
