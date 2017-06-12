from flask import Flask, request, render_template, jsonify
import json
from rdflib import URIRef, Graph, RDF, Namespace
import sys
import codecs

class OntoRobServer():
    __GRAPHFILE = '../out.n3'
    __ONTOROB_RES = Namespace("http://data.open.ac.uk/kmi/ontoRob/resource/");
    __ONTOROB_CLASS = Namespace("http://data.open.ac.uk/kmi/ontoRob#");
    __ONTOROB_PROP = Namespace("http://data.open.ac.uk/kmi/ontoRob/property/");
    
    def __init__(self): 
        self.readKB()

    def setKBfile(self,inputFile):
        """
        load a different file if needed
        """
        self.__GRAPHFILE = inputFile
        self.readKB()
        
    def readKB(self):
         self.__G = Graph()
         self.__G.parse(self.__GRAPHFILE, format="n3")
         
    def getGraph(self):
        return self.__G
        
    def addTopics(self,data):
        for node in data: 
            topic = URIRef(self.__ONTOROB_RES.topic+"/"+ node['topic'][1:])
            self.__G.add( (URIRef(self.__ONTOROB_RES+node['message']), URIRef(self.__ONTOROB_PROP.publishedOn) , topic))
            self.__G.add( (topic, RDF.type, URIRef(self.__ONTOROB_CLASS.Topic)))
    

    def query_KB(self,jsonString):
        """
        TODO change name of method. 
        """
        response_array = list()
        # print jsonString
        
        for item in jsonString : 
            # print "fff", item['message']
            component_obj= { 'node_name' : item['node'], 'msg': item['message'], 'capabs' : []  }
            
            query_string=self.build_query([item['message']]) # TODO : ONE per array?
            
            print query_string
            
            qres = self.__G.query(query_string)
            for row in qres:
                # print row
                
                if len(component_obj['capabs']) == 0 or row.capa not in list(c['type'] for c in component_obj['capabs']):
                    component_obj['capabs'].append( {'type' : row.capa, 'params' : []} )
                
                ix = 0
                for c in component_obj['capabs']:
                    print c
                    if c['type'] == row.capa: 
                        ix = component_obj['capabs'].index(c) 
                    #component_obj['capabs'][row.capa] = []
                component_obj['capabs'][ix]['params'].append(row.param)   
                # component_obj['capabs'][row.capa].append(row.param)
                
                
            response_array.append(component_obj)  
              
        return response_array
        
    def build_query(self,msg_list):
        """
        select capabilities given a set of topics
        """
        values="VALUES(?res) {"
        for msg in msg_list:
            values += "( <"+self.__ONTOROB_RES + msg + ">) "
        values+="}"
    
        query = "SELECT ?capa ?param WHERE { " + values + " ?res <"+self.__ONTOROB_PROP.evokes+"> ?capa . ?res <"+self.__ONTOROB_PROP.hasField +"> ?param . ?capa <"+self.__ONTOROB_PROP.hasParameter +"> ?param . }";
        return query
    
    def getPackage(self,msg):
        query = "SELECT ?pkg WHERE { <"+msg+"> <"+self.__ONTOROB_PROP.hasPkg+"> ?pkg . }"
        
        print 2,query
        qres = self.__G.query(query)
        
        ix = 0
        pkg=""
        res = ""
        for row in qres:
            print row
            if ix > 1: break
            res=row.pkg
            ix+=1   
        return res 
    
    def getMessageName(self,capability, parameters):
        triples=""
        for param in parameters:
            triples+= "?msg <"+self.__ONTOROB_PROP.hasField+"> <"+self.__ONTOROB_RES.field+"/"+param+">. "
        query = "SELECT ?msg WHERE { ?msg <"+self.__ONTOROB_PROP.evokes+"> <"+self.__ONTOROB_RES.capability+"/"+capability+"> . "+ triples + " }"
        
        print 1,query
        qres = self.__G.query(query)
        
        ix = 0
        res = ""
        for row in qres:
            if ix > 1: break
            res=row.msg
            ix+=1
             
        return res
        
    def getTopicFromMsg(self,msg):
        query = "SELECT ?topic WHERE { <"+msg+"> <"+self.__ONTOROB_PROP.publishedOn+"> ?topic .  }"
        print query
        qres = self.__G.query(query)
        
        ix = 0
        topic = ""
        for row in qres:
            if ix > 1: break
            topic=row.topic
            ix+=1
        return topic       
    
    def saveGraph(self):
        s = g.serialize(format='turtle')
        fw = codecs.open("../temp_graph.n3",'w')
        fw.write(s)
        fw.close()
        return
        
app = Flask(__name__)
onto_server = OntoRobServer()
g = onto_server.getGraph()  

@app.route("/")
def index():
    return "OntoRobServer ready for listening"

@app.route("/node/<node>")
def ask_nodes(node):
    """
    queries the KB : which capabilities are evoked by this node?
    """
    result = ""
    qres = g.query(
        """SELECT DISTINCT ?b
           WHERE {
               <http://data.open.ac.uk/kmi/ontoRob/resource/%s> <http://data.open.ac.uk/kmi/ontoRob/property/evokes> ?b.
           } """ % node)
    
    for row in qres:
        result+="%s <br/>" % row
    return result,200

@app.route("/capability/<capa>")
def ask_capability(capa):
    """
    queries the KB : which nodes evoke such capability?
    """
    result = ""
    qres = g.query(
        """SELECT DISTINCT ?b
           WHERE {
              ?b <http://data.open.ac.uk/kmi/ontoRob/property/evokes> <http://data.open.ac.uk/kmi/ontoRob/resource/%s>.
           } """ % capa)

    for row in qres:
        result+="%s <br/>" % row
    return jsonify(result),200

@app.route('/capabilities')
def ask_capabilities():
    
    
    parsed_json = get_nodes()
    
    # add topics dynamically
    onto_server.addTopics(parsed_json)
    
    onto_server.saveGraph()
    
    # get capabilitites
    response_array = onto_server.query_KB(parsed_json)
    
    return jsonify(response_array)

@app.route('/trigger',methods=['GET','POST'])
def triggerCapability():
    """
    in : capability + params
    out : (json) msg + topic + params
    """
    if request.method == 'POST':    
        response = json.loads(str(request.data['capability'])) # TODO check it works
    elif request.method == 'GET':
        response = json.loads(str(request.args['capability']))
    else :
        return "Method not allowed", 403

    robot_input = dict()
    
    # add parameters to robotinput
    robot_input['param_values']=response['parameters']
    fields = list( param for param in robot_input['param_values'].keys())
    robot_input['fields']=fields
    
    # get message name from capability+parameters
    msg = onto_server.getMessageName(response['type'],response['parameters'])
    robot_input['name']=msg.split("/")[-1]
    
    # get pkg
    pkg = onto_server.getPackage(msg)
    robot_input['pkg']=pkg.split("/")[-1]
    
    # get topic from Msg
    topic = onto_server.getTopicFromMsg(msg)
    robot_input['topic']=topic
     
    return jsonify(robot_input),200
     
def get_nodes():
    #TODO : include ROStalker
    return json.loads('[ { "node":"/stageros", "topic":"/odom", "message":"nav_msgs/Odometry", "method":"msg" }, { "node":"/stageros", "topic":"/scan", "message":"sensor_msgs/LaserScan", "method":"msg" }, { "node":"/move_base", "topic":"/move_base/goal", "message":"move_base_msgs/MoveBaseActionGoal", "method":"msg" } ]')   

if __name__ == "__main__":
  
    print "Starting server"
    app.run(debug=True, use_reloader=True, host='0.0.0.0')