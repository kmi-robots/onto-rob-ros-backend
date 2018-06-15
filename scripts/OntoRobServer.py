from rdflib import URIRef, Graph, RDF, Namespace
import codecs


class OntoRobServer:
    __GRAPHFILE = '../out.n3'
    __ONTOROB_RES = Namespace("http://data.open.ac.uk/kmi/ontoRob/resource/")
    __ONTOROB_CLASS = Namespace("http://data.open.ac.uk/kmi/ontoRob#")
    __ONTOROB_PROP = Namespace("http://data.open.ac.uk/kmi/ontoRob/property/")

    def __init__(self):
        self.__G = Graph()
        self.read_kb()

    @staticmethod
    def get_name_from_uri(string):
        return string[string.rfind('/') + 1:]

    def get_ontorob_res_namespace(self):
        return self.__ONTOROB_RES

    def set_kb_file(self, input_file):
        """
        load a different file if needed
        """
        self.__GRAPHFILE = input_file
        self.read_kb()

    def read_kb(self):
        self.__G.parse(self.__GRAPHFILE, format="n3")

    def get_graph(self):
        return self.__G

    def add_topics(self, data):
        for node in data:
            topic = URIRef(self.__ONTOROB_RES.topic + "/" + node['topic'][1:])
            self.__G.add((URIRef(self.__ONTOROB_RES + node['message']), URIRef(self.__ONTOROB_PROP.publishedOn), topic))
            self.__G.add((topic, RDF.type, URIRef(self.__ONTOROB_CLASS.Topic)))

    def query_kb(self, json_str):
        """
        TODO change name of method.
        """
        response_array = list()

        for item in json_str:
            component_obj = {'msg': item['message'], 'topic': item['topic'], 'capabs': []}

            query_string = self.build_query([item['message']])

            qres = self.__G.query(query_string)

            for row in qres:
                if len(component_obj['capabs']) == 0 or row.capa not in list(
                        c['type'] for c in component_obj['capabs']):
                    component_obj['capabs'].append({'type': row.capa, 'params': []})

                ix = 0
                for c in component_obj['capabs']:
                    if c['type'] == row.capa:
                        ix = component_obj['capabs'].index(c)

                component_obj['capabs'][ix]['mode'] = row.parType

                # TODO manage empty messages
                if not row.param.endswith("/"):
                    component_obj['capabs'][ix]['params'].append({"p": row.param, "mode": row.parType})

            # TODO: if you remove this if, move_base_simple, which has a PoseStamped msg
            # therefore evokes a Navigation capab, won't appear in the
            # if len(component_obj['capabs']) != 0:
            response_array.append(component_obj)

        # response_array = []
        return response_array

    def build_query(self, msg_list):
        """
        select capabilities given a set of topics
        """
        values = "VALUES(?res) {"
        for msg in msg_list:
            values += "( <" + self.__ONTOROB_RES + msg + ">) "
        values += "}"

        query = "SELECT ?capa ?param ?parType WHERE { " + values + " ?res <" + self.__ONTOROB_PROP.evokes + "> ?capa . ?res <" + self.__ONTOROB_PROP.hasField + "> ?param . ?capa <" + self.__ONTOROB_PROP.hasParameter + "> ?param . ?capa <" + self.__ONTOROB_PROP.hasParamType + "> ?parType .}";
        return query

    def fill_msg_and_pkg(self, instruction):
        print "Filling!"

        if instruction["type"] == "capability":
            topic = self.__ONTOROB_RES.topic + instruction["topic"]
            capability = instruction["capability"]
            r = self.get_msg_and_pkg(topic, capability)
            instruction["pkg"] = OntoRobServer.get_name_from_uri(r["pkg"])
            instruction["name"] = OntoRobServer.get_name_from_uri(r["msg"])

        elif instruction["type"] == "if":
            conditions = instruction["conditions"]
            for condition in conditions:
                if condition["type"] == "condition":
                    topic = self.__ONTOROB_RES.topic + condition["topic"]
                    r = self.get_msg_and_pkg_from_topic(topic)
                    condition["pkg"] = OntoRobServer.get_name_from_uri(r["pkg"])
                    condition["name"] = OntoRobServer.get_name_from_uri(r["msg"])

            thens = instruction["then"]
            for then in thens:
                self.fill_msg_and_pkg(then)

            elses = instruction["else"]
            for els in elses:
                self.fill_msg_and_pkg(els)

        elif instruction["type"] == "while":
            conditions = instruction["conditions"]
            for condition in conditions:
                topic = self.__ONTOROB_RES.topic + condition["topic"]
                r = self.get_msg_and_pkg_from_topic(topic)
                condition["pkg"] = OntoRobServer.get_name_from_uri(r["pkg"])
                condition["name"] = OntoRobServer.get_name_from_uri(r["msg"])

            dos = instruction["do"]
            for do in dos:
                self.fill_msg_and_pkg(do)

        elif instruction["type"] == "repeat":
            dos = instruction["do"]
            for do in dos:
                self.fill_msg_and_pkg(do)

    def update_program_with_msg_and_pkg(self, program):
        for instruction in program["instructions"]:
            self.fill_msg_and_pkg(instruction)

    def get_msg_and_pkg(self, topic, capability):
        q = "SELECT ?msg ?pkg WHERE {  ?msg <" + self.__ONTOROB_PROP.evokes + "> <" + capability + "> . ?msg <" + self.__ONTOROB_PROP.publishedOn + "> <" + topic + "> . ?msg <" + self.__ONTOROB_PROP.hasPkg + "> ?pkg } "

        qres = self.__G.query(q)
        ix = 0
        ret = {}
        for row in qres:
            # TODO watch out
            if ix > 1:
                print "Multiple result"
                break

            ret["pkg"] = str(row.pkg)
            ret["msg"] = str(row.msg)

        return ret

    def get_msg_and_pkg_from_topic(self, topic):
        q = "SELECT ?msg ?pkg WHERE {  ?msg <" + self.__ONTOROB_PROP.publishedOn + "> <" + topic + "> . ?msg <" + self.__ONTOROB_PROP.hasPkg + "> ?pkg } "

        qres = self.__G.query(q)
        ix = 0
        ret = {}

        for row in qres:
            # TODO watch out
            if ix > 1:
                print "Multiple result"
                break

            ret["pkg"] = str(row.pkg)
            ret["msg"] = str(row.msg)

        return ret

        # TODO maybe the query should be performed starting from the capability, not the msg

    # msg is assumed to be already in its URI form
    def get_params_from_msg(self, msg):
        q = "SELECT ?param WHERE {  <" + msg + "> <" + self.__ONTOROB_PROP.hasField + "> ?param } "
        qres = self.__G.query(q)

        ret = []

        for row in qres:
            ret.append(OntoRobServer.get_name_from_uri(str(row.param)))

        return ret

    def get_package(self, msg):
        """
        TODO remove this fct if unused
        """
        query = "SELECT ?pkg WHERE { <" + msg + "> <" + self.__ONTOROB_PROP.hasPkg + "> ?pkg . }"

        # print 2,query
        qres = self.__G.query(query)

        ix = 0
        res = ""
        for row in qres:
            # print row
            if ix > 1:
                break
            res = row.pkg
            ix += 1
        return res

    def get_message_name(self, capability, parameters):
        triples = ""
        for param in parameters:
            triples += "?msg <" + self.__ONTOROB_PROP.hasField + "> <" + self.__ONTOROB_RES.field + "/" + param + ">. "
        query = "SELECT ?msg WHERE { ?msg <" + self.__ONTOROB_PROP.evokes + "> <" + capability + "> . " + triples + " }"

        # print 1,query
        qres = self.__G.query(query)

        ix = 0
        res = ""
        for row in qres:
            if ix > 1:
                break
            res = row.msg
            ix += 1

        return res

    def get_topic_from_msg(self, msg):
        query = "SELECT ?topic WHERE { <" + msg + "> <" + self.__ONTOROB_PROP.publishedOn + "> ?topic .  }"
        # print query
        qres = self.__G.query(query)

        ix = 0
        topic = ""
        for row in qres:
            if ix > 1: break
            topic = row.topic
            ix += 1
        return topic

    def save_graph(self):
        s = self.__G.serialize(format='turtle')
        fw = codecs.open("../temp_graph.n3", 'w')
        fw.write(s)
        fw.close()
