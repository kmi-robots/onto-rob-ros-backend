from rdflib import Graph,Namespace, URIRef, Literal
from rdflib.namespace import RDF,RDFS
import codecs
import sys

my_graph = Graph()

ontorob_res = Namespace("http://data.open.ac.uk/kmi/ontoRob/resource/");
ontorob_class = Namespace("http://data.open.ac.uk/kmi/ontoRob#");
ontorob_prop = Namespace("http://data.open.ac.uk/kmi/ontoRob/property/");

def readCSV(inFile):
    """
    returns csv without headers
    """
    fr = codecs.open(inFile,"r")    
    lines = fr.readlines()[1:]
    return lines

def addMsg(l):
    """
    returns the triples to add 
    """
    obj = URIRef(ontorob_class+l[0]) # node type (act | msg | srv)
    subj = URIRef(ontorob_res+l[1]+"/"+l[2]) # msg_name 
    pkg = URIRef(ontorob_res.package+"/"+l[1]) # ros  package name
    return [ (subj, RDF.type, obj), (subj, RDFS.label, Literal(l[2],"en")), (subj, ontorob_prop.hasPkg, pkg) ]

 

def addCapabilities(l):
    """
    returns capabilities triples
    """
    caps = l[3][1:-1].split("; ") # capabilities []
    msg = URIRef(ontorob_res+l[1]+"/"+l[2]) # msg name
    
    field_type = l[4]
    params = l[5][1:-1].split("]; ") # capability_params [[]]
    result = list()
    
    for i in range(0,len(caps)):
        cap =  caps[i]
        onto_cap = URIRef(ontorob_res.capability+"/"+cap)
        result.append( ( msg,ontorob_prop.evokes, onto_cap) )
        result.append( (onto_cap , RDF.type, ontorob_class.Capability) )
        if 'r' in field_type :
            result.append( (onto_cap , ontorob_prop.hasParamType, Literal("read","en")) )
        if 'w' in field_type:
            result.append( ( onto_cap, ontorob_prop.hasParamType, Literal("write","en")) )
        
        if len(params[i]) == 0 :
            continue
        if params[i][-1] == ']' : params[i]=params[i][:-1] 
        
        cap_params = params[i][1:].split("; ")

        for cap_param in cap_params:
            
            result.append( (onto_cap, URIRef(ontorob_prop.hasParameter), URIRef(ontorob_res.field+"/"+cap_param) ) ) 
            result.append( ( msg, URIRef(ontorob_prop.hasField), URIRef(ontorob_res.field+"/"+cap_param) ) )
            result.append( ( URIRef(ontorob_res.field+"/"+cap_param), RDF.type, ontorob_class.Field) )
           
                
    return result


def saveGraph(g):
    s = g.serialize(format='turtle')
    # print s
    fw = codecs.open("../out.n3",'w')
    fw.write(s)
    fw.close()
    
def buildKB(inputF):
    """
    gets a csv and builds a n3 file
    """
    lines = readCSV(inputF)
    for l in lines:
        # TODO add capability hasCapParameter field
        current_line = l.strip().split(',')
        for triple in addMsg(current_line):  my_graph.add( triple)
                
        for triple in addCapabilities(current_line) : 
            # print triple
            my_graph.add(triple)
        # for triple in addParams(l): my_graph.add(triple)
    
    
    saveGraph(my_graph)
    
if __name__ == "__main__":
    if len(sys.argv) == 1 :   
        inputFile = "../msg-srv.csv"
    else:
        inputFile = sys.argv[1]
    
    buildKB(inputFile)
