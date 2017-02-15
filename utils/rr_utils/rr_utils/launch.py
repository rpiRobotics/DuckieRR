import yaml
import socket
import sys
from duckie_utils.instantiate_utils import instantiate
import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s

def printConnectionMsg(node_name, obj_name, tcp_port):
    msg =  "%s service started, connect via one of the following:"%(obj_name)
    msg += "\nrr+local:///?nodename=%s&service=%s\n"%(node_name, obj_name)
    if tcp_port is not None:
        msg += "\nrr+tcp://localhost:%s/?service=%s"%(tcp_port, obj_name)
        msg += "\nrr+tcp://localhost:%s/?nodename=%s&service=%s"%(tcp_port,node_name,obj_name)
        msg += "\nrr+tcp://%s.local:%s/?nodename=%s&service=%s"%(socket.gethostname(), tcp_port, node_name, obj_name)
        msg += "\nrr+tcp://<IP_ADDRESS>:%s/?nodename=%s&service=%s"%(tcp_port, node_name,obj_name)
    print msg

def LaunchRRNode(node_name, robdef, objects, tcp_port=None):
    RRN.UseNumpy = True

    # create our local transport 
    t1 = RR.LocalTransport()
    t1.StartServerAsNodeName(node_name)
    RRN.RegisterTransport(t1)

    if tcp_port is not None:
        t2 = RR.TcpTransport()
        t2.EnableNodeAnnounce(RR.IPNodeDiscoveryFlags_NODE_LOCAL | 
            RR.IPNodeDiscoveryFlags_LINK_LOCAL |
            RR.IPNodeDiscoveryFlags_SITE_LOCAL)
        RRN.RegisterTransport(t2)
        t2.StartServer(tcp_port)
        if (tcp_port == 0):
            tcp_port = t2.GetListenPort()

    # check to see if the robdef is a file or the full string...
    if robdef.split('.')[-1] == 'robdef':
        with open(robdef, 'r') as f:
            service_def = f.read()
    else:
        service_def = robdef

    # extract info about the service
    service_name = None
    defined_objects = []
    lines = service_def.splitlines()
    for line in lines:
        if line == '':
            continue
        if line[0] == '#': # comment character
            continue
        tokens = line.split() # any white space is removed
        key = tokens[0].lower()
        if key == 'service':
            service_name = tokens[1]
        if key == 'object':
            defined_objects.append(tokens[1])
        if key == 'import':
            msg = 'Import statements cannot be handled yet'
            raise RuntimeError(msg)

    if service_name is None:
        msg = 'Bad robdef file. Service name not found.'
        raise RuntimeError(msg)

    # Register the service def
    RRN.RegisterServiceType(robdef)

    # check that the objects passed to function are valid and included in the service_def
    if not isinstance(objects, dict):
        msg = 'Expecting objects as a dict, obtained %r'%objects
        raise ValueError(msg)   
    
    obj_list = []
    for node, objDefinition in objects.iteritems():
        obj_name = objDefinition['name']
        # check that we have a valid definition of that object
        if obj_name not in defined_objects:
            msg = 'Object not defined in service definition'
            raise RuntimeError(msg)
        obj_class = objDefinition['class']
        obj_config = objDefinition['configuration']
        if obj_config is not None:
            # check if the config is just a string (assume it is a file)
            if isinstance(obj_config, str):
                with open(obj_config,'r') as f:
                    config = yaml.load(f.read())
            elif isinstance(obj_config, dict):
                config = obj_config # assume we already have the correct dict
            else:
                msg = 'Expecting configuration for %s as a dict or valid yaml file, obtained %r'%(node, obj_config)
        else:
            config = None
        # try instantiating the object
        obj = instantiate(obj_class, config)
        obj_list.append(obj)
        RRN.RegisterService(obj_name, ".".join([service_name, obj_name]), obj)
        printConnectionMsg(node_name, obj_name, tcp_port)

    try:
        while True:
            pass
    except (KeyboardInterrupt,SystemExit):
        for obj in obj_list:
            obj.onShutdown()
        
        # This must be here to prevent segfault
        RRN.Shutdown()
        sys.exit(0)
