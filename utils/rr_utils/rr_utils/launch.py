import yaml
import socket
import sys
from duckie_utils.instantiate_utils import instantiate
import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s


def FormatRobdefString(robdef):
    newlinechar = '\n'+ ' '*10
    return '|'+ newlinechar + robdef.replace('\n', newlinechar).strip() + '\n'

def printConnectionMsg(node_name, obj_name, tcp_port):
    msg =  "[Launcher] %s service started, connect via one of the following:"%(obj_name)
    msg += "\n\trr+local:///?nodename=%s&service=%s"%(node_name, obj_name)
    if tcp_port is not None:
        msg += "\n\trr+tcp://localhost:%s/?service=%s"%(tcp_port, obj_name)
        msg += "\n\trr+tcp://localhost:%s/?nodename=%s&service=%s"%(tcp_port,node_name,obj_name)
        msg += "\n\trr+tcp://%s.local:%s/?nodename=%s&service=%s"%(socket.gethostname(), tcp_port, node_name, obj_name)
        msg += "\n\trr+tcp://<IP_ADDRESS>:%s/?nodename=%s&service=%s"%(tcp_port, node_name,obj_name)
    print msg

def LaunchRRNode(node_name, objects, tcp_port=None):
    RRN.UseNumpy = True

    # create our local transport 
    t1 = RR.LocalTransport()
    t1.StartServerAsNodeName(node_name)
    RRN.RegisterTransport(t1)

    if tcp_port is not None:
        tcp_port = int(tcp_port)

        t2 = RR.TcpTransport()
        t2.EnableNodeAnnounce(RR.IPNodeDiscoveryFlags_NODE_LOCAL | 
            RR.IPNodeDiscoveryFlags_LINK_LOCAL |
            RR.IPNodeDiscoveryFlags_SITE_LOCAL)
        RRN.RegisterTransport(t2)
        t2.StartServer(tcp_port)
        if (tcp_port == 0):
            tcp_port = t2.GetListenPort()


    # check that the objects passed to function are a list
    if not isinstance(objects, list):
        msg = 'Expecting objects as a list, obtained %r'%objects
        raise ValueError(msg)   
    
    launched_objects = []
    for obj in objects:
        #extract the robdef
        if not isinstance(obj, dict):
            msg = 'Expecting each object as a dictionary. Obtained %r'%obj
        
        # get the object name
        obj_name = obj['name']
        print "[Launcher] Launching %s"%obj_name
        # get the robdef
        robdef = obj['robdef']
        if not isinstance(robdef, str):
            msg = 'For object ''%s'' : Expecting robdef as a string, obtained %r'%(obj_name,robdef)
            raise ValueError(msg) 
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
                msg = '[Launcher] WARNING: import statement detected.\n'
                msg += 'If objects are defined in imported file they may not be found.\n'
                msg += 'All files that import another service should be listed after that service.'
                print msg

        if service_name is None:
            msg = 'Bad robdef file. Service name not found.'
            raise RuntimeError(msg)

        # Register the service def
        RRN.RegisterServiceType(robdef)
        
        # determine the class that implements it.
        if 'class' in obj:
            obj_class = obj['class']
            # check that we have a valid definition of that object
            if obj_name not in defined_objects:
                msg = "Object '%s' is not defined in service definition"%(obj_name)
                raise RuntimeError(msg)
            
            # check if there is any configuration specified
            if 'configuration' in obj:
                obj_config = obj['configuration']
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
            launched_objects.append(obj)
            RRN.RegisterService(obj_name, ".".join([service_name, obj_name]), obj)
            printConnectionMsg(node_name, obj_name, tcp_port)

    try:
        while True:
            pass
    except (KeyboardInterrupt,SystemExit):
        for obj in launched_objects:
            obj.onShutdown()
        
        # This must be here to prevent segfault
        RRN.Shutdown()
        sys.exit(0)
