from abc import ABCMeta, abstractmethod
import time
import RobotRaconteur
RRN = RobotRaconteur.RobotRaconteurNode.s

class abstract_attribute(object):
    def __get__(self,obj, type):
        '''
        Iterate over the names on the class, and all its superclasses, 
        and try to find the attribute name for this descriptor
        '''
        # traverse the parents in the method resolution order
        for cls in type.__mro__:
            # for each cls this, see what attributes they set
            for name,value in cls.__dict__.iteritems():
                # we found oruselves here
                if value is self:
                    # if the property gets accessed as Child.variable,
                    # obj will be done. For this case
                    # If accessed as a_child.variable, the class Chile is 
                    # in the type, and a_child in the obj.
                    this_obj = obj if obj else type

                    raise NotImplementedError(
                        "%r does not have the attribute %r "
                        "(abstract from class %r"%
                            (this_obj, name, cls.__name__))
        raise NotImplementedError(
            "%s does not set the abstract attribute <unknown>"%(type.__name__))

class RRNodeInterface():
    __metaclass__ = ABCMeta

    node_name = abstract_attribute()

    @abstractmethod
    def onShutdown(self):
        pass

    def log(self,msg):
        print "[%s] %s"%(self.node_name,msg)

    def FindAndConnect(self, nodeType, required=True, attempts=2, TO=5, tcp=False):
        transports = ["rr+local"]
        if tcp:
            transports.append("rr+tcp")

        for attempt in xrange(1,attempts+1):
            res=RRN.FindServiceByType(nodeType,transports)
            if (len(res)==0):
                msg = "WARNING: Could not find the %s Interface.\n"%(nodeType)
                msg += "Attempt (%d/%d). Will wait %d seconds and try again..."%(attempt, attempts, TO)
                self.log(msg)
                time.sleep(TO)
            else:
                return RRN.ConnectService(res[0].ConnectionURL)
         
        if required:
            msg = "[%s] Could not connect to the %s Interface after max number of (%d) attempts.\n"%(self.node_name, nodeType, attempts)
            msg += "This interface was marked as 'required'.\n"
            msg += "[%s] Shutting down."%(self.node_name)
            raise RuntimeError(msg)
        else:
            msg = "[%s] WARNING: Could not connect to the %s Interface after max number (%d) of attempts.\n"%(self.node_name, nodeType, attempts)
            msg += "This interface will not be available."
            self.log(msg)

