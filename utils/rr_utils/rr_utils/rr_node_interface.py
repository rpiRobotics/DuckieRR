from abc import ABCMeta, abstractmethod

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
