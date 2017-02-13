from abc import ABCMeta, abstractmethod

class RRNodeInterface():
	__metaclass__ = ABCMeta

	@abstractmethod
	def onShutdown(self):
		pass