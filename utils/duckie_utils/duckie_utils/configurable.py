import yaml
import numpy as np

class Configurable():
	"""Create a reconfigurable instance of a class.
	Load specified parameter names from yaml file."""
	def __init__(self, param_names,configuration):
		if not isinstance(configuration, dict):
			msg = 'Expecting a dict, obtained %r' %configuration
			raise ValueError(msg)

		# check that all parameters are set
		given = list(configuration)
		required = list(param_names)

		extra = set(given) - set(required)
		missing = set(required) - set(given)
		if extra or missing:
			msg = """Error while loading configuration for %r from %r.
			Extra parameters: %r
			Missing parameters: %r"""%(self,configuration,extra,missing)
			raise ValueError(msg)

		assert set(given) == set(required)
		for p in param_names:
			value = configuration[p]
			# if the list is 3 numbers, conver to an array
			if isinstance(value, list) and len(value) == 3:
				value=np.array(value)
			configuration[p] = value

		for p in param_names:
			setattr(self, p, configuration[p])

		return configuration
		
