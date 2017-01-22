import numpy as np
import time

class Stats():
	def __init__(self):
		self.nresets = 0
		self.reset()

	def reset(self):
		self.nresets += 1
		self.t0 = time.time()
		self.nreceived = 0
		self.nskipped = 0
		self.nprocessed = 0

	def received(self):
		self.nreceived += 1

	def skipped(self):
		self.nskipped += 1

	def processed(self):
		self.nprocessed += 1

	def info(self):
		delta = time.time() - self.t0

		if self.nreceived:
			skipped_perc = (100 * self.nskipped/self.nreceived)
		else:
			skipped_perc = 0

		def fps(x):
			return '%.1f fps'%(x / delta)

		m = ('In the last %.1f s: received %d (%s) processed %d (%s) skipped %d (%s) (%1.f%%)' %
			(delta, self.nreceived, fps(self.nreceived),
				self.nprocessed, fps(self.nprocessed),
				self.nskipped, fps(self.nskipped), skipped_perc) )
		return m