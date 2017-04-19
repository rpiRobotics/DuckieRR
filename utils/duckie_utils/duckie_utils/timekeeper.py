import time

def asms(s):
    if s is None:
        return 'n/a'
    return "%.1fms" % (s*1000)
        

class TimeKeeper():
    def __init__(self, header=None):
        self.latencies = []
        if header != None:
            self.t_acquisition = header.time
            self.c_acquisition = header.ctime
            self.latencies.append(('captured',
                dict(t=self.t_acquisition, c=self.c_acquisition,
                    delta_wall_ms=None, delta_clock_ms=None, 
                    latency_ms=asms(0))))
        else:
            self.t_acquisition = time.time()
            self.c_acquisition = time.clock()
        

        self.completed('acquired')

    def completed(self, phase):
        t = time.time() 
        c = time.clock()
        latency = t - self.t_acquisition
        latency_ms = asms(latency)
        
        if self.latencies:
            last_t = self.latencies[-1][1]['t']
            delta_wall_ms = asms(t-last_t)

            last_c = self.latencies[-1][1]['c']
            delta_clock_ms = asms(c - last_c)
        else:
            delta_wall_ms = None
            delta_clock_ms = None

        self.latencies.append((phase, 
            dict(t=t, c=c, delta_wall_ms=delta_wall_ms, delta_clock_ms=delta_clock_ms,
             latency_ms=latency_ms)))
    
    def getall(self):
        s = "\nLatencies:\n"

        for phase, data in self.latencies:
            s +=  ' %22s | total latency %10s | delta wall %8s clock %8s\n' % (phase, 
             data['latency_ms'], data['delta_wall_ms'], data['delta_clock_ms'])

        return s
