// Compile with 'gcc -o laneControl-RT laneControl-RT.cpp -lrt'
#include <stdlib.h>
#include <stdio.h>
#include <sys/mman.h>	// Needed for mlockall()
#include <unistd.h>		// needed for sysconf(int name);
#include <malloc.h>
#include <sys/time.h>	// needed for getrusage
#include <sys/resource.h>	// needed for getrusage
#include <limits.h>
#include <signal.h>
#include <math.h>
#include <RobotRaconteur.h>
#include "Duckiebot__LaneInfo.h"
#include "Duckiebot__LaneInfo_stubskel.h"
#include "Duckiebot__Drive.h"
#include "Duckiebot__Drive_stubskel.h"
#include <boost/enable_shared_from_this.hpp>

#define MY_PRIORITY (49) /* we use 49 as the PREEMPT_RT uses 50 as the priority
                            of kernel tasks and interrupt handlers by default */

#define PRE_ALLOCATION_SIZE (100*1024*1024) /* 100MB pagefault free buffer */
#define MY_STACK_SIZE       (100*1024)      /* 100 kB is enough for now. */
#define NSEC_PER_SEC    (1000000000) /* The number of nsecs per sec. */

using namespace Duckiebot;
using namespace RobotRaconteur;
using namespace std;

static volatile sig_atomic_t sig_caught = 0;
void signalHandler(int signum){
    sig_caught = 1;
}
void abortSignalHandler(int signum){
   cerr << "Caught Abort Signal, handling it before aborting..." << endl;
    // just make sure we shutdown RR.
   try{
      cerr << "Checking that RR is shutdown" << endl;
      RobotRaconteurNode::s()->Shutdown();
    }
    catch (std::exception& e){ cerr << e.what() << endl; }
    catch (...){ cerr << "Another Exception Occurred" << endl; }
    sig_caught = 1;
    //raise(SIGTERM);
    //exit(signum);
}

void stack_prefault(void)
{
   unsigned char dummy[MY_STACK_SIZE];
   memset(dummy,0,MY_STACK_SIZE);
   return;
}

int main(int argc, char *argv[])
{
	struct timespec t;
   struct sched_param param;
   int interval = 1000000; // 1ms 

   // Declare ourself as a RT task
   param.sched_priority = MY_PRIORITY;
   if (sched_setscheduler(0, SCHED_FIFO, &param) == -1){
      cerr << "sched_setscheduler failed" << endl;
      return -1;
   }

   // Lock memory
   if(mlockall(MCL_CURRENT | MCL_FUTURE) == -1){
      cerr << "mlockall failed" << endl;
      return -2;
   }

   // Pre-fault our stack
   stack_prefault();

   signal(SIGABRT, abortSignalHandler);

   // Initialize a transport
   boost::shared_ptr<LocalTransport> t1 = boost::make_shared<LocalTransport>();
   RobotRaconteurNode::s()->RegisterTransport(t1);

   // Register the service types
   RobotRaconteurNode::s()->RegisterServiceType(boost::make_shared<DuckiebotFactory>());
   RobotRaconteurNode::s()->RegisterServiceType(boost::make_shared<LaneInfo::Duckiebot__LaneInfoFactory>());
   RobotRaconteurNode::s()->RegisterServiceType(boost::make_shared<Drive::Duckiebot__DriveFactory>());

   // Connect to the service
   boost::this_thread::sleep(boost::posix_time::milliseconds(6000));
   vector<string> schemes = {"rr+local"};
   vector<ServiceInfo2> laneInfoRes = RobotRaconteurNode::s()->FindServiceByType("Duckiebot.LaneInfo.LaneInfo",schemes);
   vector<ServiceInfo2> driveRes = RobotRaconteurNode::s()->FindServiceByType("Duckiebot.Drive.Drive",schemes);

   if (laneInfoRes.size() == 0)
   {
      cout << "Could not find the laneInfo service node" << endl;
      return -1;
   }
   if (driveRes.size() == 0)
   {
      cout << "Could not find the drive service node" << endl;
      return -1;
   }
   boost::shared_ptr<Drive::Drive> drive = rr_cast<Drive::Drive>(RobotRaconteurNode::s()->ConnectService(driveRes[0].ConnectionURL, "", boost::shared_ptr<RRMap<std::string,RRObject> >(),NULL,"Duckiebot.Drive.Drive"));
   boost::shared_ptr<LaneInfo::LaneInfo> laneInfo = rr_cast<LaneInfo::LaneInfo>(RobotRaconteurNode::s()->ConnectService(laneInfoRes[0].ConnectionURL,"", boost::shared_ptr<RRMap<std::string,RRObject> >(),NULL, "Duckiebot.LaneInfo.LaneInfo"));

   // Set the gains
   const double v_bar = 0.5;
   const double k_theta = -2.0;
   const double k_d = -(pow(k_theta,2))/(4.0 * v_bar);
   const double theta_thres = M_PI/6;
   const double d_thres = fabs(k_theta / k_d) * theta_thres;
   const double d_offset = 0.0;

   signal(SIGINT, signalHandler);
   signal(SIGTERM, signalHandler);

   // Get the current time and add 1 sec (i.e. we will wait one sec. before starting..)
   clock_gettime(CLOCK_MONOTONIC, &t);
   t.tv_sec++;

   while(!sig_caught){
      // wait until next interval
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

      // Do whatever else
      boost::shared_ptr<LaneInfo::LanePose> pose = laneInfo->get_lanePose();
      double cross_track_err = pose->d - d_offset;
      double heading_err = pose->phi;

      if (fabs(cross_track_err) > d_thres) {
         cross_track_err = cross_track_err / fabs(cross_track_err) * d_thres;
      }

      double omega = k_d * cross_track_err + k_theta * heading_err;
      drive->carCmd(v_bar, omega);
      
      // We will wait for 1ms...
      t.tv_nsec += interval;
      // Account for seconds overflow
      while(t.tv_nsec >= NSEC_PER_SEC) {
         t.tv_nsec -= NSEC_PER_SEC;
         t.tv_sec++;
      }

   }

   drive->carCmd(0,0);
   RobotRaconteurNode::s()->Shutdown();
	return 0;
}
