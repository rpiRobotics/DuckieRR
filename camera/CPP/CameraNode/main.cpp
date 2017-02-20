
/**
*/
#include "CameraNode.h" // This imports RR as well
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <signal.h>
#include <exception>

using namespace Duckiebot;
using namespace RobotRaconteur;
using namespace std;

int findArg(string arg, int argc, char **argv){
    int idx=-1;
    for (int i=0; i<argc && idx==-1; i++)
        if (string (argv[i]) == arg) idx=i;
    return idx;
}
int getArgVal(string arg, int argc, char **argv, int defval=-1){
    int idx=-1;
    for (int i=0; i<argc && idx==-1; i++)
        if(string(argv[i]) == arg) idx=i;
    if (idx==-1) return defval;
    else return atoi(argv[idx+1]);
}

void printUsage(){
    cout <<"Usage: ./camera_interface [-h] [--port PORT]\n" <<endl;
    cout <<"Optional arguments:"<<endl;
    cout <<" -h, --help\tshows this help message"<<endl;
    cout <<" --port PORT\tsets the TCP port to host the service on (will auto-generate if not specified)" <<endl;
    cout << endl;
}

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

int main ( int argc,char **argv ) {
    //Parse arguments
    if (findArg("--help", argc, argv) !=-1 || findArg("-h",argc,argv)!=-1){
        printUsage();
        return 0;
    }
    
    int port = getArgVal("--port", argc, argv, 0);

    // Register Abort signal handler, just incase anything goes wrong...
    signal(SIGABRT, abortSignalHandler);
   
    //Initialize the local transport
    boost::shared_ptr<LocalTransport> t1 = boost::make_shared<LocalTransport>();
    t1->StartServerAsNodeName("Duckiebot.Camera");
    RobotRaconteurNode::s()->RegisterTransport(t1);

    //Create TCP transport, register it, and start the service
    boost::shared_ptr<TcpTransport> t2 = boost::make_shared<TcpTransport>();
    t2->EnableNodeAnnounce(IPNodeDiscoveryFlags_NODE_LOCAL | 
                           IPNodeDiscoveryFlags_LINK_LOCAL |
                           IPNodeDiscoveryFlags_SITE_LOCAL);

    RobotRaconteurNode::s()->RegisterTransport(t2);

    t2->StartServer(port);
    if(port == 0) port = t2->GetListenPort();

    // Register the service defs
    RobotRaconteurNode::s()->RegisterServiceType(boost::make_shared<DuckiebotFactory>());
    RobotRaconteurNode::s()->RegisterServiceType(boost::make_shared<Camera::Duckiebot__CameraFactory>());

    //Initialize the implementation object
    boost::shared_ptr<CameraNode> camera_obj = boost::make_shared<CameraNode>();

    //Register the service
    RobotRaconteurNode::s()->RegisterService("Camera","Duckiebot.Camera",camera_obj);

    // Determine hostname
    char hn[20]; 
    string hostname;
    string ip;
    int have_name = gethostname(hn, 20);
    if (have_name !=-1){
        hostname = string(hn);
        // TODO: figure out how to get the IP address
        ip = "<IP_ADDRESS>";
    }
    else{
        hostname = "<hostname>";
        ip = "<IP_ADDRESS>";
    }
    // Print connection messages
    cout << "Service started, connect via one of the following:" << endl;
    cout << "rr+local:///?nodename=Duckiebot.Camera&service=Camera" << endl;
    cout << "rr+tcp://localhost:" << port <<"/?service=Camera" << endl;
    cout << "rr+tcp://localhost:" << port << "/?nodename=Duckiebot.Camera&service=Camera" << endl;
    cout << "rr+tcp://" << hostname << ".local:" << port << "/?nodename=Duckiebot.Camera&service=Camera" << endl;
    cout << "rr+tcp://" << ip << ":" << port << "/?nodename=Duckiebot.Camera&service=Camera" << endl;
    
    // Register Signal Handler to catch interrupts / exit
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    // Loop until that happens...
    while (!sig_caught){ }

    camera_obj->Shutdown();
    
    //This must be here to prevent segfault
    RobotRaconteurNode::s()->Shutdown();
    return 0;
}
