
/**
*/
#include "camera_impl.h" // This imports RR as well
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <signal.h>

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
    RobotRaconteurNode::s()->Shutdown();
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
    t1->StartServerAsNodeName("DuckiebotServer.Camera");
    RobotRaconteurNode::s()->RegisterTransport(t1);

    //Create TCP transport, register it, and start the service
    boost::shared_ptr<TcpTransport> t2 = boost::make_shared<TcpTransport>();
    t2->EnableNodeAnnounce(IPNodeDiscoveryFlags_NODE_LOCAL | 
                           IPNodeDiscoveryFlags_LINK_LOCAL |
                           IPNodeDiscoveryFlags_SITE_LOCAL);

    RobotRaconteurNode::s()->RegisterTransport(t2);

    t2->StartServer(port);
    if(port == 0) port = t2->GetListenPort();

    // Register the service def
    RobotRaconteurNode::s()->RegisterServiceType(boost::make_shared<Duckiebot_InterfaceFactory>());

    //Initialize the implementation object
    boost::shared_ptr<Camera_impl> camera_obj = boost::make_shared<Camera_impl>();

    //Register the service
    RobotRaconteurNode::s()->RegisterService("Duckiebot_Camera","Duckiebot_Interface.Duckiebot_Camera",camera_obj);


    // Determine hostname
    char hn[20]; 
    string hostname;
    string ip;
    int have_name = gethostname(hn, 20);
    if (have_name){
        hostname = string(hn);
        // do some more stuff for ip...
        ip = "<IP_ADDRESS>";
    }
    else{
        hostname = "<hostname>";
        ip = "<IP_ADDRESS>";
    }
    // Print connection messages
    cout << "Service started, connect via one of the following:" << endl;
    cout << "rr+local:///?nodename=DuckiebotServer.Camera&service=Duckiebot_Camera" << endl;
    cout << "rr+tcp://localhost:" << port <<"/?service=Duckiebot_Camera" << endl;
    cout << "rr+tcp://localhost:" << port << "/?nodename=DuckiebotServer.Camera&service=Duckiebot_Camera" << endl;
    cout << "rr+tcp://" << hostname << ".local:" << port << "/?nodename=DuckiebotServer.Camera&service=Duckiebot_Camera" << endl;
    cout << "rr+tcp://" << ip << ":" << port << "/?nodename=DuckiebotServer.Camera&service=Duckiebot_Camera" << endl;
    
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
//
