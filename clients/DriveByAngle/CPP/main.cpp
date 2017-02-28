#include <RobotRaconteur.h>
#include "Duckiebot.h"
#include "Duckiebot_stubskel.h"
#include "Duckiebot__Camera.h"
#include "Duckiebot__Camera_stubskel.h"
#include "Duckiebot__Drive.h"
#include "Duckiebot__Drive_stubskel.h"
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>
#include <fstream>
#include <sstream>
#include "HelperFunctions.hpp"

using namespace std;
using namespace boost;
using namespace RobotRaconteur;
using namespace Duckiebot;

/*---------------------------
    Globals (params)
----------------------------*/
int nodetect_limit = 15;
double acc_min = -1.0;
double acc_max = 1.0;
double vel_min = -0.5;
double vel_max = 1.0;
double alpha_d = 0.0;
double Kp = 1.0;
double Kd = -2.0;
double Kp_omg = 0.95;

uint framerate = 15;
double ifs = 1.0/double(framerate);

cv::Mat_<double> K,D,R,P;
cv::Mat map1,map2;

double BinaryThresh = 80;
double PolyDPLenThresh = 0.04;
double AreaThresh = 250;
double AngleThresh = 1.3962634015954636;

uint im_w = 0;
uint im_h = 0;
uint c0 = 0;
uint r0 = 0;

boost::shared_ptr<Duckiebot::Camera::Camera> cam;
boost::shared_ptr<Duckiebot::Drive::Drive> drive;

/*-------------------------
    Signal Handling
---------------------------*/
void signalHandler(int signum){
    try{
    cerr << "Checking that RR is shutdown" << endl;
        RobotRaconteurNode::s()->Shutdown();
    }
    catch (std::exception& e){ cerr << e.what() << endl; }
    catch (...){ cerr << "Another Exception Occurred" << endl; }
    exit(-1);
}

/*-------------------------
    Main
---------------------------*/
int main ( int argc,char **argv ) {

    //Parse arguments
    if (findArg("--help", argc, argv) !=-1 || findArg("-h",argc,argv)!=-1){
        printUsage();
        return 0;
    }
    
    string config = getStrArgVal("--config", argc, argv, "default.yaml");

    // Register Signal Handler to catch interrupts / exit
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    signal(SIGABRT, signalHandler);

    /*
    cv::Point pt0(0,0), pt1(2,0), pt2(2,4), pt3(0,4);
    vector<cv::Point> rectangle = {pt0, pt1, pt2, pt3};
    cout << computeCenter(rectangle) << endl;
    
    vector<double> test = {1,2,3,4};
    cout << cv::mean(cv::Mat(test))[0] << endl;
    return 0;
    */
    try{
        // Get the parameters.
        getParams(config);

        //Register A Local Transport
        boost::shared_ptr<LocalTransport> t1 = boost::make_shared<LocalTransport>();
        RobotRaconteurNode::s()->RegisterTransport(t1);
        /*
        //Create TCP transport, register it
        boost::shared_ptr<TcpTransport> t2 = boost::make_shared<TcpTransport>();
        t2->EnableNodeDiscoveryListening();
        RobotRaconteurNode::s()->RegisterTransport(t2);
        */

        // Register the service types
        RobotRaconteurNode::s()->RegisterServiceType(boost::make_shared<Duckiebot::DuckiebotFactory>());
        RobotRaconteurNode::s()->RegisterServiceType(boost::make_shared<Duckiebot::Camera::Duckiebot__CameraFactory>());
        RobotRaconteurNode::s()->RegisterServiceType(boost::make_shared<Duckiebot::Drive::Duckiebot__DriveFactory>());

        // Wait ~10 seconds to revieve the autodiscovery packets
        cout << "Waiting ~10 seconds to connect..." << endl;
        boost::this_thread::sleep(boost::posix_time::milliseconds(10000));

        // Search for Camera and Drive on Local Transport
        vector<string> schemes = {"rr+local"};
        vector<ServiceInfo2> cam_res = RobotRaconteurNode::s()->FindServiceByType("Duckiebot.Camera.Camera", schemes);
        vector<ServiceInfo2> drive_res = RobotRaconteurNode::s()->FindServiceByType("Duckiebot.Drive.Drive", schemes);

        if(cam_res.size() == 0){
            cout << "Could not find the camera service node." << endl;
            return -1;
        }
        if(drive_res.size() == 0){
            cout << "Could not find the drive service node." << endl;
            return -1;
        }

        //Connect to the services
        cam = rr_cast<Duckiebot::Camera::Camera>(RobotRaconteurNode::s()->ConnectService(cam_res[0].ConnectionURL,"",NULL,NULL,"Duckiebot.Camera.Camera"));
        drive = rr_cast<Duckiebot::Drive::Drive>(RobotRaconteurNode::s()->ConnectService(drive_res[0].ConnectionURL,"",NULL,NULL,"Duckiebot.Drive.Drive"));
       
        if (cam->get_format() != "gray")
            cam->changeFormat("gray");

        // get and set some drive values
        vel_max = drive->get_limit();
        drive->set_trim(-0.02);
        
        // Get the initial detection
        alpha_d = get_initial_detection();

        // run the main loop
        run_main_loop();

        // once the loop stops stop the wheels
        drive->carCmd(0,0);

    }
    catch(std::exception& exp)
    {
        cout << "Error occured: " << exp.what() << endl;
        return -1;
    }
        
    //This must be here to prevent segfault
    RobotRaconteurNode::s()->Shutdown();
    
    return 0;
}