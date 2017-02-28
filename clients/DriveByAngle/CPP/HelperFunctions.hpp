#include <boost/enable_shared_from_this.hpp>
#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/atomic.hpp>
#include <boost/chrono.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>

using namespace std;
using namespace boost;
using namespace RobotRaconteur;
using namespace Duckiebot;

/*----------------------------
    COMMAND LINE PARSING
------------------------------*/
int findArg(string arg, int argc, char **argv){
    int idx=-1;
    for (int i=0; i<argc && idx==-1; i++)
        if (string (argv[i]) == arg) idx=i;
    return idx;
}
int getIntArgVal(string arg, int argc, char **argv, int defval=-1){
    int idx=-1;
    for (int i=0; i<argc && idx==-1; i++)
        if(string(argv[i]) == arg) idx=i;
    if (idx==-1) return defval;
    else return atoi(argv[idx+1]);
}
string getStrArgVal(string arg, int argc, char **argv, string defval=""){
    int idx=-1;
    for (int i=0; i<argc && idx==-1; i++)
        if(string(argv[i]) == arg) idx=i;
    if (idx==-1) return defval;
    else return string(argv[idx+1]);
}

void printUsage(){
    cout <<"Usage: ./DriveByAngle [-h] [--config CONFIG]\n" <<endl;
    cout <<"Optional arguments:"<<endl;
    cout <<" -h, --help\tshows this help message"<<endl;
    cout <<" --config CONFIG\tA config file for internal params (Otherwise use default.yaml)" <<endl;
    cout << endl;
}



/*----------------------------
    YAML FUNCTIONS
------------------------------*/
#ifdef HAVE_NEW_YAMLCPP
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}
#endif

/*----------------------------
    ImageConversions
------------------------------*/
cv::Mat DuckieImageToGrayMat(boost::shared_ptr<Duckiebot::Image> duckieim){
    // Initialize the frame
    cv::Mat frame(duckieim->height, duckieim->width, CV_8UC1);

    string fmt = duckieim->format;
    if (fmt != "gray"){
        throw std::runtime_error("Format is not 'gray'");
    }

    memcpy(frame.data, duckieim->data->ptr(), duckieim->data->Length());
    return frame;
}

/*----------------------------------
    Load Params and Config File
-----------------------------------*/
void getParams(const string& config)
{
    extern int nodetect_limit;
    extern double acc_min, acc_max, vel_min, vel_max;
    extern double Kp,Kd, Kp_omg;
    extern uint framerate;
    extern double ifs;
    extern cv::Mat_<double> K,D,R,P;
    extern cv::Mat map1, map2;
    extern double BinaryThresh, PolyDPLenThresh, AreaThresh, AngleThresh;
    extern uint im_w, im_h, c0, r0;

    stringstream errmsg;
    
    int rows,cols; //For matrices

    std::ifstream fin(config.c_str());
    if(!fin.good())
    {
        errmsg << "Unable to open Config file [" << config.c_str() <<"]" <<endl;
        throw std::runtime_error(errmsg.str());
    }
    
    try
    {
#ifdef HAVE_NEW_YAMLCPP
        YAML::Node doc = YAML::Load(fin);
#else
        YAML::Parser parser(fin);
        if (!parser)
        {
            cerr << "Unable to create YAML parser for config" << endl;
            return false;
        }
        YAML::Node doc;
        parser.GetNextDocument(doc);
#endif
        // Parse all of our new variables!
        doc["nodetect_limit"] >> nodetect_limit;

        doc["acc_min"] >> acc_min;

        doc["acc_max"] >> acc_max;

        doc["vel_min"] >> vel_min;

        doc["vel_max"] >> vel_max;
        
        doc["Kp"] >> Kp;

        doc["Kd"] >> Kd;

        doc["Kp_omg"] >> Kp_omg;

        doc["framerate"] >> framerate;
        ifs = 1.0/double(framerate);
        

        doc["image_width"] >> im_w;
        doc["image_height"] >> im_h;
        
        const YAML::Node& camera_matrix = doc["camera_matrix"];
        vector<double> Kdata;
        camera_matrix["rows"] >> rows;
        camera_matrix["cols"] >> cols;
        camera_matrix["data"] >> Kdata;
        K = cv::Mat_<double>(rows, cols, &Kdata[0]);
        
        const YAML::Node& distortion_coefficients = doc["distortion_coefficients"];
        vector<double> Ddata;
        distortion_coefficients["rows"] >> rows;
        distortion_coefficients["cols"] >> cols;
        distortion_coefficients["data"] >> Ddata;
        D = cv::Mat_<double>(rows, cols, &Ddata[0]);
        
        const YAML::Node& rectification_matrix = doc["rectification_matrix"];
        vector<double> Rdata;
        rectification_matrix["rows"] >> rows;
        rectification_matrix["cols"] >> cols;
        rectification_matrix["data"] >> Rdata;
        R = cv::Mat_<double>(rows, cols, &Rdata[0]);
        
        const YAML::Node& projection_matrix = doc["projection_matrix"];
        vector<double> Pdata;
        projection_matrix["rows"] >> rows;
        projection_matrix["cols"] >> cols;
        projection_matrix["data"] >> Pdata;
        P = cv::Mat_<double>(rows, cols, &Pdata[0]);
        c0 = P(0,2);
        r0 = P(1,2);

        //cout << "K : \n" << K << "\nD :\n" << D << "\nR : \n" << R << "\nP : \n" << P << endl;  
        
        //determine the rectification map for fast image correction
        cv::initUndistortRectifyMap(K,D,R,P, cv::Size(im_w,im_h), CV_16SC2, map1, map2);

        doc["BinaryThresh"] >> BinaryThresh;

        doc["PolyDPLenThresh"] >> PolyDPLenThresh;

        doc["AreaThresh"] >> AreaThresh;

        doc["AngleThresh"] >> AngleThresh;

        
    }    
    catch (YAML::Exception& e) {
        errmsg << "Exception parsing config file:\n" << e.what() << endl;
        throw std::runtime_error(errmsg.str());
    }

}


/*----------------------------------
    Detect the vehicle
-----------------------------------*/
static double angle_cos(cv::Point pt1, cv::Point pt2, cv::Point pt0){
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);

}
void detectVehicle(const cv::Mat &gray, vector<cv::Point>& detection){
    extern double BinaryThresh,PolyDPLenThresh,AreaThresh,AngleThresh;
    double AngleCosThresh = cos(AngleThresh);

    detection.clear();

    cv::Mat blurred, threshIm;
    
    // Apply Gaussian Blur to filter noise
    cv::GaussianBlur(gray, blurred, cv::Size(5,5), 0);

    // Apply thresholding
    cv::threshold(blurred, threshIm, BinaryThresh, 255, cv::THRESH_BINARY_INV);

    // Find contours in the image
    vector<vector<cv::Point> > contours;
    cv::findContours(threshIm, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    vector<cv::Point> approx;

    double maxArea = 0; // we will only return largest area
    //loop over contours
    for (size_t i = 0; i<contours.size(); i++){
        // Approximate contour with accuracy proportional to the conntour perimeter
        cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]),true)*PolyDPLenThresh, true);

        double area = cv::contourArea(cv::Mat(approx));
        if (approx.size() == 4 && 
            area > AreaThresh && 
            area > maxArea){
            maxArea = area;
            double maxCosine = 0;
            for (int j = 0; j < 5; j++){
                double cosine = fabs(angle_cos(approx[j%4], approx[j-2], approx[j-1]));
                maxCosine = MAX(maxCosine, cosine);
            }
            if (maxCosine < AngleCosThresh){
                detection = approx;
            }
        }
    }
}
cv::Point computeCenter(const vector<cv::Point>& verts){
    cv::Point cent;
    cv::Moments M = cv::moments(verts);
    if (M.m00 != 0){
        cent.x = int(M.m10/M.m00);
        cent.y = int(M.m01/M.m00);
    }
    return cent;
} 
int computeWidth(const vector<cv::Point>& verts){
    extern int im_w;
    int max_col = 0, min_col = im_w;
    for(int i = 0; i<verts.size(); i++){
        max_col = MAX(verts[i].x, max_col);
        min_col = MIN(verts[i].x, min_col);
    }
    return max_col - min_col;
    // I think cv::cvReduce(Mat(verts), &max, 0, cv::CV_REDUCE_MAX)) might also give the max...
}
/*----------------------------------
    get initial detection
-----------------------------------*/
boost::atomic<bool> keypress(false);
void keyboard_input_thread(){
    cin.get();
    keypress = true;
    return;
}
double get_initial_detection(void){
    extern boost::shared_ptr<Duckiebot::Camera::Camera> cam;
    extern cv::Mat map1,map2;
    extern int im_w;

    cout << "Initial Detection Routine..." << endl;
    cout << "When finished press <ENTER>" << endl;
    cout << "(Note that the car will start moving if wheels are enabled)" << endl;

    keypress = false;
    boost::thread t(keyboard_input_thread);
    
    vector<cv::Point> verts;
    while(!keypress){
	vector<cv::Point> temp_verts;
        cv::Mat grayRaw, grayRect;
        grayRaw = DuckieImageToGrayMat(cam->captureImage());

        // Rectify the image
        cv::remap(grayRaw, grayRect, map1, map2, cv::INTER_LINEAR);
        
        // Perform the detection
        detectVehicle(grayRect, temp_verts);

        if (temp_verts.size() > 0){
	    verts = temp_verts;
            cv::Point c = computeCenter(verts);
            cout << "Center: " << c << endl;
        }
    }

    if (verts.size() == 0){
        throw std::runtime_error("No Tag Detected");
    }

    double w = (double)computeWidth(verts);
    double f = 1; // The focal length of the camera... could actually extract but it's probably not necessary
    return 2*atan(w/(2*f)); // This should maybe be scaled / normalized...
}

/*----------------------------------
    run main loop
-----------------------------------*/
void run_main_loop(){
    extern int nodetect_limit;
    extern double alpha_d; 
    extern double acc_min, acc_max, vel_min, vel_max;
    extern double Kp, Kd, Kp_omg;
    extern uint framerate;
    extern double ifs;
    extern cv::Mat map1, map2;
    extern uint im_w, im_h, c0, r0;
    extern boost::shared_ptr<Duckiebot::Camera::Camera> cam;
    extern boost::shared_ptr<Duckiebot::Drive::Drive> drive;
    
    typedef boost::chrono::duration<double> dsec;
    dsec framerate_delay = dsec(ifs);
    
    double acc=0, vel=0, omg=0;
    uint framenum = 0;
    double avg_time = 0;
    double alpha = alpha_d;
    double alpha_prev = alpha_d;
    vector<double> alpha_dot_list(5,0);
    double cX = c0; 
    uint nodetect_count = 0;

    vector<cv::Point> verts;
    while(true){
        auto tic = boost::chrono::steady_clock::now();
        auto time_limit = tic + framerate_delay;

        cv::Mat grayRaw, grayRect;
        grayRaw = DuckieImageToGrayMat(cam->captureImage());

        // Rectify the image
        cv::remap(grayRaw, grayRect, map1, map2, cv::INTER_LINEAR);
        
        // Perform the detection
        detectVehicle(grayRect, verts);

        if (verts.size() > 0){
            double w = (double)computeWidth(verts);
            cX = computeCenter(verts).x;
            double f = 1; // The focal length of the camera... could actually extract but it's probably not necessary
            alpha = 2*atan(w/(2*f));
        }
        else{
            if (nodetect_count < nodetect_limit){
                alpha = alpha_d;
                cout << "no tag " << nodetect_count << endl;
                nodetect_count++;
            }
            else{
                cout << "WARNING: No tag detected for " << nodetect_limit << " frames. Stopping." << endl;
                break;
            }
        }

        //compute the derivative
        double alpha_dot = (alpha - alpha_prev)/ifs;
        // keep the last 5 values
        alpha_dot_list[framenum%5] = alpha_dot;

        double alpha_dot_avg;
        if (framenum < 4){
            alpha_dot_avg = alpha_dot;
        }
        else{
            // implement a moving average
            alpha_dot_avg = cv::mean(cv::Mat(alpha_dot_list))[0];
        }

        // save the last value
        alpha_prev = alpha;

        // determine the velocity controller action
        double ang_err = (1.0/alpha) - (1.0/alpha_d);
        acc = Kp*ang_err + Kd*alpha_dot_avg;
        acc = MAX(MIN(acc,acc_max), acc_min);

        vel += acc*ifs;
        vel = MAX(MIN(vel, vel_max), vel_min);

        // determine the steering controller accuracy
        double center_err = (double(c0)-cX)/double(c0);
        omg = Kp_omg*center_err;

        // Send the full command to the car
        drive->carCmd(vel,omg);

        // increment the frame number
        framenum++;

        //Sleep for the remainder of the time...
        boost::this_thread::sleep_until(time_limit);
        
        //check how long we have been running
        auto toc = boost::chrono::steady_clock::now() - tic;

        double toc_sec = toc.count();
        avg_time = ((framenum-1)*avg_time/framenum) + (toc_sec/framenum);
    }

    cout << "Average Loop Freq: " << 1.0/avg_time << endl;
}


