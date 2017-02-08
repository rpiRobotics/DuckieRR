#include "camera_impl.h"
#include <boost/thread/thread.hpp>
#include <boost/chrono.hpp>

Camera_impl::Camera_impl(void){
	
	// Initialize Camera
	camera = boost::make_shared<raspicam::RaspiCam>();
	cout << "[" << _nodeName << "] Opening Camera..."<<endl;

    _framerate = _framerate_high;
    _update_framerate = false;

    int32_t res[2] = {_res_w, _res_h};
    _resolution = AttachRRArrayCopy(res,2);
    camera->setCaptureSize(_res_w, _res_h);

    _format = "rgb";
    camera->setFormat(_format_options.at(_format));

    _is_shutdown = false;

    _image = boost::make_shared<DuckieImage>();
    _image->format = _format;
    _image->width = _res_w;
    _image->height = _res_h;

    if ( !camera->open()) throw std::runtime_error("Error opening the camera");
    _capturing = false;
    
    //wait a while until camera stabilizes
    cout<< "[" << _nodeName << "] " <<"Sleeping for 3 secs"<<endl;
    boost::this_thread::sleep(boost::posix_time::seconds(3) );
}

void Camera_impl::Shutdown(void){
	// release the camera
	camera->release();
	_is_shutdown = true;
	_capturing = false;
	cout << "[" << _nodeName << "] Shutdown."<<endl;

}

Camera_impl::~Camera_impl(void){
	if (!_is_shutdown) Shutdown();
}

/********************************
 *		Properties
 ********************************/
double Camera_impl::get_framerate(){ return _framerate;}
void Camera_impl::set_framerate(double value){throw runtime_error("Read only property");}

RR_SHARED_PTR<RobotRaconteur::RRArray<int32_t > > Camera_impl::get_resolution(){return _resolution;}
void Camera_impl::set_resolution(RR_SHARED_PTR<RobotRaconteur::RRArray<int32_t > > value){ throw runtime_error("Read only property");}

std::string Camera_impl::get_format(){return _format;}
void Camera_impl::set_format(std::string value){throw runtime_error("Read only property");}

uint8_t Camera_impl::get_capturing(){
	if (_capturing) return 1;
	else return 0;
}
void Camera_impl::set_capturing(uint8_t value){throw runtime_error("Read only property");}

/********************************
 *		Functions
 ********************************/
void Camera_impl::startCapturing(){
	if (_capturing) throw std::runtime_error("Already Capturing");
	_capturing = true;
	cout << "[" << _nodeName << "] Starting Capture" << endl;
	
	//Start a thread to capture and send frames
	thread(boost::bind(&Camera_impl::_capture_threadfunc, shared_from_this()));
}

void Camera_impl::stopCapturing(){
	if(!_capturing) throw std::runtime_error("Not Capturing");
	cout << "[" << _nodeName << "] Stopping Capture" << endl;
	_capturing = false;
}

RR_SHARED_PTR<DuckieImage > Camera_impl::captureImage(){
	recursive_mutex::scoped_lock lock(global_lock);

	// capture a frame
	camera->grab();

	// Extract and copy over the data
	_image->data = AttachRRArrayCopy((uint8_t*)camera->getImageBufferData(), camera->getImageBufferSize());

	return _image;
}

void Camera_impl::toggleFramerate(){
	if (_framerate != _framerate_high) _framerate = _framerate_high;
	else _framerate = _framerate_low;

	_update_framerate = true;
}

void Camera_impl::changeFormat(std::string format){
	if (_capturing) throw std::runtime_error("Must stop capturing to change format.");
	// release the camera so we can change the format.

	//determine if the new format is valid...
	map<string, raspicam::RASPICAM_FORMAT>::const_iterator it = _format_options.find(format);
	if (it == _format_options.end()) throw std::runtime_error("Valid formats are 'gray', 'rgb', and 'bgr'");
	
	// if it is...
	// release the camera so we can change the format
	camera->release();
	
	//change the format...
	camera->setFormat(it->second);

	//open the camera again...
	if ( !camera->open()) throw std::runtime_error("Error opening the camera");
    
    //wait a while until camera stabilizes
    cout<< "[" << _nodeName << "] " << "Sleeping for 3 secs" <<endl;
    boost::this_thread::sleep(boost::posix_time::seconds(3) );
	
	//update our other vals...
	_format = format;
	_image->format = _format;

	cout<< "[" << _nodeName << "] " << "Format is now" << _format << endl; 
	
}

/********************************
 *		Pipe
 ********************************/
RR_SHARED_PTR<RobotRaconteur::Pipe<RR_SHARED_PTR<DuckieImage > > > Camera_impl::get_ImageStream(){
	return _imagestream;
}
void Camera_impl::set_ImageStream(RR_SHARED_PTR<RobotRaconteur::Pipe<RR_SHARED_PTR<DuckieImage > > > value){
	_imagestream  = value;
	_imagestream->SetPipeConnectCallback(boost::bind(&Camera_impl::_imagestream_pipeconnect,shared_from_this(), _1));
	// _1 is a placeholder that tells boost _imagestream_pipeconnect expects 1 additional arguement
}

void Camera_impl::_imagestream_pipeconnect(boost::shared_ptr<PipeEndpoint<boost::shared_ptr<DuckieImage> > > pipe_ep){
	recursive_mutex::scoped_lock lock(global_lock);

	// Store the connected PipeEndpoint in nested maps by endpoint and index
	if (_imagestream_endpoints.count(pipe_ep->GetEndpoint())==0){
		_imagestream_endpoints.insert(make_pair(pipe_ep->GetEndpoint(), map<int32_t, boost::shared_ptr<PipeEndpoint<boost::shared_ptr<DuckieImage> > > >() ));
	}
	map<int32_t, boost::shared_ptr<PipeEndpoint<boost::shared_ptr<DuckieImage> > > >& dict1=_imagestream_endpoints.at(pipe_ep->GetEndpoint());
	dict1.insert(make_pair(pipe_ep->GetIndex(),pipe_ep));

	pipe_ep->SetPipeEndpointClosedCallback(boost::bind(&Camera_impl::_imagestream_pipeclosed,shared_from_this(),_1));
}

void Camera_impl::_imagestream_pipeclosed(boost::shared_ptr<PipeEndpoint<boost::shared_ptr<DuckieImage> > > pipe_ep){
	recursive_mutex::scoped_lock lock(global_lock);

	try{
		map<int32_t, boost::shared_ptr<PipeEndpoint<boost::shared_ptr<DuckieImage> > > >& dict1 = _imagestream_endpoints.at(pipe_ep->GetEndpoint());
		dict1.erase(pipe_ep->GetIndex()); 
	}
	catch(...){}
}


/********************************
 *		Private Functions
 ********************************/
void Camera_impl::_capture_threadfunc(){
	// Timimg for framerate
	typedef boost::chrono::duration<double> dsec;
	dsec framerate_delay = dsec(1./_framerate);
	
	while(_capturing && !_is_shutdown){
		if(_update_framerate){
			framerate_delay = dsec(1./_framerate);
		}
		auto time_limit = boost::chrono::steady_clock::now() + framerate_delay;

		// Capture a frame
		boost::shared_ptr<DuckieImage> frame = captureImage();

		recursive_mutex::scoped_lock lock(global_lock);

		// determine all connected endpoints
		vector<uint32_t> endpoints;
		for (map<uint32_t, map<int32_t,boost::shared_ptr<PipeEndpoint<boost::shared_ptr<DuckieImage> > > > >::iterator e=_imagestream_endpoints.begin(); e!=_imagestream_endpoints.end(); ++e){
			endpoints.push_back(e->first);
		}

		//loop through them and determine all indices at that endpoint
		for(vector<uint32_t>::iterator e=endpoints.begin(); e!=endpoints.end(); ++e){
			if(_imagestream_endpoints.count(*e)==0) continue;
			map<int32_t,boost::shared_ptr<PipeEndpoint<boost::shared_ptr<DuckieImage> > > >& dict1=_imagestream_endpoints.at(*e);

			// Get all of the indices at that endpoint
			std::vector<int32_t> indices;
			for(map<int32_t,boost::shared_ptr<PipeEndpoint<boost::shared_ptr<DuckieImage> > > >::iterator ee=dict1.begin(); ee!=dict1.end(); ++ee){
				indices.push_back(ee->first);
			}

			// now loop through all indices at the endpoints
			for(std::vector<int32_t>::iterator ee=indices.begin(); ee!=indices.end(); ++ee){
				if(dict1.count(*ee)==0) continue;

				uint32_t endpoint = *e;
				int32_t index = *ee;
				boost::shared_ptr<PipeEndpoint<boost::shared_ptr<DuckieImage> > > pipe_ep;
				try{
					pipe_ep=dict1.at(index);

					//send the data to the connected PipeEndpoint here
					pipe_ep->SendPacket(frame);

				}
				catch (...){
					// If there is an error sending the data to the pipe
					// assume that the pipe has been closed
					_imagestream_pipeclosed(pipe_ep);
				}
			}
		}

		boost::this_thread::sleep_until(time_limit);
	}
}


recursive_mutex global_lock;
