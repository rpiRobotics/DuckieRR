/*Duckiebot Interface Implementation*/

#include <RobotRaconteur.h>
#include "Duckiebot_Interface.h"
#include "Duckiebot_Interface_stubskel.h"
#include <boost/enable_shared_from_this.hpp>

#include <raspicam/raspicam.h> 

#pragma once

using namespace RobotRaconteur;
using namespace boost;
using namespace std;
using namespace Duckiebot_Interface;

class Camera_impl : public Duckiebot_Camera, public boost::enable_shared_from_this<Camera_impl>
{
	public:
		// Constructor / Destructor
		Camera_impl();
		void Shutdown();
		~Camera_impl();

		//properties
		virtual double get_framerate();
		virtual void set_framerate(double value);

		virtual RR_SHARED_PTR<RobotRaconteur::RRArray<int32_t > > get_resolution();
		virtual void set_resolution(RR_SHARED_PTR<RobotRaconteur::RRArray<int32_t > > value);

		virtual std::string get_format();
		virtual void set_format(std::string value);

		virtual uint8_t get_capturing();
		virtual void set_capturing(uint8_t value);

		//functions
		virtual void startCapturing();

		virtual void stopCapturing();

		virtual RR_SHARED_PTR<DuckieImage > captureImage();

		virtual void toggleFramerate();

		virtual void changeFormat(std::string format);

		virtual RR_SHARED_PTR<RobotRaconteur::Pipe<RR_SHARED_PTR<DuckieImage > > > get_ImageStream();
		virtual void set_ImageStream(RR_SHARED_PTR<RobotRaconteur::Pipe<RR_SHARED_PTR<DuckieImage > > > value);


	private:
		// objects / properties
		const string _nodeName = "camera";
		const double _framerate_high = 30.0;
		const double _framerate_low = 15.0;
		double _framerate;
		bool _update_framerate;

		const int32_t _res_w = 640;
		const int32_t _res_h = 480;
		RR_SHARED_PTR<RobotRaconteur::RRArray<int32_t > > _resolution;

		const map<string,raspicam::RASPICAM_FORMAT> _format_options = {{"gray", raspicam::RASPICAM_FORMAT_GRAY}, {"rgb",raspicam::RASPICAM_FORMAT_RGB}, {"bgr",raspicam::RASPICAM_FORMAT_BGR}};
		string _format;

		RR_SHARED_PTR<raspicam::RaspiCam> camera; //Camera object

		bool _is_shutdown;

		RR_SHARED_PTR<DuckieImage > _image;

		RR_SHARED_PTR<RobotRaconteur::Pipe<RR_SHARED_PTR<DuckieImage > > > _imagestream;
		map<uint32_t, map<int32_t, boost::shared_ptr<PipeEndpoint<boost::shared_ptr<DuckieImage> > > > > _imagestream_endpoints;

		bool _capturing;

		// helper functions
		void _capture_threadfunc();
		void _imagestream_pipeconnect(boost::shared_ptr<PipeEndpoint<boost::shared_ptr<DuckieImage> > > pipe_ep);
		void _imagestream_pipeclosed(boost::shared_ptr<PipeEndpoint<boost::shared_ptr<DuckieImage> > > pipe_ep);

};

// Global lock to protect from multi-threaded calls
extern recursive_mutex global_lock;