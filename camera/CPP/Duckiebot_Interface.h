//This file is automatically generated. DO NOT EDIT!

#include <RobotRaconteur.h>
#include <boost/signals2.hpp>
#pragma once

namespace Duckiebot_Interface
{

class DuckieImage;
class Camera;

class DuckieImage : public RobotRaconteur::RRStructure {
public:
std::string format;
int32_t width;
int32_t height;
RR_SHARED_PTR<RobotRaconteur::RRArray<uint8_t > > data;

virtual std::string RRType() {return "Duckiebot_Interface.DuckieImage";  }
};

class Camera : public virtual RobotRaconteur::RRObject
{
public:
virtual double get_framerate()=0;
virtual void set_framerate(double value)=0;

virtual RR_SHARED_PTR<RobotRaconteur::RRArray<int32_t > > get_resolution()=0;
virtual void set_resolution(RR_SHARED_PTR<RobotRaconteur::RRArray<int32_t > > value)=0;

virtual std::string get_format()=0;
virtual void set_format(std::string value)=0;

virtual uint8_t get_capturing()=0;
virtual void set_capturing(uint8_t value)=0;

virtual void startCapturing()=0;

virtual void stopCapturing()=0;

virtual RR_SHARED_PTR<DuckieImage > captureImage()=0;

virtual void toggleFramerate()=0;

virtual void changeFormat(std::string format)=0;

virtual RR_SHARED_PTR<RobotRaconteur::Pipe<RR_SHARED_PTR<DuckieImage > > > get_ImageStream()=0;
virtual void set_ImageStream(RR_SHARED_PTR<RobotRaconteur::Pipe<RR_SHARED_PTR<DuckieImage > > > value)=0;

virtual std::string RRType() {return "Duckiebot_Interface.Camera";  }
};

}

