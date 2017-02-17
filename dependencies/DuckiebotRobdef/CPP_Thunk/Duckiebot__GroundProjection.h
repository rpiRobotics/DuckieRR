//This file is automatically generated. DO NOT EDIT!

#include <RobotRaconteur.h>
#include <boost/signals2.hpp>
#include "Duckiebot.h"
#pragma once

namespace Duckiebot
{
namespace GroundProjection
{

class GroundProjection;

class GroundProjection : public virtual RobotRaconteur::RRObject
{
public:
virtual RR_SHARED_PTR<RobotRaconteur::RRList<Duckiebot::Segment  > > get_segments()=0;
virtual void set_segments(RR_SHARED_PTR<RobotRaconteur::RRList<Duckiebot::Segment  > > value)=0;

virtual RR_SHARED_PTR<RobotRaconteur::RRArray<double > > estimate_homography(RR_SHARED_PTR<Duckiebot::Image > im)=0;

virtual RR_SHARED_PTR<Duckiebot::Point > ground_coordinate(RR_SHARED_PTR<Duckiebot::Pixel > uv, RR_SHARED_PTR<Duckiebot::Vector2D > normalized_uv)=0;

virtual RR_SHARED_PTR<Duckiebot::Vector2D > image_coordinate(RR_SHARED_PTR<Duckiebot::Point > gp)=0;

virtual boost::signals2::signal<void (RR_SHARED_PTR<RobotRaconteur::RRList<Duckiebot::Segment  > >)>& get_newSegments()=0;

virtual std::string RRType() {return "Duckiebot.GroundProjection.GroundProjection";  }
};

}
}

