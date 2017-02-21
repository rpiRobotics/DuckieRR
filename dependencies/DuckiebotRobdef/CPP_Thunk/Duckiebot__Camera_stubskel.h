//This file is automatically generated. DO NOT EDIT!

#include "Duckiebot__Camera.h"
#include "Duckiebot_stubskel.h"
#pragma once

namespace Duckiebot
{
namespace Camera
{

class Duckiebot__CameraFactory : public virtual RobotRaconteur::ServiceFactory
{
public:
virtual std::string GetServiceName();
virtual std::string DefString();
virtual RR_SHARED_PTR<RobotRaconteur::StructureStub> FindStructureStub(std::string s);
virtual RR_SHARED_PTR<RobotRaconteur::MessageElementStructure> PackStructure(RR_SHARED_PTR<RobotRaconteur::RRStructure> structin);
virtual RR_SHARED_PTR<RobotRaconteur::RRObject> UnpackStructure(RR_SHARED_PTR<RobotRaconteur::MessageElementStructure> mstructin);
virtual RR_SHARED_PTR<RobotRaconteur::ServiceStub> CreateStub(std::string objecttype, std::string path, RR_SHARED_PTR<RobotRaconteur::ClientContext> context);
virtual RR_SHARED_PTR<RobotRaconteur::ServiceSkel> CreateSkel(std::string objecttype, std::string path, RR_SHARED_PTR<RobotRaconteur::RRObject> obj, RR_SHARED_PTR<RobotRaconteur::ServerContext> context);
virtual void DownCastAndThrowException(RobotRaconteur::RobotRaconteurException& exp);
virtual RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> DownCastException(RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> exp);
};

class async_Camera
{
public:
virtual void async_get_framerate(boost::function<void (double,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;
virtual void async_set_framerate(double value,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
virtual double async_get_framerate(boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    return RobotRaconteur::detail::async_wrap_for_spawn<double >(boost::bind((void (async_Camera::*)(boost::function<void (double, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) >, int32_t))&async_Camera::async_get_framerate, this, _1, rr_timeout), rr_yield);
}
virtual void async_set_framerate(double value, boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    RobotRaconteur::detail::async_wrap_for_spawn_void(boost::bind((void (async_Camera::*)(double, boost::function<void(RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>, int32_t))&async_Camera::async_set_framerate, this, boost::ref(value), _1, rr_timeout), rr_yield);
}
#endif

virtual void async_get_resolution(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RRArray<int32_t > >,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;
virtual void async_set_resolution(RR_SHARED_PTR<RobotRaconteur::RRArray<int32_t > > value,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
virtual RR_SHARED_PTR<RobotRaconteur::RRArray<int32_t > > async_get_resolution(boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    return RobotRaconteur::detail::async_wrap_for_spawn<RR_SHARED_PTR<RobotRaconteur::RRArray<int32_t > > >(boost::bind((void (async_Camera::*)(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RRArray<int32_t > >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) >, int32_t))&async_Camera::async_get_resolution, this, _1, rr_timeout), rr_yield);
}
virtual void async_set_resolution(RR_SHARED_PTR<RobotRaconteur::RRArray<int32_t > > value, boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    RobotRaconteur::detail::async_wrap_for_spawn_void(boost::bind((void (async_Camera::*)(RR_SHARED_PTR<RobotRaconteur::RRArray<int32_t > >, boost::function<void(RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>, int32_t))&async_Camera::async_set_resolution, this, boost::ref(value), _1, rr_timeout), rr_yield);
}
#endif

virtual void async_get_format(boost::function<void (std::string,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;
virtual void async_set_format(std::string value,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
virtual std::string async_get_format(boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    return RobotRaconteur::detail::async_wrap_for_spawn<std::string >(boost::bind((void (async_Camera::*)(boost::function<void (std::string, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) >, int32_t))&async_Camera::async_get_format, this, _1, rr_timeout), rr_yield);
}
virtual void async_set_format(std::string value, boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    RobotRaconteur::detail::async_wrap_for_spawn_void(boost::bind((void (async_Camera::*)(std::string, boost::function<void(RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>, int32_t))&async_Camera::async_set_format, this, boost::ref(value), _1, rr_timeout), rr_yield);
}
#endif

virtual void async_get_capturing(boost::function<void (uint8_t,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;
virtual void async_set_capturing(uint8_t value,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
virtual uint8_t async_get_capturing(boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    return RobotRaconteur::detail::async_wrap_for_spawn<uint8_t >(boost::bind((void (async_Camera::*)(boost::function<void (uint8_t, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) >, int32_t))&async_Camera::async_get_capturing, this, _1, rr_timeout), rr_yield);
}
virtual void async_set_capturing(uint8_t value, boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    RobotRaconteur::detail::async_wrap_for_spawn_void(boost::bind((void (async_Camera::*)(uint8_t, boost::function<void(RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>, int32_t))&async_Camera::async_set_capturing, this, boost::ref(value), _1, rr_timeout), rr_yield);
}
#endif

virtual void async_startCapturing(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
virtual void async_startCapturing(boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    RobotRaconteur::detail::async_wrap_for_spawn_void(boost::bind((void (async_Camera::*)(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>,int32_t))&async_Camera::async_startCapturing, this, _1,rr_timeout), rr_yield);
}
#endif

virtual void async_stopCapturing(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
virtual void async_stopCapturing(boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    RobotRaconteur::detail::async_wrap_for_spawn_void(boost::bind((void (async_Camera::*)(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>,int32_t))&async_Camera::async_stopCapturing, this, _1,rr_timeout), rr_yield);
}
#endif

virtual void async_captureImage(boost::function<void (RR_SHARED_PTR<Duckiebot::Image >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
RR_SHARED_PTR<Duckiebot::Image > async_captureImage(boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    return RobotRaconteur::detail::async_wrap_for_spawn<RR_SHARED_PTR<Duckiebot::Image >>(boost::bind((void (async_Camera::*)(boost::function<void (RR_SHARED_PTR<Duckiebot::Image >,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>,int32_t))&async_Camera::async_captureImage, this, _1,rr_timeout), rr_yield);
}
#endif

virtual void async_toggleFramerate(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
virtual void async_toggleFramerate(boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    RobotRaconteur::detail::async_wrap_for_spawn_void(boost::bind((void (async_Camera::*)(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>,int32_t))&async_Camera::async_toggleFramerate, this, _1,rr_timeout), rr_yield);
}
#endif

virtual void async_changeFormat(std::string format,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
virtual void async_changeFormat(std::string format,boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    RobotRaconteur::detail::async_wrap_for_spawn_void(boost::bind((void (async_Camera::*)(std::string,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>,int32_t))&async_Camera::async_changeFormat, this, boost::ref(format),_1,rr_timeout), rr_yield);
}
#endif

};
class Camera_stub : public virtual Camera, public virtual async_Camera, public virtual RobotRaconteur::ServiceStub
{
public:
Camera_stub(const std::string& path, RR_SHARED_PTR<RobotRaconteur::ClientContext> c);

virtual void RRInitStub();
virtual double get_framerate();
virtual void set_framerate(double value);

virtual RR_SHARED_PTR<RobotRaconteur::RRArray<int32_t > > get_resolution();
virtual void set_resolution(RR_SHARED_PTR<RobotRaconteur::RRArray<int32_t > > value);

virtual std::string get_format();
virtual void set_format(std::string value);

virtual uint8_t get_capturing();
virtual void set_capturing(uint8_t value);

virtual void startCapturing();

virtual void stopCapturing();

virtual RR_SHARED_PTR<Duckiebot::Image > captureImage();

virtual void toggleFramerate();

virtual void changeFormat(std::string format);

virtual RR_SHARED_PTR<RobotRaconteur::Pipe<RR_SHARED_PTR<Duckiebot::Image > > > get_ImageStream();
virtual void set_ImageStream(RR_SHARED_PTR<RobotRaconteur::Pipe<RR_SHARED_PTR<Duckiebot::Image > > > value);


virtual void DispatchEvent(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m);
virtual void DispatchPipeMessage(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m);
virtual void DispatchWireMessage(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m);
virtual RR_SHARED_PTR<RobotRaconteur::MessageEntry> CallbackCall(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m);
virtual void RRClose();
private:
RR_SHARED_PTR<RobotRaconteur::PipeClient<RR_SHARED_PTR<Duckiebot::Image > > > rrvar_ImageStream;
virtual void async_get_framerate(boost::function<void (double,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);
virtual void async_set_framerate(double value,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_get_framerate(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (double ,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
virtual void rrend_set_framerate(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual void async_get_resolution(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RRArray<int32_t > >,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);
virtual void async_set_resolution(RR_SHARED_PTR<RobotRaconteur::RRArray<int32_t > > value,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_get_resolution(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RRArray<int32_t > > ,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
virtual void rrend_set_resolution(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual void async_get_format(boost::function<void (std::string,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);
virtual void async_set_format(std::string value,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_get_format(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (std::string ,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
virtual void rrend_set_format(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual void async_get_capturing(boost::function<void (uint8_t,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);
virtual void async_set_capturing(uint8_t value,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_get_capturing(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (uint8_t ,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
virtual void rrend_set_capturing(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual void async_startCapturing(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_startCapturing(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual void async_stopCapturing(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_stopCapturing(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual void async_captureImage(boost::function<void (RR_SHARED_PTR<Duckiebot::Image >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_captureImage(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<Duckiebot::Image > ,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual void async_toggleFramerate(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_toggleFramerate(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual void async_changeFormat(std::string format,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_changeFormat(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual std::string RRType();
};


class Camera_skel : public virtual RobotRaconteur::ServiceSkel
{
public:
virtual void Init(const std::string& path, RR_SHARED_PTR<RobotRaconteur::RRObject> object, RR_SHARED_PTR<RobotRaconteur::ServerContext> context);
virtual RR_SHARED_PTR<RobotRaconteur::MessageEntry> CallGetProperty(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m);

virtual RR_SHARED_PTR<RobotRaconteur::MessageEntry> CallSetProperty(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m);

virtual RR_SHARED_PTR<RobotRaconteur::MessageEntry> CallFunction(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m);

virtual void ReleaseCastObject();

virtual void RegisterEvents(RR_SHARED_PTR<RobotRaconteur::RRObject> rrobj1);

virtual void UnregisterEvents(RR_SHARED_PTR<RobotRaconteur::RRObject> rrobj1);

virtual RR_SHARED_PTR<RobotRaconteur::RRObject> GetSubObj(const std::string &name, const std::string &ind);

virtual void InitPipeServers(RR_SHARED_PTR<RobotRaconteur::RRObject> rrobj1);

virtual void InitWireServers(RR_SHARED_PTR<RobotRaconteur::RRObject> rrobj1);

virtual void DispatchPipeMessage(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, uint32_t e);

virtual void DispatchWireMessage(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, uint32_t e);

virtual void InitCallbackServers(RR_SHARED_PTR<RobotRaconteur::RRObject> o);

virtual RR_SHARED_PTR<RobotRaconteur::MessageEntry> CallPipeFunction(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, uint32_t e);

virtual RR_SHARED_PTR<RobotRaconteur::MessageEntry> CallWireFunction(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, uint32_t e);

virtual RR_SHARED_PTR<void> GetCallbackFunction(uint32_t endpoint, const std::string& membername);

virtual RR_SHARED_PTR<RobotRaconteur::MessageEntry> CallMemoryFunction(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::Endpoint> e);

virtual std::string GetObjectType();
virtual RR_SHARED_PTR<Duckiebot::Camera::Camera > get_obj();

virtual RR_SHARED_PTR<Duckiebot::Camera::async_Camera > get_asyncobj();

protected:
static void rr_get_framerate(RR_WEAK_PTR<Duckiebot::Camera::Camera_skel> skel, double value, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
static void rr_get_resolution(RR_WEAK_PTR<Duckiebot::Camera::Camera_skel> skel, RR_SHARED_PTR<RobotRaconteur::RRArray<int32_t > > value, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
static void rr_get_format(RR_WEAK_PTR<Duckiebot::Camera::Camera_skel> skel, std::string value, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
static void rr_get_capturing(RR_WEAK_PTR<Duckiebot::Camera::Camera_skel> skel, uint8_t value, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
static void rr_startCapturing(RR_WEAK_PTR<Duckiebot::Camera::Camera_skel> skel, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
static void rr_stopCapturing(RR_WEAK_PTR<Duckiebot::Camera::Camera_skel> skel, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
static void rr_captureImage(RR_WEAK_PTR<Duckiebot::Camera::Camera_skel> skel, RR_SHARED_PTR<Duckiebot::Image > ret, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
static void rr_toggleFramerate(RR_WEAK_PTR<Duckiebot::Camera::Camera_skel> skel, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
static void rr_changeFormat(RR_WEAK_PTR<Duckiebot::Camera::Camera_skel> skel, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
 public:
protected:bool rr_InitPipeServersRun;
bool rr_InitWireServersRun;
RR_SHARED_PTR<RobotRaconteur::PipeServer<RR_SHARED_PTR<Duckiebot::Image > > > rr_ImageStream_pipe;
public: 
private:

};

}
}
