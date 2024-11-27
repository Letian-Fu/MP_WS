// Generated by gencpp from file dobot_v4_bringup/StopDrag.msg
// DO NOT EDIT!


#ifndef DOBOT_V4_BRINGUP_MESSAGE_STOPDRAG_H
#define DOBOT_V4_BRINGUP_MESSAGE_STOPDRAG_H

#include <ros/service_traits.h>


#include <dobot_v4_bringup/StopDragRequest.h>
#include <dobot_v4_bringup/StopDragResponse.h>


namespace dobot_v4_bringup
{

struct StopDrag
{

typedef StopDragRequest Request;
typedef StopDragResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct StopDrag
} // namespace dobot_v4_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_v4_bringup::StopDrag > {
  static const char* value()
  {
    return "ca16cfbd5443ad97f6cc7ffd6bb67292";
  }

  static const char* value(const ::dobot_v4_bringup::StopDrag&) { return value(); }
};

template<>
struct DataType< ::dobot_v4_bringup::StopDrag > {
  static const char* value()
  {
    return "dobot_v4_bringup/StopDrag";
  }

  static const char* value(const ::dobot_v4_bringup::StopDrag&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_v4_bringup::StopDragRequest> should match
// service_traits::MD5Sum< ::dobot_v4_bringup::StopDrag >
template<>
struct MD5Sum< ::dobot_v4_bringup::StopDragRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_v4_bringup::StopDrag >::value();
  }
  static const char* value(const ::dobot_v4_bringup::StopDragRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_v4_bringup::StopDragRequest> should match
// service_traits::DataType< ::dobot_v4_bringup::StopDrag >
template<>
struct DataType< ::dobot_v4_bringup::StopDragRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_v4_bringup::StopDrag >::value();
  }
  static const char* value(const ::dobot_v4_bringup::StopDragRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_v4_bringup::StopDragResponse> should match
// service_traits::MD5Sum< ::dobot_v4_bringup::StopDrag >
template<>
struct MD5Sum< ::dobot_v4_bringup::StopDragResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_v4_bringup::StopDrag >::value();
  }
  static const char* value(const ::dobot_v4_bringup::StopDragResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_v4_bringup::StopDragResponse> should match
// service_traits::DataType< ::dobot_v4_bringup::StopDrag >
template<>
struct DataType< ::dobot_v4_bringup::StopDragResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_v4_bringup::StopDrag >::value();
  }
  static const char* value(const ::dobot_v4_bringup::StopDragResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_V4_BRINGUP_MESSAGE_STOPDRAG_H
