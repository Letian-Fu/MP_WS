// Generated by gencpp from file dobot_v4_bringup/DragSensivity.msg
// DO NOT EDIT!


#ifndef DOBOT_V4_BRINGUP_MESSAGE_DRAGSENSIVITY_H
#define DOBOT_V4_BRINGUP_MESSAGE_DRAGSENSIVITY_H

#include <ros/service_traits.h>


#include <dobot_v4_bringup/DragSensivityRequest.h>
#include <dobot_v4_bringup/DragSensivityResponse.h>


namespace dobot_v4_bringup
{

struct DragSensivity
{

typedef DragSensivityRequest Request;
typedef DragSensivityResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct DragSensivity
} // namespace dobot_v4_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_v4_bringup::DragSensivity > {
  static const char* value()
  {
    return "c9ffa71f693aabb4ec23d98e0cce7e29";
  }

  static const char* value(const ::dobot_v4_bringup::DragSensivity&) { return value(); }
};

template<>
struct DataType< ::dobot_v4_bringup::DragSensivity > {
  static const char* value()
  {
    return "dobot_v4_bringup/DragSensivity";
  }

  static const char* value(const ::dobot_v4_bringup::DragSensivity&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_v4_bringup::DragSensivityRequest> should match
// service_traits::MD5Sum< ::dobot_v4_bringup::DragSensivity >
template<>
struct MD5Sum< ::dobot_v4_bringup::DragSensivityRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_v4_bringup::DragSensivity >::value();
  }
  static const char* value(const ::dobot_v4_bringup::DragSensivityRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_v4_bringup::DragSensivityRequest> should match
// service_traits::DataType< ::dobot_v4_bringup::DragSensivity >
template<>
struct DataType< ::dobot_v4_bringup::DragSensivityRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_v4_bringup::DragSensivity >::value();
  }
  static const char* value(const ::dobot_v4_bringup::DragSensivityRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_v4_bringup::DragSensivityResponse> should match
// service_traits::MD5Sum< ::dobot_v4_bringup::DragSensivity >
template<>
struct MD5Sum< ::dobot_v4_bringup::DragSensivityResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_v4_bringup::DragSensivity >::value();
  }
  static const char* value(const ::dobot_v4_bringup::DragSensivityResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_v4_bringup::DragSensivityResponse> should match
// service_traits::DataType< ::dobot_v4_bringup::DragSensivity >
template<>
struct DataType< ::dobot_v4_bringup::DragSensivityResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_v4_bringup::DragSensivity >::value();
  }
  static const char* value(const ::dobot_v4_bringup::DragSensivityResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_V4_BRINGUP_MESSAGE_DRAGSENSIVITY_H
