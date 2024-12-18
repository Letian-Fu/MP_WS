// Generated by gencpp from file dobot_v4_bringup/SetCollisionLevel.msg
// DO NOT EDIT!


#ifndef DOBOT_V4_BRINGUP_MESSAGE_SETCOLLISIONLEVEL_H
#define DOBOT_V4_BRINGUP_MESSAGE_SETCOLLISIONLEVEL_H

#include <ros/service_traits.h>


#include <dobot_v4_bringup/SetCollisionLevelRequest.h>
#include <dobot_v4_bringup/SetCollisionLevelResponse.h>


namespace dobot_v4_bringup
{

struct SetCollisionLevel
{

typedef SetCollisionLevelRequest Request;
typedef SetCollisionLevelResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetCollisionLevel
} // namespace dobot_v4_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_v4_bringup::SetCollisionLevel > {
  static const char* value()
  {
    return "66c784877185ea647f602bc2ad14ae86";
  }

  static const char* value(const ::dobot_v4_bringup::SetCollisionLevel&) { return value(); }
};

template<>
struct DataType< ::dobot_v4_bringup::SetCollisionLevel > {
  static const char* value()
  {
    return "dobot_v4_bringup/SetCollisionLevel";
  }

  static const char* value(const ::dobot_v4_bringup::SetCollisionLevel&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_v4_bringup::SetCollisionLevelRequest> should match
// service_traits::MD5Sum< ::dobot_v4_bringup::SetCollisionLevel >
template<>
struct MD5Sum< ::dobot_v4_bringup::SetCollisionLevelRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_v4_bringup::SetCollisionLevel >::value();
  }
  static const char* value(const ::dobot_v4_bringup::SetCollisionLevelRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_v4_bringup::SetCollisionLevelRequest> should match
// service_traits::DataType< ::dobot_v4_bringup::SetCollisionLevel >
template<>
struct DataType< ::dobot_v4_bringup::SetCollisionLevelRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_v4_bringup::SetCollisionLevel >::value();
  }
  static const char* value(const ::dobot_v4_bringup::SetCollisionLevelRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_v4_bringup::SetCollisionLevelResponse> should match
// service_traits::MD5Sum< ::dobot_v4_bringup::SetCollisionLevel >
template<>
struct MD5Sum< ::dobot_v4_bringup::SetCollisionLevelResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_v4_bringup::SetCollisionLevel >::value();
  }
  static const char* value(const ::dobot_v4_bringup::SetCollisionLevelResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_v4_bringup::SetCollisionLevelResponse> should match
// service_traits::DataType< ::dobot_v4_bringup::SetCollisionLevel >
template<>
struct DataType< ::dobot_v4_bringup::SetCollisionLevelResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_v4_bringup::SetCollisionLevel >::value();
  }
  static const char* value(const ::dobot_v4_bringup::SetCollisionLevelResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_V4_BRINGUP_MESSAGE_SETCOLLISIONLEVEL_H
