// Generated by gencpp from file dobot_v4_bringup/AOInstant.msg
// DO NOT EDIT!


#ifndef DOBOT_V4_BRINGUP_MESSAGE_AOINSTANT_H
#define DOBOT_V4_BRINGUP_MESSAGE_AOINSTANT_H

#include <ros/service_traits.h>


#include <dobot_v4_bringup/AOInstantRequest.h>
#include <dobot_v4_bringup/AOInstantResponse.h>


namespace dobot_v4_bringup
{

struct AOInstant
{

typedef AOInstantRequest Request;
typedef AOInstantResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct AOInstant
} // namespace dobot_v4_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_v4_bringup::AOInstant > {
  static const char* value()
  {
    return "c9ffa71f693aabb4ec23d98e0cce7e29";
  }

  static const char* value(const ::dobot_v4_bringup::AOInstant&) { return value(); }
};

template<>
struct DataType< ::dobot_v4_bringup::AOInstant > {
  static const char* value()
  {
    return "dobot_v4_bringup/AOInstant";
  }

  static const char* value(const ::dobot_v4_bringup::AOInstant&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_v4_bringup::AOInstantRequest> should match
// service_traits::MD5Sum< ::dobot_v4_bringup::AOInstant >
template<>
struct MD5Sum< ::dobot_v4_bringup::AOInstantRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_v4_bringup::AOInstant >::value();
  }
  static const char* value(const ::dobot_v4_bringup::AOInstantRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_v4_bringup::AOInstantRequest> should match
// service_traits::DataType< ::dobot_v4_bringup::AOInstant >
template<>
struct DataType< ::dobot_v4_bringup::AOInstantRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_v4_bringup::AOInstant >::value();
  }
  static const char* value(const ::dobot_v4_bringup::AOInstantRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_v4_bringup::AOInstantResponse> should match
// service_traits::MD5Sum< ::dobot_v4_bringup::AOInstant >
template<>
struct MD5Sum< ::dobot_v4_bringup::AOInstantResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_v4_bringup::AOInstant >::value();
  }
  static const char* value(const ::dobot_v4_bringup::AOInstantResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_v4_bringup::AOInstantResponse> should match
// service_traits::DataType< ::dobot_v4_bringup::AOInstant >
template<>
struct DataType< ::dobot_v4_bringup::AOInstantResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_v4_bringup::AOInstant >::value();
  }
  static const char* value(const ::dobot_v4_bringup::AOInstantResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_V4_BRINGUP_MESSAGE_AOINSTANT_H
