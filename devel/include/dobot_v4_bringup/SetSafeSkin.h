// Generated by gencpp from file dobot_v4_bringup/SetSafeSkin.msg
// DO NOT EDIT!


#ifndef DOBOT_V4_BRINGUP_MESSAGE_SETSAFESKIN_H
#define DOBOT_V4_BRINGUP_MESSAGE_SETSAFESKIN_H

#include <ros/service_traits.h>


#include <dobot_v4_bringup/SetSafeSkinRequest.h>
#include <dobot_v4_bringup/SetSafeSkinResponse.h>


namespace dobot_v4_bringup
{

struct SetSafeSkin
{

typedef SetSafeSkinRequest Request;
typedef SetSafeSkinResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetSafeSkin
} // namespace dobot_v4_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_v4_bringup::SetSafeSkin > {
  static const char* value()
  {
    return "8797dfb6f36af0bf78c64c64affcc433";
  }

  static const char* value(const ::dobot_v4_bringup::SetSafeSkin&) { return value(); }
};

template<>
struct DataType< ::dobot_v4_bringup::SetSafeSkin > {
  static const char* value()
  {
    return "dobot_v4_bringup/SetSafeSkin";
  }

  static const char* value(const ::dobot_v4_bringup::SetSafeSkin&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_v4_bringup::SetSafeSkinRequest> should match
// service_traits::MD5Sum< ::dobot_v4_bringup::SetSafeSkin >
template<>
struct MD5Sum< ::dobot_v4_bringup::SetSafeSkinRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_v4_bringup::SetSafeSkin >::value();
  }
  static const char* value(const ::dobot_v4_bringup::SetSafeSkinRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_v4_bringup::SetSafeSkinRequest> should match
// service_traits::DataType< ::dobot_v4_bringup::SetSafeSkin >
template<>
struct DataType< ::dobot_v4_bringup::SetSafeSkinRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_v4_bringup::SetSafeSkin >::value();
  }
  static const char* value(const ::dobot_v4_bringup::SetSafeSkinRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_v4_bringup::SetSafeSkinResponse> should match
// service_traits::MD5Sum< ::dobot_v4_bringup::SetSafeSkin >
template<>
struct MD5Sum< ::dobot_v4_bringup::SetSafeSkinResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_v4_bringup::SetSafeSkin >::value();
  }
  static const char* value(const ::dobot_v4_bringup::SetSafeSkinResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_v4_bringup::SetSafeSkinResponse> should match
// service_traits::DataType< ::dobot_v4_bringup::SetSafeSkin >
template<>
struct DataType< ::dobot_v4_bringup::SetSafeSkinResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_v4_bringup::SetSafeSkin >::value();
  }
  static const char* value(const ::dobot_v4_bringup::SetSafeSkinResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_V4_BRINGUP_MESSAGE_SETSAFESKIN_H
