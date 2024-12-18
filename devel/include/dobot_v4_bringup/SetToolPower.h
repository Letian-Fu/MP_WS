// Generated by gencpp from file dobot_v4_bringup/SetToolPower.msg
// DO NOT EDIT!


#ifndef DOBOT_V4_BRINGUP_MESSAGE_SETTOOLPOWER_H
#define DOBOT_V4_BRINGUP_MESSAGE_SETTOOLPOWER_H

#include <ros/service_traits.h>


#include <dobot_v4_bringup/SetToolPowerRequest.h>
#include <dobot_v4_bringup/SetToolPowerResponse.h>


namespace dobot_v4_bringup
{

struct SetToolPower
{

typedef SetToolPowerRequest Request;
typedef SetToolPowerResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetToolPower
} // namespace dobot_v4_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_v4_bringup::SetToolPower > {
  static const char* value()
  {
    return "e9885118d8baa0e46bd047b6deaa3f81";
  }

  static const char* value(const ::dobot_v4_bringup::SetToolPower&) { return value(); }
};

template<>
struct DataType< ::dobot_v4_bringup::SetToolPower > {
  static const char* value()
  {
    return "dobot_v4_bringup/SetToolPower";
  }

  static const char* value(const ::dobot_v4_bringup::SetToolPower&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_v4_bringup::SetToolPowerRequest> should match
// service_traits::MD5Sum< ::dobot_v4_bringup::SetToolPower >
template<>
struct MD5Sum< ::dobot_v4_bringup::SetToolPowerRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_v4_bringup::SetToolPower >::value();
  }
  static const char* value(const ::dobot_v4_bringup::SetToolPowerRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_v4_bringup::SetToolPowerRequest> should match
// service_traits::DataType< ::dobot_v4_bringup::SetToolPower >
template<>
struct DataType< ::dobot_v4_bringup::SetToolPowerRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_v4_bringup::SetToolPower >::value();
  }
  static const char* value(const ::dobot_v4_bringup::SetToolPowerRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_v4_bringup::SetToolPowerResponse> should match
// service_traits::MD5Sum< ::dobot_v4_bringup::SetToolPower >
template<>
struct MD5Sum< ::dobot_v4_bringup::SetToolPowerResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_v4_bringup::SetToolPower >::value();
  }
  static const char* value(const ::dobot_v4_bringup::SetToolPowerResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_v4_bringup::SetToolPowerResponse> should match
// service_traits::DataType< ::dobot_v4_bringup::SetToolPower >
template<>
struct DataType< ::dobot_v4_bringup::SetToolPowerResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_v4_bringup::SetToolPower >::value();
  }
  static const char* value(const ::dobot_v4_bringup::SetToolPowerResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_V4_BRINGUP_MESSAGE_SETTOOLPOWER_H
