// Generated by gencpp from file dobot_bringup/EmergencyStop.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_EMERGENCYSTOP_H
#define DOBOT_BRINGUP_MESSAGE_EMERGENCYSTOP_H

#include <ros/service_traits.h>


#include <dobot_bringup/EmergencyStopRequest.h>
#include <dobot_bringup/EmergencyStopResponse.h>


namespace dobot_bringup
{

struct EmergencyStop
{

typedef EmergencyStopRequest Request;
typedef EmergencyStopResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct EmergencyStop
} // namespace dobot_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_bringup::EmergencyStop > {
  static const char* value()
  {
    return "ca16cfbd5443ad97f6cc7ffd6bb67292";
  }

  static const char* value(const ::dobot_bringup::EmergencyStop&) { return value(); }
};

template<>
struct DataType< ::dobot_bringup::EmergencyStop > {
  static const char* value()
  {
    return "dobot_bringup/EmergencyStop";
  }

  static const char* value(const ::dobot_bringup::EmergencyStop&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_bringup::EmergencyStopRequest> should match
// service_traits::MD5Sum< ::dobot_bringup::EmergencyStop >
template<>
struct MD5Sum< ::dobot_bringup::EmergencyStopRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::EmergencyStop >::value();
  }
  static const char* value(const ::dobot_bringup::EmergencyStopRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::EmergencyStopRequest> should match
// service_traits::DataType< ::dobot_bringup::EmergencyStop >
template<>
struct DataType< ::dobot_bringup::EmergencyStopRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::EmergencyStop >::value();
  }
  static const char* value(const ::dobot_bringup::EmergencyStopRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_bringup::EmergencyStopResponse> should match
// service_traits::MD5Sum< ::dobot_bringup::EmergencyStop >
template<>
struct MD5Sum< ::dobot_bringup::EmergencyStopResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::EmergencyStop >::value();
  }
  static const char* value(const ::dobot_bringup::EmergencyStopResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::EmergencyStopResponse> should match
// service_traits::DataType< ::dobot_bringup::EmergencyStop >
template<>
struct DataType< ::dobot_bringup::EmergencyStopResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::EmergencyStop >::value();
  }
  static const char* value(const ::dobot_bringup::EmergencyStopResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_EMERGENCYSTOP_H
