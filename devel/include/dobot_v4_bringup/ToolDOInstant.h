// Generated by gencpp from file dobot_v4_bringup/ToolDOInstant.msg
// DO NOT EDIT!


#ifndef DOBOT_V4_BRINGUP_MESSAGE_TOOLDOINSTANT_H
#define DOBOT_V4_BRINGUP_MESSAGE_TOOLDOINSTANT_H

#include <ros/service_traits.h>


#include <dobot_v4_bringup/ToolDOInstantRequest.h>
#include <dobot_v4_bringup/ToolDOInstantResponse.h>


namespace dobot_v4_bringup
{

struct ToolDOInstant
{

typedef ToolDOInstantRequest Request;
typedef ToolDOInstantResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ToolDOInstant
} // namespace dobot_v4_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_v4_bringup::ToolDOInstant > {
  static const char* value()
  {
    return "096263ad689c1ece47cec4376ad5d3b5";
  }

  static const char* value(const ::dobot_v4_bringup::ToolDOInstant&) { return value(); }
};

template<>
struct DataType< ::dobot_v4_bringup::ToolDOInstant > {
  static const char* value()
  {
    return "dobot_v4_bringup/ToolDOInstant";
  }

  static const char* value(const ::dobot_v4_bringup::ToolDOInstant&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_v4_bringup::ToolDOInstantRequest> should match
// service_traits::MD5Sum< ::dobot_v4_bringup::ToolDOInstant >
template<>
struct MD5Sum< ::dobot_v4_bringup::ToolDOInstantRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_v4_bringup::ToolDOInstant >::value();
  }
  static const char* value(const ::dobot_v4_bringup::ToolDOInstantRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_v4_bringup::ToolDOInstantRequest> should match
// service_traits::DataType< ::dobot_v4_bringup::ToolDOInstant >
template<>
struct DataType< ::dobot_v4_bringup::ToolDOInstantRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_v4_bringup::ToolDOInstant >::value();
  }
  static const char* value(const ::dobot_v4_bringup::ToolDOInstantRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_v4_bringup::ToolDOInstantResponse> should match
// service_traits::MD5Sum< ::dobot_v4_bringup::ToolDOInstant >
template<>
struct MD5Sum< ::dobot_v4_bringup::ToolDOInstantResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_v4_bringup::ToolDOInstant >::value();
  }
  static const char* value(const ::dobot_v4_bringup::ToolDOInstantResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_v4_bringup::ToolDOInstantResponse> should match
// service_traits::DataType< ::dobot_v4_bringup::ToolDOInstant >
template<>
struct DataType< ::dobot_v4_bringup::ToolDOInstantResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_v4_bringup::ToolDOInstant >::value();
  }
  static const char* value(const ::dobot_v4_bringup::ToolDOInstantResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_V4_BRINGUP_MESSAGE_TOOLDOINSTANT_H
