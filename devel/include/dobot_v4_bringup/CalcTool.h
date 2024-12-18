// Generated by gencpp from file dobot_v4_bringup/CalcTool.msg
// DO NOT EDIT!


#ifndef DOBOT_V4_BRINGUP_MESSAGE_CALCTOOL_H
#define DOBOT_V4_BRINGUP_MESSAGE_CALCTOOL_H

#include <ros/service_traits.h>


#include <dobot_v4_bringup/CalcToolRequest.h>
#include <dobot_v4_bringup/CalcToolResponse.h>


namespace dobot_v4_bringup
{

struct CalcTool
{

typedef CalcToolRequest Request;
typedef CalcToolResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct CalcTool
} // namespace dobot_v4_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_v4_bringup::CalcTool > {
  static const char* value()
  {
    return "2ca5e99fbfe69882ece191e51f4e52e0";
  }

  static const char* value(const ::dobot_v4_bringup::CalcTool&) { return value(); }
};

template<>
struct DataType< ::dobot_v4_bringup::CalcTool > {
  static const char* value()
  {
    return "dobot_v4_bringup/CalcTool";
  }

  static const char* value(const ::dobot_v4_bringup::CalcTool&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_v4_bringup::CalcToolRequest> should match
// service_traits::MD5Sum< ::dobot_v4_bringup::CalcTool >
template<>
struct MD5Sum< ::dobot_v4_bringup::CalcToolRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_v4_bringup::CalcTool >::value();
  }
  static const char* value(const ::dobot_v4_bringup::CalcToolRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_v4_bringup::CalcToolRequest> should match
// service_traits::DataType< ::dobot_v4_bringup::CalcTool >
template<>
struct DataType< ::dobot_v4_bringup::CalcToolRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_v4_bringup::CalcTool >::value();
  }
  static const char* value(const ::dobot_v4_bringup::CalcToolRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_v4_bringup::CalcToolResponse> should match
// service_traits::MD5Sum< ::dobot_v4_bringup::CalcTool >
template<>
struct MD5Sum< ::dobot_v4_bringup::CalcToolResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_v4_bringup::CalcTool >::value();
  }
  static const char* value(const ::dobot_v4_bringup::CalcToolResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_v4_bringup::CalcToolResponse> should match
// service_traits::DataType< ::dobot_v4_bringup::CalcTool >
template<>
struct DataType< ::dobot_v4_bringup::CalcToolResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_v4_bringup::CalcTool >::value();
  }
  static const char* value(const ::dobot_v4_bringup::CalcToolResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_V4_BRINGUP_MESSAGE_CALCTOOL_H
