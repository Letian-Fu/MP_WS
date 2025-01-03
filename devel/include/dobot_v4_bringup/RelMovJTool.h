// Generated by gencpp from file dobot_v4_bringup/RelMovJTool.msg
// DO NOT EDIT!


#ifndef DOBOT_V4_BRINGUP_MESSAGE_RELMOVJTOOL_H
#define DOBOT_V4_BRINGUP_MESSAGE_RELMOVJTOOL_H

#include <ros/service_traits.h>


#include <dobot_v4_bringup/RelMovJToolRequest.h>
#include <dobot_v4_bringup/RelMovJToolResponse.h>


namespace dobot_v4_bringup
{

struct RelMovJTool
{

typedef RelMovJToolRequest Request;
typedef RelMovJToolResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct RelMovJTool
} // namespace dobot_v4_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_v4_bringup::RelMovJTool > {
  static const char* value()
  {
    return "acbcde02d4dabec81bbbb65ee94d60dc";
  }

  static const char* value(const ::dobot_v4_bringup::RelMovJTool&) { return value(); }
};

template<>
struct DataType< ::dobot_v4_bringup::RelMovJTool > {
  static const char* value()
  {
    return "dobot_v4_bringup/RelMovJTool";
  }

  static const char* value(const ::dobot_v4_bringup::RelMovJTool&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_v4_bringup::RelMovJToolRequest> should match
// service_traits::MD5Sum< ::dobot_v4_bringup::RelMovJTool >
template<>
struct MD5Sum< ::dobot_v4_bringup::RelMovJToolRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_v4_bringup::RelMovJTool >::value();
  }
  static const char* value(const ::dobot_v4_bringup::RelMovJToolRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_v4_bringup::RelMovJToolRequest> should match
// service_traits::DataType< ::dobot_v4_bringup::RelMovJTool >
template<>
struct DataType< ::dobot_v4_bringup::RelMovJToolRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_v4_bringup::RelMovJTool >::value();
  }
  static const char* value(const ::dobot_v4_bringup::RelMovJToolRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_v4_bringup::RelMovJToolResponse> should match
// service_traits::MD5Sum< ::dobot_v4_bringup::RelMovJTool >
template<>
struct MD5Sum< ::dobot_v4_bringup::RelMovJToolResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_v4_bringup::RelMovJTool >::value();
  }
  static const char* value(const ::dobot_v4_bringup::RelMovJToolResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_v4_bringup::RelMovJToolResponse> should match
// service_traits::DataType< ::dobot_v4_bringup::RelMovJTool >
template<>
struct DataType< ::dobot_v4_bringup::RelMovJToolResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_v4_bringup::RelMovJTool >::value();
  }
  static const char* value(const ::dobot_v4_bringup::RelMovJToolResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_V4_BRINGUP_MESSAGE_RELMOVJTOOL_H
