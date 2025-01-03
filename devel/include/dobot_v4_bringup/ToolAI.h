// Generated by gencpp from file dobot_v4_bringup/ToolAI.msg
// DO NOT EDIT!


#ifndef DOBOT_V4_BRINGUP_MESSAGE_TOOLAI_H
#define DOBOT_V4_BRINGUP_MESSAGE_TOOLAI_H

#include <ros/service_traits.h>


#include <dobot_v4_bringup/ToolAIRequest.h>
#include <dobot_v4_bringup/ToolAIResponse.h>


namespace dobot_v4_bringup
{

struct ToolAI
{

typedef ToolAIRequest Request;
typedef ToolAIResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ToolAI
} // namespace dobot_v4_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_v4_bringup::ToolAI > {
  static const char* value()
  {
    return "6d753c0adbdd6c06a6eb5c36aec96b0c";
  }

  static const char* value(const ::dobot_v4_bringup::ToolAI&) { return value(); }
};

template<>
struct DataType< ::dobot_v4_bringup::ToolAI > {
  static const char* value()
  {
    return "dobot_v4_bringup/ToolAI";
  }

  static const char* value(const ::dobot_v4_bringup::ToolAI&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_v4_bringup::ToolAIRequest> should match
// service_traits::MD5Sum< ::dobot_v4_bringup::ToolAI >
template<>
struct MD5Sum< ::dobot_v4_bringup::ToolAIRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_v4_bringup::ToolAI >::value();
  }
  static const char* value(const ::dobot_v4_bringup::ToolAIRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_v4_bringup::ToolAIRequest> should match
// service_traits::DataType< ::dobot_v4_bringup::ToolAI >
template<>
struct DataType< ::dobot_v4_bringup::ToolAIRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_v4_bringup::ToolAI >::value();
  }
  static const char* value(const ::dobot_v4_bringup::ToolAIRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_v4_bringup::ToolAIResponse> should match
// service_traits::MD5Sum< ::dobot_v4_bringup::ToolAI >
template<>
struct MD5Sum< ::dobot_v4_bringup::ToolAIResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_v4_bringup::ToolAI >::value();
  }
  static const char* value(const ::dobot_v4_bringup::ToolAIResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_v4_bringup::ToolAIResponse> should match
// service_traits::DataType< ::dobot_v4_bringup::ToolAI >
template<>
struct DataType< ::dobot_v4_bringup::ToolAIResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_v4_bringup::ToolAI >::value();
  }
  static const char* value(const ::dobot_v4_bringup::ToolAIResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_V4_BRINGUP_MESSAGE_TOOLAI_H
