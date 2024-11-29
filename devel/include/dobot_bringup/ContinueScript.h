// Generated by gencpp from file dobot_bringup/ContinueScript.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_CONTINUESCRIPT_H
#define DOBOT_BRINGUP_MESSAGE_CONTINUESCRIPT_H

#include <ros/service_traits.h>


#include <dobot_bringup/ContinueScriptRequest.h>
#include <dobot_bringup/ContinueScriptResponse.h>


namespace dobot_bringup
{

struct ContinueScript
{

typedef ContinueScriptRequest Request;
typedef ContinueScriptResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ContinueScript
} // namespace dobot_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_bringup::ContinueScript > {
  static const char* value()
  {
    return "ca16cfbd5443ad97f6cc7ffd6bb67292";
  }

  static const char* value(const ::dobot_bringup::ContinueScript&) { return value(); }
};

template<>
struct DataType< ::dobot_bringup::ContinueScript > {
  static const char* value()
  {
    return "dobot_bringup/ContinueScript";
  }

  static const char* value(const ::dobot_bringup::ContinueScript&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_bringup::ContinueScriptRequest> should match
// service_traits::MD5Sum< ::dobot_bringup::ContinueScript >
template<>
struct MD5Sum< ::dobot_bringup::ContinueScriptRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::ContinueScript >::value();
  }
  static const char* value(const ::dobot_bringup::ContinueScriptRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::ContinueScriptRequest> should match
// service_traits::DataType< ::dobot_bringup::ContinueScript >
template<>
struct DataType< ::dobot_bringup::ContinueScriptRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::ContinueScript >::value();
  }
  static const char* value(const ::dobot_bringup::ContinueScriptRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_bringup::ContinueScriptResponse> should match
// service_traits::MD5Sum< ::dobot_bringup::ContinueScript >
template<>
struct MD5Sum< ::dobot_bringup::ContinueScriptResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::ContinueScript >::value();
  }
  static const char* value(const ::dobot_bringup::ContinueScriptResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::ContinueScriptResponse> should match
// service_traits::DataType< ::dobot_bringup::ContinueScript >
template<>
struct DataType< ::dobot_bringup::ContinueScriptResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::ContinueScript >::value();
  }
  static const char* value(const ::dobot_bringup::ContinueScriptResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_CONTINUESCRIPT_H