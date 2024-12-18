// Generated by gencpp from file dobot_bringup/RunScript.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_RUNSCRIPT_H
#define DOBOT_BRINGUP_MESSAGE_RUNSCRIPT_H

#include <ros/service_traits.h>


#include <dobot_bringup/RunScriptRequest.h>
#include <dobot_bringup/RunScriptResponse.h>


namespace dobot_bringup
{

struct RunScript
{

typedef RunScriptRequest Request;
typedef RunScriptResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct RunScript
} // namespace dobot_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_bringup::RunScript > {
  static const char* value()
  {
    return "7a897c62a8cb97cf0bd8a288103abd8a";
  }

  static const char* value(const ::dobot_bringup::RunScript&) { return value(); }
};

template<>
struct DataType< ::dobot_bringup::RunScript > {
  static const char* value()
  {
    return "dobot_bringup/RunScript";
  }

  static const char* value(const ::dobot_bringup::RunScript&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_bringup::RunScriptRequest> should match
// service_traits::MD5Sum< ::dobot_bringup::RunScript >
template<>
struct MD5Sum< ::dobot_bringup::RunScriptRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::RunScript >::value();
  }
  static const char* value(const ::dobot_bringup::RunScriptRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::RunScriptRequest> should match
// service_traits::DataType< ::dobot_bringup::RunScript >
template<>
struct DataType< ::dobot_bringup::RunScriptRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::RunScript >::value();
  }
  static const char* value(const ::dobot_bringup::RunScriptRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_bringup::RunScriptResponse> should match
// service_traits::MD5Sum< ::dobot_bringup::RunScript >
template<>
struct MD5Sum< ::dobot_bringup::RunScriptResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::RunScript >::value();
  }
  static const char* value(const ::dobot_bringup::RunScriptResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::RunScriptResponse> should match
// service_traits::DataType< ::dobot_bringup::RunScript >
template<>
struct DataType< ::dobot_bringup::RunScriptResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::RunScript >::value();
  }
  static const char* value(const ::dobot_bringup::RunScriptResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_RUNSCRIPT_H
