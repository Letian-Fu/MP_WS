// Generated by gencpp from file dobot_bringup/DOExecute.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_DOEXECUTE_H
#define DOBOT_BRINGUP_MESSAGE_DOEXECUTE_H

#include <ros/service_traits.h>


#include <dobot_bringup/DOExecuteRequest.h>
#include <dobot_bringup/DOExecuteResponse.h>


namespace dobot_bringup
{

struct DOExecute
{

typedef DOExecuteRequest Request;
typedef DOExecuteResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct DOExecute
} // namespace dobot_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_bringup::DOExecute > {
  static const char* value()
  {
    return "096263ad689c1ece47cec4376ad5d3b5";
  }

  static const char* value(const ::dobot_bringup::DOExecute&) { return value(); }
};

template<>
struct DataType< ::dobot_bringup::DOExecute > {
  static const char* value()
  {
    return "dobot_bringup/DOExecute";
  }

  static const char* value(const ::dobot_bringup::DOExecute&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_bringup::DOExecuteRequest> should match
// service_traits::MD5Sum< ::dobot_bringup::DOExecute >
template<>
struct MD5Sum< ::dobot_bringup::DOExecuteRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::DOExecute >::value();
  }
  static const char* value(const ::dobot_bringup::DOExecuteRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::DOExecuteRequest> should match
// service_traits::DataType< ::dobot_bringup::DOExecute >
template<>
struct DataType< ::dobot_bringup::DOExecuteRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::DOExecute >::value();
  }
  static const char* value(const ::dobot_bringup::DOExecuteRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_bringup::DOExecuteResponse> should match
// service_traits::MD5Sum< ::dobot_bringup::DOExecute >
template<>
struct MD5Sum< ::dobot_bringup::DOExecuteResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::DOExecute >::value();
  }
  static const char* value(const ::dobot_bringup::DOExecuteResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::DOExecuteResponse> should match
// service_traits::DataType< ::dobot_bringup::DOExecute >
template<>
struct DataType< ::dobot_bringup::DOExecuteResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::DOExecute >::value();
  }
  static const char* value(const ::dobot_bringup::DOExecuteResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_DOEXECUTE_H