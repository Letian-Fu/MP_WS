// Generated by gencpp from file dobot_v4_bringup/DOInstant.msg
// DO NOT EDIT!


#ifndef DOBOT_V4_BRINGUP_MESSAGE_DOINSTANT_H
#define DOBOT_V4_BRINGUP_MESSAGE_DOINSTANT_H

#include <ros/service_traits.h>


#include <dobot_v4_bringup/DOInstantRequest.h>
#include <dobot_v4_bringup/DOInstantResponse.h>


namespace dobot_v4_bringup
{

struct DOInstant
{

typedef DOInstantRequest Request;
typedef DOInstantResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct DOInstant
} // namespace dobot_v4_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_v4_bringup::DOInstant > {
  static const char* value()
  {
    return "096263ad689c1ece47cec4376ad5d3b5";
  }

  static const char* value(const ::dobot_v4_bringup::DOInstant&) { return value(); }
};

template<>
struct DataType< ::dobot_v4_bringup::DOInstant > {
  static const char* value()
  {
    return "dobot_v4_bringup/DOInstant";
  }

  static const char* value(const ::dobot_v4_bringup::DOInstant&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_v4_bringup::DOInstantRequest> should match
// service_traits::MD5Sum< ::dobot_v4_bringup::DOInstant >
template<>
struct MD5Sum< ::dobot_v4_bringup::DOInstantRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_v4_bringup::DOInstant >::value();
  }
  static const char* value(const ::dobot_v4_bringup::DOInstantRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_v4_bringup::DOInstantRequest> should match
// service_traits::DataType< ::dobot_v4_bringup::DOInstant >
template<>
struct DataType< ::dobot_v4_bringup::DOInstantRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_v4_bringup::DOInstant >::value();
  }
  static const char* value(const ::dobot_v4_bringup::DOInstantRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_v4_bringup::DOInstantResponse> should match
// service_traits::MD5Sum< ::dobot_v4_bringup::DOInstant >
template<>
struct MD5Sum< ::dobot_v4_bringup::DOInstantResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_v4_bringup::DOInstant >::value();
  }
  static const char* value(const ::dobot_v4_bringup::DOInstantResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_v4_bringup::DOInstantResponse> should match
// service_traits::DataType< ::dobot_v4_bringup::DOInstant >
template<>
struct DataType< ::dobot_v4_bringup::DOInstantResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_v4_bringup::DOInstant >::value();
  }
  static const char* value(const ::dobot_v4_bringup::DOInstantResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_V4_BRINGUP_MESSAGE_DOINSTANT_H
