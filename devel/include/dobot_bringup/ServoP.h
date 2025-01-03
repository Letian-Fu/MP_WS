// Generated by gencpp from file dobot_bringup/ServoP.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_SERVOP_H
#define DOBOT_BRINGUP_MESSAGE_SERVOP_H

#include <ros/service_traits.h>


#include <dobot_bringup/ServoPRequest.h>
#include <dobot_bringup/ServoPResponse.h>


namespace dobot_bringup
{

struct ServoP
{

typedef ServoPRequest Request;
typedef ServoPResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ServoP
} // namespace dobot_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_bringup::ServoP > {
  static const char* value()
  {
    return "4d5bd0c11266916c52206a250a837c3f";
  }

  static const char* value(const ::dobot_bringup::ServoP&) { return value(); }
};

template<>
struct DataType< ::dobot_bringup::ServoP > {
  static const char* value()
  {
    return "dobot_bringup/ServoP";
  }

  static const char* value(const ::dobot_bringup::ServoP&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_bringup::ServoPRequest> should match
// service_traits::MD5Sum< ::dobot_bringup::ServoP >
template<>
struct MD5Sum< ::dobot_bringup::ServoPRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::ServoP >::value();
  }
  static const char* value(const ::dobot_bringup::ServoPRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::ServoPRequest> should match
// service_traits::DataType< ::dobot_bringup::ServoP >
template<>
struct DataType< ::dobot_bringup::ServoPRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::ServoP >::value();
  }
  static const char* value(const ::dobot_bringup::ServoPRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_bringup::ServoPResponse> should match
// service_traits::MD5Sum< ::dobot_bringup::ServoP >
template<>
struct MD5Sum< ::dobot_bringup::ServoPResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::ServoP >::value();
  }
  static const char* value(const ::dobot_bringup::ServoPResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::ServoPResponse> should match
// service_traits::DataType< ::dobot_bringup::ServoP >
template<>
struct DataType< ::dobot_bringup::ServoPResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::ServoP >::value();
  }
  static const char* value(const ::dobot_bringup::ServoPResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_SERVOP_H
