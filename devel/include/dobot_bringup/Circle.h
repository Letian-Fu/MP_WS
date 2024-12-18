// Generated by gencpp from file dobot_bringup/Circle.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_CIRCLE_H
#define DOBOT_BRINGUP_MESSAGE_CIRCLE_H

#include <ros/service_traits.h>


#include <dobot_bringup/CircleRequest.h>
#include <dobot_bringup/CircleResponse.h>


namespace dobot_bringup
{

struct Circle
{

typedef CircleRequest Request;
typedef CircleResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Circle
} // namespace dobot_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_bringup::Circle > {
  static const char* value()
  {
    return "595948d4551fca3138762c937ac2d5fd";
  }

  static const char* value(const ::dobot_bringup::Circle&) { return value(); }
};

template<>
struct DataType< ::dobot_bringup::Circle > {
  static const char* value()
  {
    return "dobot_bringup/Circle";
  }

  static const char* value(const ::dobot_bringup::Circle&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_bringup::CircleRequest> should match
// service_traits::MD5Sum< ::dobot_bringup::Circle >
template<>
struct MD5Sum< ::dobot_bringup::CircleRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::Circle >::value();
  }
  static const char* value(const ::dobot_bringup::CircleRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::CircleRequest> should match
// service_traits::DataType< ::dobot_bringup::Circle >
template<>
struct DataType< ::dobot_bringup::CircleRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::Circle >::value();
  }
  static const char* value(const ::dobot_bringup::CircleRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_bringup::CircleResponse> should match
// service_traits::MD5Sum< ::dobot_bringup::Circle >
template<>
struct MD5Sum< ::dobot_bringup::CircleResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::Circle >::value();
  }
  static const char* value(const ::dobot_bringup::CircleResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::CircleResponse> should match
// service_traits::DataType< ::dobot_bringup::Circle >
template<>
struct DataType< ::dobot_bringup::CircleResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::Circle >::value();
  }
  static const char* value(const ::dobot_bringup::CircleResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_CIRCLE_H
