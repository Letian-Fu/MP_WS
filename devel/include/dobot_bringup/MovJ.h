// Generated by gencpp from file dobot_bringup/MovJ.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_MOVJ_H
#define DOBOT_BRINGUP_MESSAGE_MOVJ_H

#include <ros/service_traits.h>


#include <dobot_bringup/MovJRequest.h>
#include <dobot_bringup/MovJResponse.h>


namespace dobot_bringup
{

struct MovJ
{

typedef MovJRequest Request;
typedef MovJResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct MovJ
} // namespace dobot_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_bringup::MovJ > {
  static const char* value()
  {
    return "4c865d064beeb78c6b96e5e5798d8d9f";
  }

  static const char* value(const ::dobot_bringup::MovJ&) { return value(); }
};

template<>
struct DataType< ::dobot_bringup::MovJ > {
  static const char* value()
  {
    return "dobot_bringup/MovJ";
  }

  static const char* value(const ::dobot_bringup::MovJ&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_bringup::MovJRequest> should match
// service_traits::MD5Sum< ::dobot_bringup::MovJ >
template<>
struct MD5Sum< ::dobot_bringup::MovJRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::MovJ >::value();
  }
  static const char* value(const ::dobot_bringup::MovJRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::MovJRequest> should match
// service_traits::DataType< ::dobot_bringup::MovJ >
template<>
struct DataType< ::dobot_bringup::MovJRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::MovJ >::value();
  }
  static const char* value(const ::dobot_bringup::MovJRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_bringup::MovJResponse> should match
// service_traits::MD5Sum< ::dobot_bringup::MovJ >
template<>
struct MD5Sum< ::dobot_bringup::MovJResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::MovJ >::value();
  }
  static const char* value(const ::dobot_bringup::MovJResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::MovJResponse> should match
// service_traits::DataType< ::dobot_bringup::MovJ >
template<>
struct DataType< ::dobot_bringup::MovJResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::MovJ >::value();
  }
  static const char* value(const ::dobot_bringup::MovJResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_MOVJ_H