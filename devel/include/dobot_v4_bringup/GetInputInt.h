// Generated by gencpp from file dobot_v4_bringup/GetInputInt.msg
// DO NOT EDIT!


#ifndef DOBOT_V4_BRINGUP_MESSAGE_GETINPUTINT_H
#define DOBOT_V4_BRINGUP_MESSAGE_GETINPUTINT_H

#include <ros/service_traits.h>


#include <dobot_v4_bringup/GetInputIntRequest.h>
#include <dobot_v4_bringup/GetInputIntResponse.h>


namespace dobot_v4_bringup
{

struct GetInputInt
{

typedef GetInputIntRequest Request;
typedef GetInputIntResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetInputInt
} // namespace dobot_v4_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_v4_bringup::GetInputInt > {
  static const char* value()
  {
    return "4f4e99215b78104599bc3fd88b4cdc1c";
  }

  static const char* value(const ::dobot_v4_bringup::GetInputInt&) { return value(); }
};

template<>
struct DataType< ::dobot_v4_bringup::GetInputInt > {
  static const char* value()
  {
    return "dobot_v4_bringup/GetInputInt";
  }

  static const char* value(const ::dobot_v4_bringup::GetInputInt&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_v4_bringup::GetInputIntRequest> should match
// service_traits::MD5Sum< ::dobot_v4_bringup::GetInputInt >
template<>
struct MD5Sum< ::dobot_v4_bringup::GetInputIntRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_v4_bringup::GetInputInt >::value();
  }
  static const char* value(const ::dobot_v4_bringup::GetInputIntRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_v4_bringup::GetInputIntRequest> should match
// service_traits::DataType< ::dobot_v4_bringup::GetInputInt >
template<>
struct DataType< ::dobot_v4_bringup::GetInputIntRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_v4_bringup::GetInputInt >::value();
  }
  static const char* value(const ::dobot_v4_bringup::GetInputIntRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_v4_bringup::GetInputIntResponse> should match
// service_traits::MD5Sum< ::dobot_v4_bringup::GetInputInt >
template<>
struct MD5Sum< ::dobot_v4_bringup::GetInputIntResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_v4_bringup::GetInputInt >::value();
  }
  static const char* value(const ::dobot_v4_bringup::GetInputIntResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_v4_bringup::GetInputIntResponse> should match
// service_traits::DataType< ::dobot_v4_bringup::GetInputInt >
template<>
struct DataType< ::dobot_v4_bringup::GetInputIntResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_v4_bringup::GetInputInt >::value();
  }
  static const char* value(const ::dobot_v4_bringup::GetInputIntResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_V4_BRINGUP_MESSAGE_GETINPUTINT_H