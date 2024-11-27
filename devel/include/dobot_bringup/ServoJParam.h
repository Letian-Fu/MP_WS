// Generated by gencpp from file dobot_bringup/ServoJParam.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_SERVOJPARAM_H
#define DOBOT_BRINGUP_MESSAGE_SERVOJPARAM_H

#include <ros/service_traits.h>


#include <dobot_bringup/ServoJParamRequest.h>
#include <dobot_bringup/ServoJParamResponse.h>


namespace dobot_bringup
{

struct ServoJParam
{

typedef ServoJParamRequest Request;
typedef ServoJParamResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ServoJParam
} // namespace dobot_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_bringup::ServoJParam > {
  static const char* value()
  {
    return "039b017b1ca1d408116139346f28d908";
  }

  static const char* value(const ::dobot_bringup::ServoJParam&) { return value(); }
};

template<>
struct DataType< ::dobot_bringup::ServoJParam > {
  static const char* value()
  {
    return "dobot_bringup/ServoJParam";
  }

  static const char* value(const ::dobot_bringup::ServoJParam&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_bringup::ServoJParamRequest> should match
// service_traits::MD5Sum< ::dobot_bringup::ServoJParam >
template<>
struct MD5Sum< ::dobot_bringup::ServoJParamRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::ServoJParam >::value();
  }
  static const char* value(const ::dobot_bringup::ServoJParamRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::ServoJParamRequest> should match
// service_traits::DataType< ::dobot_bringup::ServoJParam >
template<>
struct DataType< ::dobot_bringup::ServoJParamRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::ServoJParam >::value();
  }
  static const char* value(const ::dobot_bringup::ServoJParamRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_bringup::ServoJParamResponse> should match
// service_traits::MD5Sum< ::dobot_bringup::ServoJParam >
template<>
struct MD5Sum< ::dobot_bringup::ServoJParamResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::ServoJParam >::value();
  }
  static const char* value(const ::dobot_bringup::ServoJParamResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::ServoJParamResponse> should match
// service_traits::DataType< ::dobot_bringup::ServoJParam >
template<>
struct DataType< ::dobot_bringup::ServoJParamResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::ServoJParam >::value();
  }
  static const char* value(const ::dobot_bringup::ServoJParamResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_SERVOJPARAM_H
