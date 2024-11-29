// Generated by gencpp from file dobot_bringup/GetTraceStartPose.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_GETTRACESTARTPOSE_H
#define DOBOT_BRINGUP_MESSAGE_GETTRACESTARTPOSE_H

#include <ros/service_traits.h>


#include <dobot_bringup/GetTraceStartPoseRequest.h>
#include <dobot_bringup/GetTraceStartPoseResponse.h>


namespace dobot_bringup
{

struct GetTraceStartPose
{

typedef GetTraceStartPoseRequest Request;
typedef GetTraceStartPoseResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetTraceStartPose
} // namespace dobot_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_bringup::GetTraceStartPose > {
  static const char* value()
  {
    return "82df6582f8dcb92a8fcd749010d9e909";
  }

  static const char* value(const ::dobot_bringup::GetTraceStartPose&) { return value(); }
};

template<>
struct DataType< ::dobot_bringup::GetTraceStartPose > {
  static const char* value()
  {
    return "dobot_bringup/GetTraceStartPose";
  }

  static const char* value(const ::dobot_bringup::GetTraceStartPose&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_bringup::GetTraceStartPoseRequest> should match
// service_traits::MD5Sum< ::dobot_bringup::GetTraceStartPose >
template<>
struct MD5Sum< ::dobot_bringup::GetTraceStartPoseRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::GetTraceStartPose >::value();
  }
  static const char* value(const ::dobot_bringup::GetTraceStartPoseRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::GetTraceStartPoseRequest> should match
// service_traits::DataType< ::dobot_bringup::GetTraceStartPose >
template<>
struct DataType< ::dobot_bringup::GetTraceStartPoseRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::GetTraceStartPose >::value();
  }
  static const char* value(const ::dobot_bringup::GetTraceStartPoseRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_bringup::GetTraceStartPoseResponse> should match
// service_traits::MD5Sum< ::dobot_bringup::GetTraceStartPose >
template<>
struct MD5Sum< ::dobot_bringup::GetTraceStartPoseResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::GetTraceStartPose >::value();
  }
  static const char* value(const ::dobot_bringup::GetTraceStartPoseResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::GetTraceStartPoseResponse> should match
// service_traits::DataType< ::dobot_bringup::GetTraceStartPose >
template<>
struct DataType< ::dobot_bringup::GetTraceStartPoseResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::GetTraceStartPose >::value();
  }
  static const char* value(const ::dobot_bringup::GetTraceStartPoseResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_GETTRACESTARTPOSE_H