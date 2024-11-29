// Generated by gencpp from file rosdemo_v4/EnableRobot.msg
// DO NOT EDIT!


#ifndef ROSDEMO_V4_MESSAGE_ENABLEROBOT_H
#define ROSDEMO_V4_MESSAGE_ENABLEROBOT_H

#include <ros/service_traits.h>


#include <rosdemo_v4/EnableRobotRequest.h>
#include <rosdemo_v4/EnableRobotResponse.h>


namespace rosdemo_v4
{

struct EnableRobot
{

typedef EnableRobotRequest Request;
typedef EnableRobotResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct EnableRobot
} // namespace rosdemo_v4


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::rosdemo_v4::EnableRobot > {
  static const char* value()
  {
    return "ca16cfbd5443ad97f6cc7ffd6bb67292";
  }

  static const char* value(const ::rosdemo_v4::EnableRobot&) { return value(); }
};

template<>
struct DataType< ::rosdemo_v4::EnableRobot > {
  static const char* value()
  {
    return "rosdemo_v4/EnableRobot";
  }

  static const char* value(const ::rosdemo_v4::EnableRobot&) { return value(); }
};


// service_traits::MD5Sum< ::rosdemo_v4::EnableRobotRequest> should match
// service_traits::MD5Sum< ::rosdemo_v4::EnableRobot >
template<>
struct MD5Sum< ::rosdemo_v4::EnableRobotRequest>
{
  static const char* value()
  {
    return MD5Sum< ::rosdemo_v4::EnableRobot >::value();
  }
  static const char* value(const ::rosdemo_v4::EnableRobotRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::rosdemo_v4::EnableRobotRequest> should match
// service_traits::DataType< ::rosdemo_v4::EnableRobot >
template<>
struct DataType< ::rosdemo_v4::EnableRobotRequest>
{
  static const char* value()
  {
    return DataType< ::rosdemo_v4::EnableRobot >::value();
  }
  static const char* value(const ::rosdemo_v4::EnableRobotRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::rosdemo_v4::EnableRobotResponse> should match
// service_traits::MD5Sum< ::rosdemo_v4::EnableRobot >
template<>
struct MD5Sum< ::rosdemo_v4::EnableRobotResponse>
{
  static const char* value()
  {
    return MD5Sum< ::rosdemo_v4::EnableRobot >::value();
  }
  static const char* value(const ::rosdemo_v4::EnableRobotResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::rosdemo_v4::EnableRobotResponse> should match
// service_traits::DataType< ::rosdemo_v4::EnableRobot >
template<>
struct DataType< ::rosdemo_v4::EnableRobotResponse>
{
  static const char* value()
  {
    return DataType< ::rosdemo_v4::EnableRobot >::value();
  }
  static const char* value(const ::rosdemo_v4::EnableRobotResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ROSDEMO_V4_MESSAGE_ENABLEROBOT_H