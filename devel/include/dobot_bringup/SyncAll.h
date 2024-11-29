// Generated by gencpp from file dobot_bringup/SyncAll.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_SYNCALL_H
#define DOBOT_BRINGUP_MESSAGE_SYNCALL_H

#include <ros/service_traits.h>


#include <dobot_bringup/SyncAllRequest.h>
#include <dobot_bringup/SyncAllResponse.h>


namespace dobot_bringup
{

struct SyncAll
{

typedef SyncAllRequest Request;
typedef SyncAllResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SyncAll
} // namespace dobot_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_bringup::SyncAll > {
  static const char* value()
  {
    return "ca16cfbd5443ad97f6cc7ffd6bb67292";
  }

  static const char* value(const ::dobot_bringup::SyncAll&) { return value(); }
};

template<>
struct DataType< ::dobot_bringup::SyncAll > {
  static const char* value()
  {
    return "dobot_bringup/SyncAll";
  }

  static const char* value(const ::dobot_bringup::SyncAll&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_bringup::SyncAllRequest> should match
// service_traits::MD5Sum< ::dobot_bringup::SyncAll >
template<>
struct MD5Sum< ::dobot_bringup::SyncAllRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::SyncAll >::value();
  }
  static const char* value(const ::dobot_bringup::SyncAllRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::SyncAllRequest> should match
// service_traits::DataType< ::dobot_bringup::SyncAll >
template<>
struct DataType< ::dobot_bringup::SyncAllRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::SyncAll >::value();
  }
  static const char* value(const ::dobot_bringup::SyncAllRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_bringup::SyncAllResponse> should match
// service_traits::MD5Sum< ::dobot_bringup::SyncAll >
template<>
struct MD5Sum< ::dobot_bringup::SyncAllResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::SyncAll >::value();
  }
  static const char* value(const ::dobot_bringup::SyncAllResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::SyncAllResponse> should match
// service_traits::DataType< ::dobot_bringup::SyncAll >
template<>
struct DataType< ::dobot_bringup::SyncAllResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::SyncAll >::value();
  }
  static const char* value(const ::dobot_bringup::SyncAllResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_SYNCALL_H