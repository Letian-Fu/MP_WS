// Generated by gencpp from file dobot_bringup/ContinuesRequest.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_CONTINUESREQUEST_H
#define DOBOT_BRINGUP_MESSAGE_CONTINUESREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dobot_bringup
{
template <class ContainerAllocator>
struct ContinuesRequest_
{
  typedef ContinuesRequest_<ContainerAllocator> Type;

  ContinuesRequest_()
    {
    }
  ContinuesRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::dobot_bringup::ContinuesRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dobot_bringup::ContinuesRequest_<ContainerAllocator> const> ConstPtr;

}; // struct ContinuesRequest_

typedef ::dobot_bringup::ContinuesRequest_<std::allocator<void> > ContinuesRequest;

typedef boost::shared_ptr< ::dobot_bringup::ContinuesRequest > ContinuesRequestPtr;
typedef boost::shared_ptr< ::dobot_bringup::ContinuesRequest const> ContinuesRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dobot_bringup::ContinuesRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dobot_bringup::ContinuesRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace dobot_bringup

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dobot_bringup::ContinuesRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot_bringup::ContinuesRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_bringup::ContinuesRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_bringup::ContinuesRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_bringup::ContinuesRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_bringup::ContinuesRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dobot_bringup::ContinuesRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::dobot_bringup::ContinuesRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::dobot_bringup::ContinuesRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dobot_bringup/ContinuesRequest";
  }

  static const char* value(const ::dobot_bringup::ContinuesRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dobot_bringup::ContinuesRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::dobot_bringup::ContinuesRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dobot_bringup::ContinuesRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ContinuesRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dobot_bringup::ContinuesRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::dobot_bringup::ContinuesRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_CONTINUESREQUEST_H
