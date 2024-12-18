// Generated by gencpp from file dobot_bringup/WaitRequest.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_WAITREQUEST_H
#define DOBOT_BRINGUP_MESSAGE_WAITREQUEST_H


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
struct WaitRequest_
{
  typedef WaitRequest_<ContainerAllocator> Type;

  WaitRequest_()
    : time(0)  {
    }
  WaitRequest_(const ContainerAllocator& _alloc)
    : time(0)  {
  (void)_alloc;
    }



   typedef int32_t _time_type;
  _time_type time;





  typedef boost::shared_ptr< ::dobot_bringup::WaitRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dobot_bringup::WaitRequest_<ContainerAllocator> const> ConstPtr;

}; // struct WaitRequest_

typedef ::dobot_bringup::WaitRequest_<std::allocator<void> > WaitRequest;

typedef boost::shared_ptr< ::dobot_bringup::WaitRequest > WaitRequestPtr;
typedef boost::shared_ptr< ::dobot_bringup::WaitRequest const> WaitRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dobot_bringup::WaitRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dobot_bringup::WaitRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dobot_bringup::WaitRequest_<ContainerAllocator1> & lhs, const ::dobot_bringup::WaitRequest_<ContainerAllocator2> & rhs)
{
  return lhs.time == rhs.time;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dobot_bringup::WaitRequest_<ContainerAllocator1> & lhs, const ::dobot_bringup::WaitRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dobot_bringup

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dobot_bringup::WaitRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot_bringup::WaitRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_bringup::WaitRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_bringup::WaitRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_bringup::WaitRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_bringup::WaitRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dobot_bringup::WaitRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "86f5c1180cd9d94f9d1de0b211fa9fd2";
  }

  static const char* value(const ::dobot_bringup::WaitRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x86f5c1180cd9d94fULL;
  static const uint64_t static_value2 = 0x9d1de0b211fa9fd2ULL;
};

template<class ContainerAllocator>
struct DataType< ::dobot_bringup::WaitRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dobot_bringup/WaitRequest";
  }

  static const char* value(const ::dobot_bringup::WaitRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dobot_bringup::WaitRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 time\n"
;
  }

  static const char* value(const ::dobot_bringup::WaitRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dobot_bringup::WaitRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.time);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct WaitRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dobot_bringup::WaitRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dobot_bringup::WaitRequest_<ContainerAllocator>& v)
  {
    s << indent << "time: ";
    Printer<int32_t>::stream(s, indent + "  ", v.time);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_WAITREQUEST_H
