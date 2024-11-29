// Generated by gencpp from file dobot_v4_bringup/RobotStatus.msg
// DO NOT EDIT!


#ifndef DOBOT_V4_BRINGUP_MESSAGE_ROBOTSTATUS_H
#define DOBOT_V4_BRINGUP_MESSAGE_ROBOTSTATUS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dobot_v4_bringup
{
template <class ContainerAllocator>
struct RobotStatus_
{
  typedef RobotStatus_<ContainerAllocator> Type;

  RobotStatus_()
    : is_enable(false)
    , is_connected(false)  {
    }
  RobotStatus_(const ContainerAllocator& _alloc)
    : is_enable(false)
    , is_connected(false)  {
  (void)_alloc;
    }



   typedef uint8_t _is_enable_type;
  _is_enable_type is_enable;

   typedef uint8_t _is_connected_type;
  _is_connected_type is_connected;





  typedef boost::shared_ptr< ::dobot_v4_bringup::RobotStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dobot_v4_bringup::RobotStatus_<ContainerAllocator> const> ConstPtr;

}; // struct RobotStatus_

typedef ::dobot_v4_bringup::RobotStatus_<std::allocator<void> > RobotStatus;

typedef boost::shared_ptr< ::dobot_v4_bringup::RobotStatus > RobotStatusPtr;
typedef boost::shared_ptr< ::dobot_v4_bringup::RobotStatus const> RobotStatusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dobot_v4_bringup::RobotStatus_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dobot_v4_bringup::RobotStatus_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dobot_v4_bringup::RobotStatus_<ContainerAllocator1> & lhs, const ::dobot_v4_bringup::RobotStatus_<ContainerAllocator2> & rhs)
{
  return lhs.is_enable == rhs.is_enable &&
    lhs.is_connected == rhs.is_connected;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dobot_v4_bringup::RobotStatus_<ContainerAllocator1> & lhs, const ::dobot_v4_bringup::RobotStatus_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dobot_v4_bringup

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dobot_v4_bringup::RobotStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot_v4_bringup::RobotStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_v4_bringup::RobotStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_v4_bringup::RobotStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_v4_bringup::RobotStatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_v4_bringup::RobotStatus_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dobot_v4_bringup::RobotStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "03c2e6f26397d687a7b9b9208ea5a445";
  }

  static const char* value(const ::dobot_v4_bringup::RobotStatus_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x03c2e6f26397d687ULL;
  static const uint64_t static_value2 = 0xa7b9b9208ea5a445ULL;
};

template<class ContainerAllocator>
struct DataType< ::dobot_v4_bringup::RobotStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dobot_v4_bringup/RobotStatus";
  }

  static const char* value(const ::dobot_v4_bringup::RobotStatus_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dobot_v4_bringup::RobotStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool is_enable\n"
"bool is_connected\n"
;
  }

  static const char* value(const ::dobot_v4_bringup::RobotStatus_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dobot_v4_bringup::RobotStatus_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.is_enable);
      stream.next(m.is_connected);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RobotStatus_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dobot_v4_bringup::RobotStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dobot_v4_bringup::RobotStatus_<ContainerAllocator>& v)
  {
    s << indent << "is_enable: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_enable);
    s << indent << "is_connected: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_connected);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOBOT_V4_BRINGUP_MESSAGE_ROBOTSTATUS_H