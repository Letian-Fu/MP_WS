// Generated by gencpp from file dobot_v4_bringup/GetStartPoseRequest.msg
// DO NOT EDIT!


#ifndef DOBOT_V4_BRINGUP_MESSAGE_GETSTARTPOSEREQUEST_H
#define DOBOT_V4_BRINGUP_MESSAGE_GETSTARTPOSEREQUEST_H


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
struct GetStartPoseRequest_
{
  typedef GetStartPoseRequest_<ContainerAllocator> Type;

  GetStartPoseRequest_()
    : traceName()  {
    }
  GetStartPoseRequest_(const ContainerAllocator& _alloc)
    : traceName(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _traceName_type;
  _traceName_type traceName;





  typedef boost::shared_ptr< ::dobot_v4_bringup::GetStartPoseRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dobot_v4_bringup::GetStartPoseRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetStartPoseRequest_

typedef ::dobot_v4_bringup::GetStartPoseRequest_<std::allocator<void> > GetStartPoseRequest;

typedef boost::shared_ptr< ::dobot_v4_bringup::GetStartPoseRequest > GetStartPoseRequestPtr;
typedef boost::shared_ptr< ::dobot_v4_bringup::GetStartPoseRequest const> GetStartPoseRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dobot_v4_bringup::GetStartPoseRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dobot_v4_bringup::GetStartPoseRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dobot_v4_bringup::GetStartPoseRequest_<ContainerAllocator1> & lhs, const ::dobot_v4_bringup::GetStartPoseRequest_<ContainerAllocator2> & rhs)
{
  return lhs.traceName == rhs.traceName;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dobot_v4_bringup::GetStartPoseRequest_<ContainerAllocator1> & lhs, const ::dobot_v4_bringup::GetStartPoseRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dobot_v4_bringup

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dobot_v4_bringup::GetStartPoseRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot_v4_bringup::GetStartPoseRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_v4_bringup::GetStartPoseRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_v4_bringup::GetStartPoseRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_v4_bringup::GetStartPoseRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_v4_bringup::GetStartPoseRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dobot_v4_bringup::GetStartPoseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2509f3cb4d70960bf00043cebb91e3b7";
  }

  static const char* value(const ::dobot_v4_bringup::GetStartPoseRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2509f3cb4d70960bULL;
  static const uint64_t static_value2 = 0xf00043cebb91e3b7ULL;
};

template<class ContainerAllocator>
struct DataType< ::dobot_v4_bringup::GetStartPoseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dobot_v4_bringup/GetStartPoseRequest";
  }

  static const char* value(const ::dobot_v4_bringup::GetStartPoseRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dobot_v4_bringup::GetStartPoseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string traceName\n"
;
  }

  static const char* value(const ::dobot_v4_bringup::GetStartPoseRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dobot_v4_bringup::GetStartPoseRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.traceName);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetStartPoseRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dobot_v4_bringup::GetStartPoseRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dobot_v4_bringup::GetStartPoseRequest_<ContainerAllocator>& v)
  {
    s << indent << "traceName: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.traceName);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOBOT_V4_BRINGUP_MESSAGE_GETSTARTPOSEREQUEST_H
