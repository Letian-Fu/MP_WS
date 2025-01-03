// Generated by gencpp from file dobot_v4_bringup/EnableSafeSkinResponse.msg
// DO NOT EDIT!


#ifndef DOBOT_V4_BRINGUP_MESSAGE_ENABLESAFESKINRESPONSE_H
#define DOBOT_V4_BRINGUP_MESSAGE_ENABLESAFESKINRESPONSE_H


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
struct EnableSafeSkinResponse_
{
  typedef EnableSafeSkinResponse_<ContainerAllocator> Type;

  EnableSafeSkinResponse_()
    : res(0)  {
    }
  EnableSafeSkinResponse_(const ContainerAllocator& _alloc)
    : res(0)  {
  (void)_alloc;
    }



   typedef int32_t _res_type;
  _res_type res;





  typedef boost::shared_ptr< ::dobot_v4_bringup::EnableSafeSkinResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dobot_v4_bringup::EnableSafeSkinResponse_<ContainerAllocator> const> ConstPtr;

}; // struct EnableSafeSkinResponse_

typedef ::dobot_v4_bringup::EnableSafeSkinResponse_<std::allocator<void> > EnableSafeSkinResponse;

typedef boost::shared_ptr< ::dobot_v4_bringup::EnableSafeSkinResponse > EnableSafeSkinResponsePtr;
typedef boost::shared_ptr< ::dobot_v4_bringup::EnableSafeSkinResponse const> EnableSafeSkinResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dobot_v4_bringup::EnableSafeSkinResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dobot_v4_bringup::EnableSafeSkinResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dobot_v4_bringup::EnableSafeSkinResponse_<ContainerAllocator1> & lhs, const ::dobot_v4_bringup::EnableSafeSkinResponse_<ContainerAllocator2> & rhs)
{
  return lhs.res == rhs.res;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dobot_v4_bringup::EnableSafeSkinResponse_<ContainerAllocator1> & lhs, const ::dobot_v4_bringup::EnableSafeSkinResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dobot_v4_bringup

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dobot_v4_bringup::EnableSafeSkinResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot_v4_bringup::EnableSafeSkinResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_v4_bringup::EnableSafeSkinResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_v4_bringup::EnableSafeSkinResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_v4_bringup::EnableSafeSkinResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_v4_bringup::EnableSafeSkinResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dobot_v4_bringup::EnableSafeSkinResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ca16cfbd5443ad97f6cc7ffd6bb67292";
  }

  static const char* value(const ::dobot_v4_bringup::EnableSafeSkinResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xca16cfbd5443ad97ULL;
  static const uint64_t static_value2 = 0xf6cc7ffd6bb67292ULL;
};

template<class ContainerAllocator>
struct DataType< ::dobot_v4_bringup::EnableSafeSkinResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dobot_v4_bringup/EnableSafeSkinResponse";
  }

  static const char* value(const ::dobot_v4_bringup::EnableSafeSkinResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dobot_v4_bringup::EnableSafeSkinResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 res\n"
;
  }

  static const char* value(const ::dobot_v4_bringup::EnableSafeSkinResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dobot_v4_bringup::EnableSafeSkinResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.res);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct EnableSafeSkinResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dobot_v4_bringup::EnableSafeSkinResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dobot_v4_bringup::EnableSafeSkinResponse_<ContainerAllocator>& v)
  {
    s << indent << "res: ";
    Printer<int32_t>::stream(s, indent + "  ", v.res);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOBOT_V4_BRINGUP_MESSAGE_ENABLESAFESKINRESPONSE_H
