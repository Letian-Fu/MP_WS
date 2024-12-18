// Generated by gencpp from file dobot_v4_bringup/SetPostCollisionModeRequest.msg
// DO NOT EDIT!


#ifndef DOBOT_V4_BRINGUP_MESSAGE_SETPOSTCOLLISIONMODEREQUEST_H
#define DOBOT_V4_BRINGUP_MESSAGE_SETPOSTCOLLISIONMODEREQUEST_H


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
struct SetPostCollisionModeRequest_
{
  typedef SetPostCollisionModeRequest_<ContainerAllocator> Type;

  SetPostCollisionModeRequest_()
    : mode(0)  {
    }
  SetPostCollisionModeRequest_(const ContainerAllocator& _alloc)
    : mode(0)  {
  (void)_alloc;
    }



   typedef int32_t _mode_type;
  _mode_type mode;





  typedef boost::shared_ptr< ::dobot_v4_bringup::SetPostCollisionModeRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dobot_v4_bringup::SetPostCollisionModeRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetPostCollisionModeRequest_

typedef ::dobot_v4_bringup::SetPostCollisionModeRequest_<std::allocator<void> > SetPostCollisionModeRequest;

typedef boost::shared_ptr< ::dobot_v4_bringup::SetPostCollisionModeRequest > SetPostCollisionModeRequestPtr;
typedef boost::shared_ptr< ::dobot_v4_bringup::SetPostCollisionModeRequest const> SetPostCollisionModeRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dobot_v4_bringup::SetPostCollisionModeRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dobot_v4_bringup::SetPostCollisionModeRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dobot_v4_bringup::SetPostCollisionModeRequest_<ContainerAllocator1> & lhs, const ::dobot_v4_bringup::SetPostCollisionModeRequest_<ContainerAllocator2> & rhs)
{
  return lhs.mode == rhs.mode;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dobot_v4_bringup::SetPostCollisionModeRequest_<ContainerAllocator1> & lhs, const ::dobot_v4_bringup::SetPostCollisionModeRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dobot_v4_bringup

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dobot_v4_bringup::SetPostCollisionModeRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot_v4_bringup::SetPostCollisionModeRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_v4_bringup::SetPostCollisionModeRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_v4_bringup::SetPostCollisionModeRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_v4_bringup::SetPostCollisionModeRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_v4_bringup::SetPostCollisionModeRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dobot_v4_bringup::SetPostCollisionModeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff63f6ea3c3e9b7504b2cb3ee0a09d92";
  }

  static const char* value(const ::dobot_v4_bringup::SetPostCollisionModeRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xff63f6ea3c3e9b75ULL;
  static const uint64_t static_value2 = 0x04b2cb3ee0a09d92ULL;
};

template<class ContainerAllocator>
struct DataType< ::dobot_v4_bringup::SetPostCollisionModeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dobot_v4_bringup/SetPostCollisionModeRequest";
  }

  static const char* value(const ::dobot_v4_bringup::SetPostCollisionModeRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dobot_v4_bringup::SetPostCollisionModeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 mode\n"
;
  }

  static const char* value(const ::dobot_v4_bringup::SetPostCollisionModeRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dobot_v4_bringup::SetPostCollisionModeRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mode);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetPostCollisionModeRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dobot_v4_bringup::SetPostCollisionModeRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dobot_v4_bringup::SetPostCollisionModeRequest_<ContainerAllocator>& v)
  {
    s << indent << "mode: ";
    Printer<int32_t>::stream(s, indent + "  ", v.mode);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOBOT_V4_BRINGUP_MESSAGE_SETPOSTCOLLISIONMODEREQUEST_H
