// Generated by gencpp from file dobot_v4_bringup/ToolDORequest.msg
// DO NOT EDIT!


#ifndef DOBOT_V4_BRINGUP_MESSAGE_TOOLDOREQUEST_H
#define DOBOT_V4_BRINGUP_MESSAGE_TOOLDOREQUEST_H


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
struct ToolDORequest_
{
  typedef ToolDORequest_<ContainerAllocator> Type;

  ToolDORequest_()
    : index(0)
    , status(0)  {
    }
  ToolDORequest_(const ContainerAllocator& _alloc)
    : index(0)
    , status(0)  {
  (void)_alloc;
    }



   typedef int32_t _index_type;
  _index_type index;

   typedef int32_t _status_type;
  _status_type status;





  typedef boost::shared_ptr< ::dobot_v4_bringup::ToolDORequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dobot_v4_bringup::ToolDORequest_<ContainerAllocator> const> ConstPtr;

}; // struct ToolDORequest_

typedef ::dobot_v4_bringup::ToolDORequest_<std::allocator<void> > ToolDORequest;

typedef boost::shared_ptr< ::dobot_v4_bringup::ToolDORequest > ToolDORequestPtr;
typedef boost::shared_ptr< ::dobot_v4_bringup::ToolDORequest const> ToolDORequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dobot_v4_bringup::ToolDORequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dobot_v4_bringup::ToolDORequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dobot_v4_bringup::ToolDORequest_<ContainerAllocator1> & lhs, const ::dobot_v4_bringup::ToolDORequest_<ContainerAllocator2> & rhs)
{
  return lhs.index == rhs.index &&
    lhs.status == rhs.status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dobot_v4_bringup::ToolDORequest_<ContainerAllocator1> & lhs, const ::dobot_v4_bringup::ToolDORequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dobot_v4_bringup

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dobot_v4_bringup::ToolDORequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot_v4_bringup::ToolDORequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_v4_bringup::ToolDORequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_v4_bringup::ToolDORequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_v4_bringup::ToolDORequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_v4_bringup::ToolDORequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dobot_v4_bringup::ToolDORequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6007ef6166fd1338f216c1446a6cbad9";
  }

  static const char* value(const ::dobot_v4_bringup::ToolDORequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6007ef6166fd1338ULL;
  static const uint64_t static_value2 = 0xf216c1446a6cbad9ULL;
};

template<class ContainerAllocator>
struct DataType< ::dobot_v4_bringup::ToolDORequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dobot_v4_bringup/ToolDORequest";
  }

  static const char* value(const ::dobot_v4_bringup::ToolDORequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dobot_v4_bringup::ToolDORequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 index\n"
"int32 status\n"
;
  }

  static const char* value(const ::dobot_v4_bringup::ToolDORequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dobot_v4_bringup::ToolDORequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.index);
      stream.next(m.status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ToolDORequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dobot_v4_bringup::ToolDORequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dobot_v4_bringup::ToolDORequest_<ContainerAllocator>& v)
  {
    s << indent << "index: ";
    Printer<int32_t>::stream(s, indent + "  ", v.index);
    s << indent << "status: ";
    Printer<int32_t>::stream(s, indent + "  ", v.status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOBOT_V4_BRINGUP_MESSAGE_TOOLDOREQUEST_H
