// Generated by gencpp from file dobot_bringup/ToolRequest.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_TOOLREQUEST_H
#define DOBOT_BRINGUP_MESSAGE_TOOLREQUEST_H


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
struct ToolRequest_
{
  typedef ToolRequest_<ContainerAllocator> Type;

  ToolRequest_()
    : index(0)  {
    }
  ToolRequest_(const ContainerAllocator& _alloc)
    : index(0)  {
  (void)_alloc;
    }



   typedef int32_t _index_type;
  _index_type index;





  typedef boost::shared_ptr< ::dobot_bringup::ToolRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dobot_bringup::ToolRequest_<ContainerAllocator> const> ConstPtr;

}; // struct ToolRequest_

typedef ::dobot_bringup::ToolRequest_<std::allocator<void> > ToolRequest;

typedef boost::shared_ptr< ::dobot_bringup::ToolRequest > ToolRequestPtr;
typedef boost::shared_ptr< ::dobot_bringup::ToolRequest const> ToolRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dobot_bringup::ToolRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dobot_bringup::ToolRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dobot_bringup::ToolRequest_<ContainerAllocator1> & lhs, const ::dobot_bringup::ToolRequest_<ContainerAllocator2> & rhs)
{
  return lhs.index == rhs.index;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dobot_bringup::ToolRequest_<ContainerAllocator1> & lhs, const ::dobot_bringup::ToolRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dobot_bringup

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dobot_bringup::ToolRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot_bringup::ToolRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_bringup::ToolRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_bringup::ToolRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_bringup::ToolRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_bringup::ToolRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dobot_bringup::ToolRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "483eea06cdc3e559421ee31116d0f3b9";
  }

  static const char* value(const ::dobot_bringup::ToolRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x483eea06cdc3e559ULL;
  static const uint64_t static_value2 = 0x421ee31116d0f3b9ULL;
};

template<class ContainerAllocator>
struct DataType< ::dobot_bringup::ToolRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dobot_bringup/ToolRequest";
  }

  static const char* value(const ::dobot_bringup::ToolRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dobot_bringup::ToolRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 index\n"
;
  }

  static const char* value(const ::dobot_bringup::ToolRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dobot_bringup::ToolRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.index);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ToolRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dobot_bringup::ToolRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dobot_bringup::ToolRequest_<ContainerAllocator>& v)
  {
    s << indent << "index: ";
    Printer<int32_t>::stream(s, indent + "  ", v.index);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_TOOLREQUEST_H
