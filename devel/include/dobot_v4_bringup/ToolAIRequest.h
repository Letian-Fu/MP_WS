// Generated by gencpp from file dobot_v4_bringup/ToolAIRequest.msg
// DO NOT EDIT!


#ifndef DOBOT_V4_BRINGUP_MESSAGE_TOOLAIREQUEST_H
#define DOBOT_V4_BRINGUP_MESSAGE_TOOLAIREQUEST_H


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
struct ToolAIRequest_
{
  typedef ToolAIRequest_<ContainerAllocator> Type;

  ToolAIRequest_()
    : index(0)  {
    }
  ToolAIRequest_(const ContainerAllocator& _alloc)
    : index(0)  {
  (void)_alloc;
    }



   typedef int32_t _index_type;
  _index_type index;





  typedef boost::shared_ptr< ::dobot_v4_bringup::ToolAIRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dobot_v4_bringup::ToolAIRequest_<ContainerAllocator> const> ConstPtr;

}; // struct ToolAIRequest_

typedef ::dobot_v4_bringup::ToolAIRequest_<std::allocator<void> > ToolAIRequest;

typedef boost::shared_ptr< ::dobot_v4_bringup::ToolAIRequest > ToolAIRequestPtr;
typedef boost::shared_ptr< ::dobot_v4_bringup::ToolAIRequest const> ToolAIRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dobot_v4_bringup::ToolAIRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dobot_v4_bringup::ToolAIRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dobot_v4_bringup::ToolAIRequest_<ContainerAllocator1> & lhs, const ::dobot_v4_bringup::ToolAIRequest_<ContainerAllocator2> & rhs)
{
  return lhs.index == rhs.index;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dobot_v4_bringup::ToolAIRequest_<ContainerAllocator1> & lhs, const ::dobot_v4_bringup::ToolAIRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dobot_v4_bringup

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dobot_v4_bringup::ToolAIRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot_v4_bringup::ToolAIRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_v4_bringup::ToolAIRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_v4_bringup::ToolAIRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_v4_bringup::ToolAIRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_v4_bringup::ToolAIRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dobot_v4_bringup::ToolAIRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "483eea06cdc3e559421ee31116d0f3b9";
  }

  static const char* value(const ::dobot_v4_bringup::ToolAIRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x483eea06cdc3e559ULL;
  static const uint64_t static_value2 = 0x421ee31116d0f3b9ULL;
};

template<class ContainerAllocator>
struct DataType< ::dobot_v4_bringup::ToolAIRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dobot_v4_bringup/ToolAIRequest";
  }

  static const char* value(const ::dobot_v4_bringup::ToolAIRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dobot_v4_bringup::ToolAIRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 index\n"
;
  }

  static const char* value(const ::dobot_v4_bringup::ToolAIRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dobot_v4_bringup::ToolAIRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.index);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ToolAIRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dobot_v4_bringup::ToolAIRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dobot_v4_bringup::ToolAIRequest_<ContainerAllocator>& v)
  {
    s << indent << "index: ";
    Printer<int32_t>::stream(s, indent + "  ", v.index);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOBOT_V4_BRINGUP_MESSAGE_TOOLAIREQUEST_H
