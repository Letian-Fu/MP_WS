// Generated by gencpp from file dobot_bringup/CPRequest.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_CPREQUEST_H
#define DOBOT_BRINGUP_MESSAGE_CPREQUEST_H


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
struct CPRequest_
{
  typedef CPRequest_<ContainerAllocator> Type;

  CPRequest_()
    : r(0)  {
    }
  CPRequest_(const ContainerAllocator& _alloc)
    : r(0)  {
  (void)_alloc;
    }



   typedef int32_t _r_type;
  _r_type r;





  typedef boost::shared_ptr< ::dobot_bringup::CPRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dobot_bringup::CPRequest_<ContainerAllocator> const> ConstPtr;

}; // struct CPRequest_

typedef ::dobot_bringup::CPRequest_<std::allocator<void> > CPRequest;

typedef boost::shared_ptr< ::dobot_bringup::CPRequest > CPRequestPtr;
typedef boost::shared_ptr< ::dobot_bringup::CPRequest const> CPRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dobot_bringup::CPRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dobot_bringup::CPRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dobot_bringup::CPRequest_<ContainerAllocator1> & lhs, const ::dobot_bringup::CPRequest_<ContainerAllocator2> & rhs)
{
  return lhs.r == rhs.r;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dobot_bringup::CPRequest_<ContainerAllocator1> & lhs, const ::dobot_bringup::CPRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dobot_bringup

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dobot_bringup::CPRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot_bringup::CPRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_bringup::CPRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_bringup::CPRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_bringup::CPRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_bringup::CPRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dobot_bringup::CPRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "aa7eba76ae20db2204a8f1d4b9816c23";
  }

  static const char* value(const ::dobot_bringup::CPRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xaa7eba76ae20db22ULL;
  static const uint64_t static_value2 = 0x04a8f1d4b9816c23ULL;
};

template<class ContainerAllocator>
struct DataType< ::dobot_bringup::CPRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dobot_bringup/CPRequest";
  }

  static const char* value(const ::dobot_bringup::CPRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dobot_bringup::CPRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# r --> 1 - 100\n"
"int32 r\n"
;
  }

  static const char* value(const ::dobot_bringup::CPRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dobot_bringup::CPRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.r);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CPRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dobot_bringup::CPRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dobot_bringup::CPRequest_<ContainerAllocator>& v)
  {
    s << indent << "r: ";
    Printer<int32_t>::stream(s, indent + "  ", v.r);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_CPREQUEST_H
