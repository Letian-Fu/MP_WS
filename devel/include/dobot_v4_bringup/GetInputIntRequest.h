// Generated by gencpp from file dobot_v4_bringup/GetInputIntRequest.msg
// DO NOT EDIT!


#ifndef DOBOT_V4_BRINGUP_MESSAGE_GETINPUTINTREQUEST_H
#define DOBOT_V4_BRINGUP_MESSAGE_GETINPUTINTREQUEST_H


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
struct GetInputIntRequest_
{
  typedef GetInputIntRequest_<ContainerAllocator> Type;

  GetInputIntRequest_()
    : address(0)  {
    }
  GetInputIntRequest_(const ContainerAllocator& _alloc)
    : address(0)  {
  (void)_alloc;
    }



   typedef int32_t _address_type;
  _address_type address;





  typedef boost::shared_ptr< ::dobot_v4_bringup::GetInputIntRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dobot_v4_bringup::GetInputIntRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetInputIntRequest_

typedef ::dobot_v4_bringup::GetInputIntRequest_<std::allocator<void> > GetInputIntRequest;

typedef boost::shared_ptr< ::dobot_v4_bringup::GetInputIntRequest > GetInputIntRequestPtr;
typedef boost::shared_ptr< ::dobot_v4_bringup::GetInputIntRequest const> GetInputIntRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dobot_v4_bringup::GetInputIntRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dobot_v4_bringup::GetInputIntRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dobot_v4_bringup::GetInputIntRequest_<ContainerAllocator1> & lhs, const ::dobot_v4_bringup::GetInputIntRequest_<ContainerAllocator2> & rhs)
{
  return lhs.address == rhs.address;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dobot_v4_bringup::GetInputIntRequest_<ContainerAllocator1> & lhs, const ::dobot_v4_bringup::GetInputIntRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dobot_v4_bringup

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dobot_v4_bringup::GetInputIntRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot_v4_bringup::GetInputIntRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_v4_bringup::GetInputIntRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_v4_bringup::GetInputIntRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_v4_bringup::GetInputIntRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_v4_bringup::GetInputIntRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dobot_v4_bringup::GetInputIntRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dff3ccdc45e0333e4e8a2e8d7bdf4f6a";
  }

  static const char* value(const ::dobot_v4_bringup::GetInputIntRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xdff3ccdc45e0333eULL;
  static const uint64_t static_value2 = 0x4e8a2e8d7bdf4f6aULL;
};

template<class ContainerAllocator>
struct DataType< ::dobot_v4_bringup::GetInputIntRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dobot_v4_bringup/GetInputIntRequest";
  }

  static const char* value(const ::dobot_v4_bringup::GetInputIntRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dobot_v4_bringup::GetInputIntRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 address\n"
;
  }

  static const char* value(const ::dobot_v4_bringup::GetInputIntRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dobot_v4_bringup::GetInputIntRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.address);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetInputIntRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dobot_v4_bringup::GetInputIntRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dobot_v4_bringup::GetInputIntRequest_<ContainerAllocator>& v)
  {
    s << indent << "address: ";
    Printer<int32_t>::stream(s, indent + "  ", v.address);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOBOT_V4_BRINGUP_MESSAGE_GETINPUTINTREQUEST_H
