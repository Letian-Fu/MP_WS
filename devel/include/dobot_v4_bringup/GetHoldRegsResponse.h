// Generated by gencpp from file dobot_v4_bringup/GetHoldRegsResponse.msg
// DO NOT EDIT!


#ifndef DOBOT_V4_BRINGUP_MESSAGE_GETHOLDREGSRESPONSE_H
#define DOBOT_V4_BRINGUP_MESSAGE_GETHOLDREGSRESPONSE_H


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
struct GetHoldRegsResponse_
{
  typedef GetHoldRegsResponse_<ContainerAllocator> Type;

  GetHoldRegsResponse_()
    : res(0)  {
    }
  GetHoldRegsResponse_(const ContainerAllocator& _alloc)
    : res(0)  {
  (void)_alloc;
    }



   typedef int32_t _res_type;
  _res_type res;





  typedef boost::shared_ptr< ::dobot_v4_bringup::GetHoldRegsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dobot_v4_bringup::GetHoldRegsResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetHoldRegsResponse_

typedef ::dobot_v4_bringup::GetHoldRegsResponse_<std::allocator<void> > GetHoldRegsResponse;

typedef boost::shared_ptr< ::dobot_v4_bringup::GetHoldRegsResponse > GetHoldRegsResponsePtr;
typedef boost::shared_ptr< ::dobot_v4_bringup::GetHoldRegsResponse const> GetHoldRegsResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dobot_v4_bringup::GetHoldRegsResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dobot_v4_bringup::GetHoldRegsResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dobot_v4_bringup::GetHoldRegsResponse_<ContainerAllocator1> & lhs, const ::dobot_v4_bringup::GetHoldRegsResponse_<ContainerAllocator2> & rhs)
{
  return lhs.res == rhs.res;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dobot_v4_bringup::GetHoldRegsResponse_<ContainerAllocator1> & lhs, const ::dobot_v4_bringup::GetHoldRegsResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dobot_v4_bringup

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dobot_v4_bringup::GetHoldRegsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot_v4_bringup::GetHoldRegsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_v4_bringup::GetHoldRegsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_v4_bringup::GetHoldRegsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_v4_bringup::GetHoldRegsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_v4_bringup::GetHoldRegsResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dobot_v4_bringup::GetHoldRegsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ca16cfbd5443ad97f6cc7ffd6bb67292";
  }

  static const char* value(const ::dobot_v4_bringup::GetHoldRegsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xca16cfbd5443ad97ULL;
  static const uint64_t static_value2 = 0xf6cc7ffd6bb67292ULL;
};

template<class ContainerAllocator>
struct DataType< ::dobot_v4_bringup::GetHoldRegsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dobot_v4_bringup/GetHoldRegsResponse";
  }

  static const char* value(const ::dobot_v4_bringup::GetHoldRegsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dobot_v4_bringup::GetHoldRegsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 res\n"
;
  }

  static const char* value(const ::dobot_v4_bringup::GetHoldRegsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dobot_v4_bringup::GetHoldRegsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.res);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetHoldRegsResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dobot_v4_bringup::GetHoldRegsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dobot_v4_bringup::GetHoldRegsResponse_<ContainerAllocator>& v)
  {
    s << indent << "res: ";
    Printer<int32_t>::stream(s, indent + "  ", v.res);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOBOT_V4_BRINGUP_MESSAGE_GETHOLDREGSRESPONSE_H
