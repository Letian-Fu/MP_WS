// Generated by gencpp from file rosdemo_v3/DisableRobotResponse.msg
// DO NOT EDIT!


#ifndef ROSDEMO_V3_MESSAGE_DISABLEROBOTRESPONSE_H
#define ROSDEMO_V3_MESSAGE_DISABLEROBOTRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rosdemo_v3
{
template <class ContainerAllocator>
struct DisableRobotResponse_
{
  typedef DisableRobotResponse_<ContainerAllocator> Type;

  DisableRobotResponse_()
    : res(0)  {
    }
  DisableRobotResponse_(const ContainerAllocator& _alloc)
    : res(0)  {
  (void)_alloc;
    }



   typedef int32_t _res_type;
  _res_type res;





  typedef boost::shared_ptr< ::rosdemo_v3::DisableRobotResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rosdemo_v3::DisableRobotResponse_<ContainerAllocator> const> ConstPtr;

}; // struct DisableRobotResponse_

typedef ::rosdemo_v3::DisableRobotResponse_<std::allocator<void> > DisableRobotResponse;

typedef boost::shared_ptr< ::rosdemo_v3::DisableRobotResponse > DisableRobotResponsePtr;
typedef boost::shared_ptr< ::rosdemo_v3::DisableRobotResponse const> DisableRobotResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rosdemo_v3::DisableRobotResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rosdemo_v3::DisableRobotResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::rosdemo_v3::DisableRobotResponse_<ContainerAllocator1> & lhs, const ::rosdemo_v3::DisableRobotResponse_<ContainerAllocator2> & rhs)
{
  return lhs.res == rhs.res;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::rosdemo_v3::DisableRobotResponse_<ContainerAllocator1> & lhs, const ::rosdemo_v3::DisableRobotResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace rosdemo_v3

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::rosdemo_v3::DisableRobotResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rosdemo_v3::DisableRobotResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rosdemo_v3::DisableRobotResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rosdemo_v3::DisableRobotResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosdemo_v3::DisableRobotResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosdemo_v3::DisableRobotResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rosdemo_v3::DisableRobotResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ca16cfbd5443ad97f6cc7ffd6bb67292";
  }

  static const char* value(const ::rosdemo_v3::DisableRobotResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xca16cfbd5443ad97ULL;
  static const uint64_t static_value2 = 0xf6cc7ffd6bb67292ULL;
};

template<class ContainerAllocator>
struct DataType< ::rosdemo_v3::DisableRobotResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rosdemo_v3/DisableRobotResponse";
  }

  static const char* value(const ::rosdemo_v3::DisableRobotResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rosdemo_v3::DisableRobotResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 res\n"
;
  }

  static const char* value(const ::rosdemo_v3::DisableRobotResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rosdemo_v3::DisableRobotResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.res);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DisableRobotResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rosdemo_v3::DisableRobotResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rosdemo_v3::DisableRobotResponse_<ContainerAllocator>& v)
  {
    s << indent << "res: ";
    Printer<int32_t>::stream(s, indent + "  ", v.res);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSDEMO_V3_MESSAGE_DISABLEROBOTRESPONSE_H
