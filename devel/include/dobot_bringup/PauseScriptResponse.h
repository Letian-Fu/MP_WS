// Generated by gencpp from file dobot_bringup/PauseScriptResponse.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_PAUSESCRIPTRESPONSE_H
#define DOBOT_BRINGUP_MESSAGE_PAUSESCRIPTRESPONSE_H


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
struct PauseScriptResponse_
{
  typedef PauseScriptResponse_<ContainerAllocator> Type;

  PauseScriptResponse_()
    : res(0)  {
    }
  PauseScriptResponse_(const ContainerAllocator& _alloc)
    : res(0)  {
  (void)_alloc;
    }



   typedef int32_t _res_type;
  _res_type res;





  typedef boost::shared_ptr< ::dobot_bringup::PauseScriptResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dobot_bringup::PauseScriptResponse_<ContainerAllocator> const> ConstPtr;

}; // struct PauseScriptResponse_

typedef ::dobot_bringup::PauseScriptResponse_<std::allocator<void> > PauseScriptResponse;

typedef boost::shared_ptr< ::dobot_bringup::PauseScriptResponse > PauseScriptResponsePtr;
typedef boost::shared_ptr< ::dobot_bringup::PauseScriptResponse const> PauseScriptResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dobot_bringup::PauseScriptResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dobot_bringup::PauseScriptResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dobot_bringup::PauseScriptResponse_<ContainerAllocator1> & lhs, const ::dobot_bringup::PauseScriptResponse_<ContainerAllocator2> & rhs)
{
  return lhs.res == rhs.res;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dobot_bringup::PauseScriptResponse_<ContainerAllocator1> & lhs, const ::dobot_bringup::PauseScriptResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dobot_bringup

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dobot_bringup::PauseScriptResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot_bringup::PauseScriptResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_bringup::PauseScriptResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_bringup::PauseScriptResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_bringup::PauseScriptResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_bringup::PauseScriptResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dobot_bringup::PauseScriptResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ca16cfbd5443ad97f6cc7ffd6bb67292";
  }

  static const char* value(const ::dobot_bringup::PauseScriptResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xca16cfbd5443ad97ULL;
  static const uint64_t static_value2 = 0xf6cc7ffd6bb67292ULL;
};

template<class ContainerAllocator>
struct DataType< ::dobot_bringup::PauseScriptResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dobot_bringup/PauseScriptResponse";
  }

  static const char* value(const ::dobot_bringup::PauseScriptResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dobot_bringup::PauseScriptResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 res\n"
;
  }

  static const char* value(const ::dobot_bringup::PauseScriptResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dobot_bringup::PauseScriptResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.res);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PauseScriptResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dobot_bringup::PauseScriptResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dobot_bringup::PauseScriptResponse_<ContainerAllocator>& v)
  {
    s << indent << "res: ";
    Printer<int32_t>::stream(s, indent + "  ", v.res);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_PAUSESCRIPTRESPONSE_H
