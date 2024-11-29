// Generated by gencpp from file dobot_v4_bringup/RunScriptRequest.msg
// DO NOT EDIT!


#ifndef DOBOT_V4_BRINGUP_MESSAGE_RUNSCRIPTREQUEST_H
#define DOBOT_V4_BRINGUP_MESSAGE_RUNSCRIPTREQUEST_H


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
struct RunScriptRequest_
{
  typedef RunScriptRequest_<ContainerAllocator> Type;

  RunScriptRequest_()
    : projectName()  {
    }
  RunScriptRequest_(const ContainerAllocator& _alloc)
    : projectName(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _projectName_type;
  _projectName_type projectName;





  typedef boost::shared_ptr< ::dobot_v4_bringup::RunScriptRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dobot_v4_bringup::RunScriptRequest_<ContainerAllocator> const> ConstPtr;

}; // struct RunScriptRequest_

typedef ::dobot_v4_bringup::RunScriptRequest_<std::allocator<void> > RunScriptRequest;

typedef boost::shared_ptr< ::dobot_v4_bringup::RunScriptRequest > RunScriptRequestPtr;
typedef boost::shared_ptr< ::dobot_v4_bringup::RunScriptRequest const> RunScriptRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dobot_v4_bringup::RunScriptRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dobot_v4_bringup::RunScriptRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dobot_v4_bringup::RunScriptRequest_<ContainerAllocator1> & lhs, const ::dobot_v4_bringup::RunScriptRequest_<ContainerAllocator2> & rhs)
{
  return lhs.projectName == rhs.projectName;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dobot_v4_bringup::RunScriptRequest_<ContainerAllocator1> & lhs, const ::dobot_v4_bringup::RunScriptRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dobot_v4_bringup

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dobot_v4_bringup::RunScriptRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot_v4_bringup::RunScriptRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_v4_bringup::RunScriptRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_v4_bringup::RunScriptRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_v4_bringup::RunScriptRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_v4_bringup::RunScriptRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dobot_v4_bringup::RunScriptRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d1c1e6a53ce92bbb5ea20e4f493dfd79";
  }

  static const char* value(const ::dobot_v4_bringup::RunScriptRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd1c1e6a53ce92bbbULL;
  static const uint64_t static_value2 = 0x5ea20e4f493dfd79ULL;
};

template<class ContainerAllocator>
struct DataType< ::dobot_v4_bringup::RunScriptRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dobot_v4_bringup/RunScriptRequest";
  }

  static const char* value(const ::dobot_v4_bringup::RunScriptRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dobot_v4_bringup::RunScriptRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string projectName\n"
;
  }

  static const char* value(const ::dobot_v4_bringup::RunScriptRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dobot_v4_bringup::RunScriptRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.projectName);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RunScriptRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dobot_v4_bringup::RunScriptRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dobot_v4_bringup::RunScriptRequest_<ContainerAllocator>& v)
  {
    s << indent << "projectName: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.projectName);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOBOT_V4_BRINGUP_MESSAGE_RUNSCRIPTREQUEST_H