// Generated by gencpp from file dobot_bringup/AOExecuteRequest.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_AOEXECUTEREQUEST_H
#define DOBOT_BRINGUP_MESSAGE_AOEXECUTEREQUEST_H


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
struct AOExecuteRequest_
{
  typedef AOExecuteRequest_<ContainerAllocator> Type;

  AOExecuteRequest_()
    : index(0)
    , value(0)  {
    }
  AOExecuteRequest_(const ContainerAllocator& _alloc)
    : index(0)
    , value(0)  {
  (void)_alloc;
    }



   typedef int32_t _index_type;
  _index_type index;

   typedef int32_t _value_type;
  _value_type value;





  typedef boost::shared_ptr< ::dobot_bringup::AOExecuteRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dobot_bringup::AOExecuteRequest_<ContainerAllocator> const> ConstPtr;

}; // struct AOExecuteRequest_

typedef ::dobot_bringup::AOExecuteRequest_<std::allocator<void> > AOExecuteRequest;

typedef boost::shared_ptr< ::dobot_bringup::AOExecuteRequest > AOExecuteRequestPtr;
typedef boost::shared_ptr< ::dobot_bringup::AOExecuteRequest const> AOExecuteRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dobot_bringup::AOExecuteRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dobot_bringup::AOExecuteRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dobot_bringup::AOExecuteRequest_<ContainerAllocator1> & lhs, const ::dobot_bringup::AOExecuteRequest_<ContainerAllocator2> & rhs)
{
  return lhs.index == rhs.index &&
    lhs.value == rhs.value;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dobot_bringup::AOExecuteRequest_<ContainerAllocator1> & lhs, const ::dobot_bringup::AOExecuteRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dobot_bringup

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dobot_bringup::AOExecuteRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot_bringup::AOExecuteRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_bringup::AOExecuteRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_bringup::AOExecuteRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_bringup::AOExecuteRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_bringup::AOExecuteRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dobot_bringup::AOExecuteRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff3a2288c5d88aa33bf422f4f0c87960";
  }

  static const char* value(const ::dobot_bringup::AOExecuteRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xff3a2288c5d88aa3ULL;
  static const uint64_t static_value2 = 0x3bf422f4f0c87960ULL;
};

template<class ContainerAllocator>
struct DataType< ::dobot_bringup::AOExecuteRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dobot_bringup/AOExecuteRequest";
  }

  static const char* value(const ::dobot_bringup::AOExecuteRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dobot_bringup::AOExecuteRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# index --> 1 - 2\n"
"# value --> 0 - 10\n"
"int32 index\n"
"int32 value\n"
;
  }

  static const char* value(const ::dobot_bringup::AOExecuteRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dobot_bringup::AOExecuteRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.index);
      stream.next(m.value);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AOExecuteRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dobot_bringup::AOExecuteRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dobot_bringup::AOExecuteRequest_<ContainerAllocator>& v)
  {
    s << indent << "index: ";
    Printer<int32_t>::stream(s, indent + "  ", v.index);
    s << indent << "value: ";
    Printer<int32_t>::stream(s, indent + "  ", v.value);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_AOEXECUTEREQUEST_H
