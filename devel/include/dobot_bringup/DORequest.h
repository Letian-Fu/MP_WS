// Generated by gencpp from file dobot_bringup/DORequest.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_DOREQUEST_H
#define DOBOT_BRINGUP_MESSAGE_DOREQUEST_H


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
struct DORequest_
{
  typedef DORequest_<ContainerAllocator> Type;

  DORequest_()
    : index(0)
    , status(0)  {
    }
  DORequest_(const ContainerAllocator& _alloc)
    : index(0)
    , status(0)  {
  (void)_alloc;
    }



   typedef int32_t _index_type;
  _index_type index;

   typedef int32_t _status_type;
  _status_type status;





  typedef boost::shared_ptr< ::dobot_bringup::DORequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dobot_bringup::DORequest_<ContainerAllocator> const> ConstPtr;

}; // struct DORequest_

typedef ::dobot_bringup::DORequest_<std::allocator<void> > DORequest;

typedef boost::shared_ptr< ::dobot_bringup::DORequest > DORequestPtr;
typedef boost::shared_ptr< ::dobot_bringup::DORequest const> DORequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dobot_bringup::DORequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dobot_bringup::DORequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dobot_bringup::DORequest_<ContainerAllocator1> & lhs, const ::dobot_bringup::DORequest_<ContainerAllocator2> & rhs)
{
  return lhs.index == rhs.index &&
    lhs.status == rhs.status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dobot_bringup::DORequest_<ContainerAllocator1> & lhs, const ::dobot_bringup::DORequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dobot_bringup

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dobot_bringup::DORequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot_bringup::DORequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_bringup::DORequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_bringup::DORequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_bringup::DORequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_bringup::DORequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dobot_bringup::DORequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6007ef6166fd1338f216c1446a6cbad9";
  }

  static const char* value(const ::dobot_bringup::DORequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6007ef6166fd1338ULL;
  static const uint64_t static_value2 = 0xf216c1446a6cbad9ULL;
};

template<class ContainerAllocator>
struct DataType< ::dobot_bringup::DORequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dobot_bringup/DORequest";
  }

  static const char* value(const ::dobot_bringup::DORequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dobot_bringup::DORequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# r --> 1 - 24\n"
"int32 index\n"
"int32 status\n"
;
  }

  static const char* value(const ::dobot_bringup::DORequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dobot_bringup::DORequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.index);
      stream.next(m.status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DORequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dobot_bringup::DORequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dobot_bringup::DORequest_<ContainerAllocator>& v)
  {
    s << indent << "index: ";
    Printer<int32_t>::stream(s, indent + "  ", v.index);
    s << indent << "status: ";
    Printer<int32_t>::stream(s, indent + "  ", v.status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_DOREQUEST_H
