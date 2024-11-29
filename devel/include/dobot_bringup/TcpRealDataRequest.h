// Generated by gencpp from file dobot_bringup/TcpRealDataRequest.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_TCPREALDATAREQUEST_H
#define DOBOT_BRINGUP_MESSAGE_TCPREALDATAREQUEST_H


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
struct TcpRealDataRequest_
{
  typedef TcpRealDataRequest_<ContainerAllocator> Type;

  TcpRealDataRequest_()
    : index(0)
    , size(0)  {
    }
  TcpRealDataRequest_(const ContainerAllocator& _alloc)
    : index(0)
    , size(0)  {
  (void)_alloc;
    }



   typedef uint32_t _index_type;
  _index_type index;

   typedef uint32_t _size_type;
  _size_type size;





  typedef boost::shared_ptr< ::dobot_bringup::TcpRealDataRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dobot_bringup::TcpRealDataRequest_<ContainerAllocator> const> ConstPtr;

}; // struct TcpRealDataRequest_

typedef ::dobot_bringup::TcpRealDataRequest_<std::allocator<void> > TcpRealDataRequest;

typedef boost::shared_ptr< ::dobot_bringup::TcpRealDataRequest > TcpRealDataRequestPtr;
typedef boost::shared_ptr< ::dobot_bringup::TcpRealDataRequest const> TcpRealDataRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dobot_bringup::TcpRealDataRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dobot_bringup::TcpRealDataRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dobot_bringup::TcpRealDataRequest_<ContainerAllocator1> & lhs, const ::dobot_bringup::TcpRealDataRequest_<ContainerAllocator2> & rhs)
{
  return lhs.index == rhs.index &&
    lhs.size == rhs.size;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dobot_bringup::TcpRealDataRequest_<ContainerAllocator1> & lhs, const ::dobot_bringup::TcpRealDataRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dobot_bringup

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dobot_bringup::TcpRealDataRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot_bringup::TcpRealDataRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_bringup::TcpRealDataRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_bringup::TcpRealDataRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_bringup::TcpRealDataRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_bringup::TcpRealDataRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dobot_bringup::TcpRealDataRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "00ac1b2dfdd7eaac444ab500500930fd";
  }

  static const char* value(const ::dobot_bringup::TcpRealDataRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x00ac1b2dfdd7eaacULL;
  static const uint64_t static_value2 = 0x444ab500500930fdULL;
};

template<class ContainerAllocator>
struct DataType< ::dobot_bringup::TcpRealDataRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dobot_bringup/TcpRealDataRequest";
  }

  static const char* value(const ::dobot_bringup::TcpRealDataRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dobot_bringup::TcpRealDataRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 index\n"
"uint32 size\n"
;
  }

  static const char* value(const ::dobot_bringup::TcpRealDataRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dobot_bringup::TcpRealDataRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.index);
      stream.next(m.size);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TcpRealDataRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dobot_bringup::TcpRealDataRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dobot_bringup::TcpRealDataRequest_<ContainerAllocator>& v)
  {
    s << indent << "index: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.index);
    s << indent << "size: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.size);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_TCPREALDATAREQUEST_H