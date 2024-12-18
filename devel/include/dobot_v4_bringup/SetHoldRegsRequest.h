// Generated by gencpp from file dobot_v4_bringup/SetHoldRegsRequest.msg
// DO NOT EDIT!


#ifndef DOBOT_V4_BRINGUP_MESSAGE_SETHOLDREGSREQUEST_H
#define DOBOT_V4_BRINGUP_MESSAGE_SETHOLDREGSREQUEST_H


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
struct SetHoldRegsRequest_
{
  typedef SetHoldRegsRequest_<ContainerAllocator> Type;

  SetHoldRegsRequest_()
    : index(0)
    , addr(0)
    , count(0)
    , valTab()
    , valType()  {
    }
  SetHoldRegsRequest_(const ContainerAllocator& _alloc)
    : index(0)
    , addr(0)
    , count(0)
    , valTab(_alloc)
    , valType(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _index_type;
  _index_type index;

   typedef int32_t _addr_type;
  _addr_type addr;

   typedef int32_t _count_type;
  _count_type count;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _valTab_type;
  _valTab_type valTab;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _valType_type;
  _valType_type valType;





  typedef boost::shared_ptr< ::dobot_v4_bringup::SetHoldRegsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dobot_v4_bringup::SetHoldRegsRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetHoldRegsRequest_

typedef ::dobot_v4_bringup::SetHoldRegsRequest_<std::allocator<void> > SetHoldRegsRequest;

typedef boost::shared_ptr< ::dobot_v4_bringup::SetHoldRegsRequest > SetHoldRegsRequestPtr;
typedef boost::shared_ptr< ::dobot_v4_bringup::SetHoldRegsRequest const> SetHoldRegsRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dobot_v4_bringup::SetHoldRegsRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dobot_v4_bringup::SetHoldRegsRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dobot_v4_bringup::SetHoldRegsRequest_<ContainerAllocator1> & lhs, const ::dobot_v4_bringup::SetHoldRegsRequest_<ContainerAllocator2> & rhs)
{
  return lhs.index == rhs.index &&
    lhs.addr == rhs.addr &&
    lhs.count == rhs.count &&
    lhs.valTab == rhs.valTab &&
    lhs.valType == rhs.valType;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dobot_v4_bringup::SetHoldRegsRequest_<ContainerAllocator1> & lhs, const ::dobot_v4_bringup::SetHoldRegsRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dobot_v4_bringup

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dobot_v4_bringup::SetHoldRegsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot_v4_bringup::SetHoldRegsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_v4_bringup::SetHoldRegsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_v4_bringup::SetHoldRegsRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_v4_bringup::SetHoldRegsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_v4_bringup::SetHoldRegsRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dobot_v4_bringup::SetHoldRegsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "aedfe9c7dbb997fe242018bbe793e544";
  }

  static const char* value(const ::dobot_v4_bringup::SetHoldRegsRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xaedfe9c7dbb997feULL;
  static const uint64_t static_value2 = 0x242018bbe793e544ULL;
};

template<class ContainerAllocator>
struct DataType< ::dobot_v4_bringup::SetHoldRegsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dobot_v4_bringup/SetHoldRegsRequest";
  }

  static const char* value(const ::dobot_v4_bringup::SetHoldRegsRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dobot_v4_bringup::SetHoldRegsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32    index\n"
"int32    addr\n"
"int32    count\n"
"string   valTab\n"
"string   valType\n"
;
  }

  static const char* value(const ::dobot_v4_bringup::SetHoldRegsRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dobot_v4_bringup::SetHoldRegsRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.index);
      stream.next(m.addr);
      stream.next(m.count);
      stream.next(m.valTab);
      stream.next(m.valType);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetHoldRegsRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dobot_v4_bringup::SetHoldRegsRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dobot_v4_bringup::SetHoldRegsRequest_<ContainerAllocator>& v)
  {
    s << indent << "index: ";
    Printer<int32_t>::stream(s, indent + "  ", v.index);
    s << indent << "addr: ";
    Printer<int32_t>::stream(s, indent + "  ", v.addr);
    s << indent << "count: ";
    Printer<int32_t>::stream(s, indent + "  ", v.count);
    s << indent << "valTab: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.valTab);
    s << indent << "valType: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.valType);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOBOT_V4_BRINGUP_MESSAGE_SETHOLDREGSREQUEST_H
