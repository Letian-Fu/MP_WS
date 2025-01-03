// Generated by gencpp from file dobot_v4_bringup/CalcToolRequest.msg
// DO NOT EDIT!


#ifndef DOBOT_V4_BRINGUP_MESSAGE_CALCTOOLREQUEST_H
#define DOBOT_V4_BRINGUP_MESSAGE_CALCTOOLREQUEST_H


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
struct CalcToolRequest_
{
  typedef CalcToolRequest_<ContainerAllocator> Type;

  CalcToolRequest_()
    : index(0)
    , matrix(0)
    , offset()  {
    }
  CalcToolRequest_(const ContainerAllocator& _alloc)
    : index(0)
    , matrix(0)
    , offset(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _index_type;
  _index_type index;

   typedef int32_t _matrix_type;
  _matrix_type matrix;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _offset_type;
  _offset_type offset;





  typedef boost::shared_ptr< ::dobot_v4_bringup::CalcToolRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dobot_v4_bringup::CalcToolRequest_<ContainerAllocator> const> ConstPtr;

}; // struct CalcToolRequest_

typedef ::dobot_v4_bringup::CalcToolRequest_<std::allocator<void> > CalcToolRequest;

typedef boost::shared_ptr< ::dobot_v4_bringup::CalcToolRequest > CalcToolRequestPtr;
typedef boost::shared_ptr< ::dobot_v4_bringup::CalcToolRequest const> CalcToolRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dobot_v4_bringup::CalcToolRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dobot_v4_bringup::CalcToolRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dobot_v4_bringup::CalcToolRequest_<ContainerAllocator1> & lhs, const ::dobot_v4_bringup::CalcToolRequest_<ContainerAllocator2> & rhs)
{
  return lhs.index == rhs.index &&
    lhs.matrix == rhs.matrix &&
    lhs.offset == rhs.offset;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dobot_v4_bringup::CalcToolRequest_<ContainerAllocator1> & lhs, const ::dobot_v4_bringup::CalcToolRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dobot_v4_bringup

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dobot_v4_bringup::CalcToolRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot_v4_bringup::CalcToolRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_v4_bringup::CalcToolRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_v4_bringup::CalcToolRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_v4_bringup::CalcToolRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_v4_bringup::CalcToolRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dobot_v4_bringup::CalcToolRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b4d764c6d2c2cb958959b14894e6a4d8";
  }

  static const char* value(const ::dobot_v4_bringup::CalcToolRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb4d764c6d2c2cb95ULL;
  static const uint64_t static_value2 = 0x8959b14894e6a4d8ULL;
};

template<class ContainerAllocator>
struct DataType< ::dobot_v4_bringup::CalcToolRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dobot_v4_bringup/CalcToolRequest";
  }

  static const char* value(const ::dobot_v4_bringup::CalcToolRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dobot_v4_bringup::CalcToolRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32  index\n"
"int32  matrix\n"
"string offset\n"
;
  }

  static const char* value(const ::dobot_v4_bringup::CalcToolRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dobot_v4_bringup::CalcToolRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.index);
      stream.next(m.matrix);
      stream.next(m.offset);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CalcToolRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dobot_v4_bringup::CalcToolRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dobot_v4_bringup::CalcToolRequest_<ContainerAllocator>& v)
  {
    s << indent << "index: ";
    Printer<int32_t>::stream(s, indent + "  ", v.index);
    s << indent << "matrix: ";
    Printer<int32_t>::stream(s, indent + "  ", v.matrix);
    s << indent << "offset: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.offset);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOBOT_V4_BRINGUP_MESSAGE_CALCTOOLREQUEST_H
