// Generated by gencpp from file dobot_bringup/MovLIORequest.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_MOVLIOREQUEST_H
#define DOBOT_BRINGUP_MESSAGE_MOVLIOREQUEST_H


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
struct MovLIORequest_
{
  typedef MovLIORequest_<ContainerAllocator> Type;

  MovLIORequest_()
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , rx(0.0)
    , ry(0.0)
    , rz(0.0)
    , paramValue()  {
    }
  MovLIORequest_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , rx(0.0)
    , ry(0.0)
    , rz(0.0)
    , paramValue(_alloc)  {
  (void)_alloc;
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _z_type;
  _z_type z;

   typedef double _rx_type;
  _rx_type rx;

   typedef double _ry_type;
  _ry_type ry;

   typedef double _rz_type;
  _rz_type rz;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> _paramValue_type;
  _paramValue_type paramValue;





  typedef boost::shared_ptr< ::dobot_bringup::MovLIORequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dobot_bringup::MovLIORequest_<ContainerAllocator> const> ConstPtr;

}; // struct MovLIORequest_

typedef ::dobot_bringup::MovLIORequest_<std::allocator<void> > MovLIORequest;

typedef boost::shared_ptr< ::dobot_bringup::MovLIORequest > MovLIORequestPtr;
typedef boost::shared_ptr< ::dobot_bringup::MovLIORequest const> MovLIORequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dobot_bringup::MovLIORequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dobot_bringup::MovLIORequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dobot_bringup::MovLIORequest_<ContainerAllocator1> & lhs, const ::dobot_bringup::MovLIORequest_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z &&
    lhs.rx == rhs.rx &&
    lhs.ry == rhs.ry &&
    lhs.rz == rhs.rz &&
    lhs.paramValue == rhs.paramValue;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dobot_bringup::MovLIORequest_<ContainerAllocator1> & lhs, const ::dobot_bringup::MovLIORequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dobot_bringup

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dobot_bringup::MovLIORequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot_bringup::MovLIORequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_bringup::MovLIORequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_bringup::MovLIORequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_bringup::MovLIORequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_bringup::MovLIORequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dobot_bringup::MovLIORequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d29fefa56bba8afdde2ed3c880d7a985";
  }

  static const char* value(const ::dobot_bringup::MovLIORequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd29fefa56bba8afdULL;
  static const uint64_t static_value2 = 0xde2ed3c880d7a985ULL;
};

template<class ContainerAllocator>
struct DataType< ::dobot_bringup::MovLIORequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dobot_bringup/MovLIORequest";
  }

  static const char* value(const ::dobot_bringup::MovLIORequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dobot_bringup::MovLIORequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 rx\n"
"float64 ry\n"
"float64 rz\n"
"string[] paramValue\n"
;
  }

  static const char* value(const ::dobot_bringup::MovLIORequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dobot_bringup::MovLIORequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.rx);
      stream.next(m.ry);
      stream.next(m.rz);
      stream.next(m.paramValue);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MovLIORequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dobot_bringup::MovLIORequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dobot_bringup::MovLIORequest_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
    s << indent << "rx: ";
    Printer<double>::stream(s, indent + "  ", v.rx);
    s << indent << "ry: ";
    Printer<double>::stream(s, indent + "  ", v.ry);
    s << indent << "rz: ";
    Printer<double>::stream(s, indent + "  ", v.rz);
    s << indent << "paramValue[]" << std::endl;
    for (size_t i = 0; i < v.paramValue.size(); ++i)
    {
      s << indent << "  paramValue[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.paramValue[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_MOVLIOREQUEST_H
