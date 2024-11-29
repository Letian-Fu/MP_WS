// Generated by gencpp from file dobot_v4_bringup/ModbusRTUCreateRequest.msg
// DO NOT EDIT!


#ifndef DOBOT_V4_BRINGUP_MESSAGE_MODBUSRTUCREATEREQUEST_H
#define DOBOT_V4_BRINGUP_MESSAGE_MODBUSRTUCREATEREQUEST_H


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
struct ModbusRTUCreateRequest_
{
  typedef ModbusRTUCreateRequest_<ContainerAllocator> Type;

  ModbusRTUCreateRequest_()
    : slave_id(0)
    , baud(0)
    , parity()
    , data_bit(0)
    , stop_bit(0)  {
    }
  ModbusRTUCreateRequest_(const ContainerAllocator& _alloc)
    : slave_id(0)
    , baud(0)
    , parity(_alloc)
    , data_bit(0)
    , stop_bit(0)  {
  (void)_alloc;
    }



   typedef int32_t _slave_id_type;
  _slave_id_type slave_id;

   typedef int32_t _baud_type;
  _baud_type baud;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _parity_type;
  _parity_type parity;

   typedef int32_t _data_bit_type;
  _data_bit_type data_bit;

   typedef int32_t _stop_bit_type;
  _stop_bit_type stop_bit;





  typedef boost::shared_ptr< ::dobot_v4_bringup::ModbusRTUCreateRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dobot_v4_bringup::ModbusRTUCreateRequest_<ContainerAllocator> const> ConstPtr;

}; // struct ModbusRTUCreateRequest_

typedef ::dobot_v4_bringup::ModbusRTUCreateRequest_<std::allocator<void> > ModbusRTUCreateRequest;

typedef boost::shared_ptr< ::dobot_v4_bringup::ModbusRTUCreateRequest > ModbusRTUCreateRequestPtr;
typedef boost::shared_ptr< ::dobot_v4_bringup::ModbusRTUCreateRequest const> ModbusRTUCreateRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dobot_v4_bringup::ModbusRTUCreateRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dobot_v4_bringup::ModbusRTUCreateRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dobot_v4_bringup::ModbusRTUCreateRequest_<ContainerAllocator1> & lhs, const ::dobot_v4_bringup::ModbusRTUCreateRequest_<ContainerAllocator2> & rhs)
{
  return lhs.slave_id == rhs.slave_id &&
    lhs.baud == rhs.baud &&
    lhs.parity == rhs.parity &&
    lhs.data_bit == rhs.data_bit &&
    lhs.stop_bit == rhs.stop_bit;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dobot_v4_bringup::ModbusRTUCreateRequest_<ContainerAllocator1> & lhs, const ::dobot_v4_bringup::ModbusRTUCreateRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dobot_v4_bringup

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dobot_v4_bringup::ModbusRTUCreateRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot_v4_bringup::ModbusRTUCreateRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_v4_bringup::ModbusRTUCreateRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_v4_bringup::ModbusRTUCreateRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_v4_bringup::ModbusRTUCreateRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_v4_bringup::ModbusRTUCreateRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dobot_v4_bringup::ModbusRTUCreateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "eec84b00c5325caca0ec6593be21df0e";
  }

  static const char* value(const ::dobot_v4_bringup::ModbusRTUCreateRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xeec84b00c5325cacULL;
  static const uint64_t static_value2 = 0xa0ec6593be21df0eULL;
};

template<class ContainerAllocator>
struct DataType< ::dobot_v4_bringup::ModbusRTUCreateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dobot_v4_bringup/ModbusRTUCreateRequest";
  }

  static const char* value(const ::dobot_v4_bringup::ModbusRTUCreateRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dobot_v4_bringup::ModbusRTUCreateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32   slave_id\n"
"int32   baud\n"
"string  parity\n"
"int32   data_bit\n"
"int32   stop_bit\n"
;
  }

  static const char* value(const ::dobot_v4_bringup::ModbusRTUCreateRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dobot_v4_bringup::ModbusRTUCreateRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.slave_id);
      stream.next(m.baud);
      stream.next(m.parity);
      stream.next(m.data_bit);
      stream.next(m.stop_bit);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ModbusRTUCreateRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dobot_v4_bringup::ModbusRTUCreateRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dobot_v4_bringup::ModbusRTUCreateRequest_<ContainerAllocator>& v)
  {
    s << indent << "slave_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.slave_id);
    s << indent << "baud: ";
    Printer<int32_t>::stream(s, indent + "  ", v.baud);
    s << indent << "parity: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.parity);
    s << indent << "data_bit: ";
    Printer<int32_t>::stream(s, indent + "  ", v.data_bit);
    s << indent << "stop_bit: ";
    Printer<int32_t>::stream(s, indent + "  ", v.stop_bit);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOBOT_V4_BRINGUP_MESSAGE_MODBUSRTUCREATEREQUEST_H