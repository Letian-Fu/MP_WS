// Generated by gencpp from file dobot_bringup/GetErrorIDResponse.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_GETERRORIDRESPONSE_H
#define DOBOT_BRINGUP_MESSAGE_GETERRORIDRESPONSE_H


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
struct GetErrorIDResponse_
{
  typedef GetErrorIDResponse_<ContainerAllocator> Type;

  GetErrorIDResponse_()
    : res(0)
    , errorID()  {
    }
  GetErrorIDResponse_(const ContainerAllocator& _alloc)
    : res(0)
    , errorID(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _res_type;
  _res_type res;

   typedef std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> _errorID_type;
  _errorID_type errorID;





  typedef boost::shared_ptr< ::dobot_bringup::GetErrorIDResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dobot_bringup::GetErrorIDResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetErrorIDResponse_

typedef ::dobot_bringup::GetErrorIDResponse_<std::allocator<void> > GetErrorIDResponse;

typedef boost::shared_ptr< ::dobot_bringup::GetErrorIDResponse > GetErrorIDResponsePtr;
typedef boost::shared_ptr< ::dobot_bringup::GetErrorIDResponse const> GetErrorIDResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dobot_bringup::GetErrorIDResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dobot_bringup::GetErrorIDResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dobot_bringup::GetErrorIDResponse_<ContainerAllocator1> & lhs, const ::dobot_bringup::GetErrorIDResponse_<ContainerAllocator2> & rhs)
{
  return lhs.res == rhs.res &&
    lhs.errorID == rhs.errorID;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dobot_bringup::GetErrorIDResponse_<ContainerAllocator1> & lhs, const ::dobot_bringup::GetErrorIDResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dobot_bringup

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dobot_bringup::GetErrorIDResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot_bringup::GetErrorIDResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_bringup::GetErrorIDResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_bringup::GetErrorIDResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_bringup::GetErrorIDResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_bringup::GetErrorIDResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dobot_bringup::GetErrorIDResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "21732a31ce08e00841e34036c6b93272";
  }

  static const char* value(const ::dobot_bringup::GetErrorIDResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x21732a31ce08e008ULL;
  static const uint64_t static_value2 = 0x41e34036c6b93272ULL;
};

template<class ContainerAllocator>
struct DataType< ::dobot_bringup::GetErrorIDResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dobot_bringup/GetErrorIDResponse";
  }

  static const char* value(const ::dobot_bringup::GetErrorIDResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dobot_bringup::GetErrorIDResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 res\n"
"int32[] errorID\n"
;
  }

  static const char* value(const ::dobot_bringup::GetErrorIDResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dobot_bringup::GetErrorIDResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.res);
      stream.next(m.errorID);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetErrorIDResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dobot_bringup::GetErrorIDResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dobot_bringup::GetErrorIDResponse_<ContainerAllocator>& v)
  {
    s << indent << "res: ";
    Printer<int32_t>::stream(s, indent + "  ", v.res);
    s << indent << "errorID[]" << std::endl;
    for (size_t i = 0; i < v.errorID.size(); ++i)
    {
      s << indent << "  errorID[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.errorID[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_GETERRORIDRESPONSE_H