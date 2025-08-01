// Generated by gencpp from file gripper_action/gripperMoveResult.msg
// DO NOT EDIT!


#ifndef GRIPPER_ACTION_MESSAGE_GRIPPERMOVERESULT_H
#define GRIPPER_ACTION_MESSAGE_GRIPPERMOVERESULT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace gripper_action
{
template <class ContainerAllocator>
struct gripperMoveResult_
{
  typedef gripperMoveResult_<ContainerAllocator> Type;

  gripperMoveResult_()
    : success(false)
    , failed(false)
    , error(false)  {
    }
  gripperMoveResult_(const ContainerAllocator& _alloc)
    : success(false)
    , failed(false)
    , error(false)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;

   typedef uint8_t _failed_type;
  _failed_type failed;

   typedef uint8_t _error_type;
  _error_type error;





  typedef boost::shared_ptr< ::gripper_action::gripperMoveResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gripper_action::gripperMoveResult_<ContainerAllocator> const> ConstPtr;

}; // struct gripperMoveResult_

typedef ::gripper_action::gripperMoveResult_<std::allocator<void> > gripperMoveResult;

typedef boost::shared_ptr< ::gripper_action::gripperMoveResult > gripperMoveResultPtr;
typedef boost::shared_ptr< ::gripper_action::gripperMoveResult const> gripperMoveResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::gripper_action::gripperMoveResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::gripper_action::gripperMoveResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::gripper_action::gripperMoveResult_<ContainerAllocator1> & lhs, const ::gripper_action::gripperMoveResult_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success &&
    lhs.failed == rhs.failed &&
    lhs.error == rhs.error;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::gripper_action::gripperMoveResult_<ContainerAllocator1> & lhs, const ::gripper_action::gripperMoveResult_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace gripper_action

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::gripper_action::gripperMoveResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gripper_action::gripperMoveResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gripper_action::gripperMoveResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gripper_action::gripperMoveResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gripper_action::gripperMoveResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gripper_action::gripperMoveResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::gripper_action::gripperMoveResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "86a42cb736fc6aa448ba78030c84689c";
  }

  static const char* value(const ::gripper_action::gripperMoveResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x86a42cb736fc6aa4ULL;
  static const uint64_t static_value2 = 0x48ba78030c84689cULL;
};

template<class ContainerAllocator>
struct DataType< ::gripper_action::gripperMoveResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "gripper_action/gripperMoveResult";
  }

  static const char* value(const ::gripper_action::gripperMoveResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::gripper_action::gripperMoveResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"bool success\n"
"bool failed\n"
"bool error\n"
;
  }

  static const char* value(const ::gripper_action::gripperMoveResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::gripper_action::gripperMoveResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
      stream.next(m.failed);
      stream.next(m.error);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct gripperMoveResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gripper_action::gripperMoveResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::gripper_action::gripperMoveResult_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
    s << indent << "failed: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.failed);
    s << indent << "error: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.error);
  }
};

} // namespace message_operations
} // namespace ros

#endif // GRIPPER_ACTION_MESSAGE_GRIPPERMOVERESULT_H
