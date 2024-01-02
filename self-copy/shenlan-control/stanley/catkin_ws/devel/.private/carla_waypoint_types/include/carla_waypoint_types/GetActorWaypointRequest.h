// Generated by gencpp from file carla_waypoint_types/GetActorWaypointRequest.msg
// DO NOT EDIT!


#ifndef CARLA_WAYPOINT_TYPES_MESSAGE_GETACTORWAYPOINTREQUEST_H
#define CARLA_WAYPOINT_TYPES_MESSAGE_GETACTORWAYPOINTREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace carla_waypoint_types
{
template <class ContainerAllocator>
struct GetActorWaypointRequest_
{
  typedef GetActorWaypointRequest_<ContainerAllocator> Type;

  GetActorWaypointRequest_()
    : id(0)  {
    }
  GetActorWaypointRequest_(const ContainerAllocator& _alloc)
    : id(0)  {
  (void)_alloc;
    }



   typedef uint32_t _id_type;
  _id_type id;





  typedef boost::shared_ptr< ::carla_waypoint_types::GetActorWaypointRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::carla_waypoint_types::GetActorWaypointRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetActorWaypointRequest_

typedef ::carla_waypoint_types::GetActorWaypointRequest_<std::allocator<void> > GetActorWaypointRequest;

typedef boost::shared_ptr< ::carla_waypoint_types::GetActorWaypointRequest > GetActorWaypointRequestPtr;
typedef boost::shared_ptr< ::carla_waypoint_types::GetActorWaypointRequest const> GetActorWaypointRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::carla_waypoint_types::GetActorWaypointRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::carla_waypoint_types::GetActorWaypointRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::carla_waypoint_types::GetActorWaypointRequest_<ContainerAllocator1> & lhs, const ::carla_waypoint_types::GetActorWaypointRequest_<ContainerAllocator2> & rhs)
{
  return lhs.id == rhs.id;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::carla_waypoint_types::GetActorWaypointRequest_<ContainerAllocator1> & lhs, const ::carla_waypoint_types::GetActorWaypointRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace carla_waypoint_types

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::carla_waypoint_types::GetActorWaypointRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::carla_waypoint_types::GetActorWaypointRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::carla_waypoint_types::GetActorWaypointRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::carla_waypoint_types::GetActorWaypointRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::carla_waypoint_types::GetActorWaypointRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::carla_waypoint_types::GetActorWaypointRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::carla_waypoint_types::GetActorWaypointRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "309d4b30834b338cced19e5622a97a03";
  }

  static const char* value(const ::carla_waypoint_types::GetActorWaypointRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x309d4b30834b338cULL;
  static const uint64_t static_value2 = 0xced19e5622a97a03ULL;
};

template<class ContainerAllocator>
struct DataType< ::carla_waypoint_types::GetActorWaypointRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "carla_waypoint_types/GetActorWaypointRequest";
  }

  static const char* value(const ::carla_waypoint_types::GetActorWaypointRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::carla_waypoint_types::GetActorWaypointRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#\n"
"# Copyright (c) 2020 Intel Corporation.\n"
"#\n"
"# This work is licensed under the terms of the MIT license.\n"
"# For a copy, see <https://opensource.org/licenses/MIT>.\n"
"#\n"
"uint32 id\n"
;
  }

  static const char* value(const ::carla_waypoint_types::GetActorWaypointRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::carla_waypoint_types::GetActorWaypointRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetActorWaypointRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::carla_waypoint_types::GetActorWaypointRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::carla_waypoint_types::GetActorWaypointRequest_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CARLA_WAYPOINT_TYPES_MESSAGE_GETACTORWAYPOINTREQUEST_H