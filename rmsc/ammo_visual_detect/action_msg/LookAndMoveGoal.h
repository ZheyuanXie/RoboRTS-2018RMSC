// Generated by gencpp from file mag_track/LookAndMoveGoal.msg
// DO NOT EDIT!


#ifndef MAG_TRACK_MESSAGE_LOOKANDMOVEGOAL_H
#define MAG_TRACK_MESSAGE_LOOKANDMOVEGOAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/PoseStamped.h>

namespace mag_track
{
template <class ContainerAllocator>
struct LookAndMoveGoal_
{
  typedef LookAndMoveGoal_<ContainerAllocator> Type;

  LookAndMoveGoal_()
    : relative_pose()  {
    }
  LookAndMoveGoal_(const ContainerAllocator& _alloc)
    : relative_pose(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::PoseStamped_<ContainerAllocator>  _relative_pose_type;
  _relative_pose_type relative_pose;





  typedef boost::shared_ptr< ::mag_track::LookAndMoveGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mag_track::LookAndMoveGoal_<ContainerAllocator> const> ConstPtr;

}; // struct LookAndMoveGoal_

typedef ::mag_track::LookAndMoveGoal_<std::allocator<void> > LookAndMoveGoal;

typedef boost::shared_ptr< ::mag_track::LookAndMoveGoal > LookAndMoveGoalPtr;
typedef boost::shared_ptr< ::mag_track::LookAndMoveGoal const> LookAndMoveGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mag_track::LookAndMoveGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mag_track::LookAndMoveGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace mag_track

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'mag_track': ['/home/zwa/my_ws_actuell/devel/share/mag_track/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::mag_track::LookAndMoveGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mag_track::LookAndMoveGoal_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mag_track::LookAndMoveGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mag_track::LookAndMoveGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mag_track::LookAndMoveGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mag_track::LookAndMoveGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mag_track::LookAndMoveGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8c05d6d779889ba0bc93b43cba460d34";
  }

  static const char* value(const ::mag_track::LookAndMoveGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8c05d6d779889ba0ULL;
  static const uint64_t static_value2 = 0xbc93b43cba460d34ULL;
};

template<class ContainerAllocator>
struct DataType< ::mag_track::LookAndMoveGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mag_track/LookAndMoveGoal";
  }

  static const char* value(const ::mag_track::LookAndMoveGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mag_track::LookAndMoveGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#goal definition\n\
geometry_msgs/PoseStamped relative_pose\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseStamped\n\
# A Pose with reference coordinate frame and timestamp\n\
Header header\n\
Pose pose\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of position and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
";
  }

  static const char* value(const ::mag_track::LookAndMoveGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mag_track::LookAndMoveGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.relative_pose);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LookAndMoveGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mag_track::LookAndMoveGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mag_track::LookAndMoveGoal_<ContainerAllocator>& v)
  {
    s << indent << "relative_pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.relative_pose);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MAG_TRACK_MESSAGE_LOOKANDMOVEGOAL_H
