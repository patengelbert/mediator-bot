// Generated by gencpp from file beginner_tutorials/Action.msg
// DO NOT EDIT!


#ifndef BEGINNER_TUTORIALS_MESSAGE_ACTION_H
#define BEGINNER_TUTORIALS_MESSAGE_ACTION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace beginner_tutorials
{
template <class ContainerAllocator>
struct Action_
{
  typedef Action_<ContainerAllocator> Type;

  Action_()
    : Action()
    , Mood()
    , Person()  {
    }
  Action_(const ContainerAllocator& _alloc)
    : Action(_alloc)
    , Mood(_alloc)
    , Person(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _Action_type;
  _Action_type Action;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _Mood_type;
  _Mood_type Mood;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _Person_type;
  _Person_type Person;




  typedef boost::shared_ptr< ::beginner_tutorials::Action_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::beginner_tutorials::Action_<ContainerAllocator> const> ConstPtr;

}; // struct Action_

typedef ::beginner_tutorials::Action_<std::allocator<void> > Action;

typedef boost::shared_ptr< ::beginner_tutorials::Action > ActionPtr;
typedef boost::shared_ptr< ::beginner_tutorials::Action const> ActionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::beginner_tutorials::Action_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::beginner_tutorials::Action_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace beginner_tutorials

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'beginner_tutorials': ['/home/becks/mediator-bot/build/src/beginner_tutorials/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::beginner_tutorials::Action_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::beginner_tutorials::Action_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::beginner_tutorials::Action_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::beginner_tutorials::Action_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::beginner_tutorials::Action_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::beginner_tutorials::Action_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::beginner_tutorials::Action_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bb65f1b1581903c347625617e0507a4d";
  }

  static const char* value(const ::beginner_tutorials::Action_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbb65f1b1581903c3ULL;
  static const uint64_t static_value2 = 0x47625617e0507a4dULL;
};

template<class ContainerAllocator>
struct DataType< ::beginner_tutorials::Action_<ContainerAllocator> >
{
  static const char* value()
  {
    return "beginner_tutorials/Action";
  }

  static const char* value(const ::beginner_tutorials::Action_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::beginner_tutorials::Action_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string Action\n\
string Mood\n\
string Person\n\
";
  }

  static const char* value(const ::beginner_tutorials::Action_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::beginner_tutorials::Action_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.Action);
      stream.next(m.Mood);
      stream.next(m.Person);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Action_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::beginner_tutorials::Action_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::beginner_tutorials::Action_<ContainerAllocator>& v)
  {
    s << indent << "Action: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.Action);
    s << indent << "Mood: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.Mood);
    s << indent << "Person: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.Person);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BEGINNER_TUTORIALS_MESSAGE_ACTION_H
