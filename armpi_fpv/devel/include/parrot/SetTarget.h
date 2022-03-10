// Generated by gencpp from file parrot/SetTarget.msg
// DO NOT EDIT!


#ifndef PARROT_MESSAGE_SETTARGET_H
#define PARROT_MESSAGE_SETTARGET_H

#include <ros/service_traits.h>


#include <parrot/SetTargetRequest.h>
#include <parrot/SetTargetResponse.h>


namespace parrot
{

struct SetTarget
{

typedef SetTargetRequest Request;
typedef SetTargetResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetTarget
} // namespace parrot


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::parrot::SetTarget > {
  static const char* value()
  {
    return "546971982e3fbbd5a41e60fb6432e357";
  }

  static const char* value(const ::parrot::SetTarget&) { return value(); }
};

template<>
struct DataType< ::parrot::SetTarget > {
  static const char* value()
  {
    return "parrot/SetTarget";
  }

  static const char* value(const ::parrot::SetTarget&) { return value(); }
};


// service_traits::MD5Sum< ::parrot::SetTargetRequest> should match
// service_traits::MD5Sum< ::parrot::SetTarget >
template<>
struct MD5Sum< ::parrot::SetTargetRequest>
{
  static const char* value()
  {
    return MD5Sum< ::parrot::SetTarget >::value();
  }
  static const char* value(const ::parrot::SetTargetRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::parrot::SetTargetRequest> should match
// service_traits::DataType< ::parrot::SetTarget >
template<>
struct DataType< ::parrot::SetTargetRequest>
{
  static const char* value()
  {
    return DataType< ::parrot::SetTarget >::value();
  }
  static const char* value(const ::parrot::SetTargetRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::parrot::SetTargetResponse> should match
// service_traits::MD5Sum< ::parrot::SetTarget >
template<>
struct MD5Sum< ::parrot::SetTargetResponse>
{
  static const char* value()
  {
    return MD5Sum< ::parrot::SetTarget >::value();
  }
  static const char* value(const ::parrot::SetTargetResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::parrot::SetTargetResponse> should match
// service_traits::DataType< ::parrot::SetTarget >
template<>
struct DataType< ::parrot::SetTargetResponse>
{
  static const char* value()
  {
    return DataType< ::parrot::SetTarget >::value();
  }
  static const char* value(const ::parrot::SetTargetResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // PARROT_MESSAGE_SETTARGET_H