#ifndef _ROS_starrobot_msgs_Crash_h
#define _ROS_starrobot_msgs_Crash_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace starrobot_msgs
{

  class Crash : public ros::Msg
  {
    public:
      typedef bool _crash1_type;
      _crash1_type crash1;
      typedef bool _crash2_type;
      _crash2_type crash2;
      typedef bool _crash3_type;
      _crash3_type crash3;
      typedef bool _crash4_type;
      _crash4_type crash4;

    Crash():
      crash1(0),
      crash2(0),
      crash3(0),
      crash4(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_crash1;
      u_crash1.real = this->crash1;
      *(outbuffer + offset + 0) = (u_crash1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->crash1);
      union {
        bool real;
        uint8_t base;
      } u_crash2;
      u_crash2.real = this->crash2;
      *(outbuffer + offset + 0) = (u_crash2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->crash2);
      union {
        bool real;
        uint8_t base;
      } u_crash3;
      u_crash3.real = this->crash3;
      *(outbuffer + offset + 0) = (u_crash3.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->crash3);
      union {
        bool real;
        uint8_t base;
      } u_crash4;
      u_crash4.real = this->crash4;
      *(outbuffer + offset + 0) = (u_crash4.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->crash4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_crash1;
      u_crash1.base = 0;
      u_crash1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->crash1 = u_crash1.real;
      offset += sizeof(this->crash1);
      union {
        bool real;
        uint8_t base;
      } u_crash2;
      u_crash2.base = 0;
      u_crash2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->crash2 = u_crash2.real;
      offset += sizeof(this->crash2);
      union {
        bool real;
        uint8_t base;
      } u_crash3;
      u_crash3.base = 0;
      u_crash3.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->crash3 = u_crash3.real;
      offset += sizeof(this->crash3);
      union {
        bool real;
        uint8_t base;
      } u_crash4;
      u_crash4.base = 0;
      u_crash4.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->crash4 = u_crash4.real;
      offset += sizeof(this->crash4);
     return offset;
    }

    const char * getType(){ return "starrobot_msgs/Crash"; };
    const char * getMD5(){ return "02b77d6efd11009bfc24883f65767248"; };

  };

}
#endif
