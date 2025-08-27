#ifndef _ROS_starrobot_msgs_Fall_h
#define _ROS_starrobot_msgs_Fall_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace starrobot_msgs
{

  class Fall : public ros::Msg
  {
    public:
      typedef bool _fall1_type;
      _fall1_type fall1;
      typedef bool _fall2_type;
      _fall2_type fall2;
      typedef bool _fall3_type;
      _fall3_type fall3;
      typedef bool _fall4_type;
      _fall4_type fall4;

    Fall():
      fall1(0),
      fall2(0),
      fall3(0),
      fall4(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_fall1;
      u_fall1.real = this->fall1;
      *(outbuffer + offset + 0) = (u_fall1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fall1);
      union {
        bool real;
        uint8_t base;
      } u_fall2;
      u_fall2.real = this->fall2;
      *(outbuffer + offset + 0) = (u_fall2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fall2);
      union {
        bool real;
        uint8_t base;
      } u_fall3;
      u_fall3.real = this->fall3;
      *(outbuffer + offset + 0) = (u_fall3.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fall3);
      union {
        bool real;
        uint8_t base;
      } u_fall4;
      u_fall4.real = this->fall4;
      *(outbuffer + offset + 0) = (u_fall4.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fall4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_fall1;
      u_fall1.base = 0;
      u_fall1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->fall1 = u_fall1.real;
      offset += sizeof(this->fall1);
      union {
        bool real;
        uint8_t base;
      } u_fall2;
      u_fall2.base = 0;
      u_fall2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->fall2 = u_fall2.real;
      offset += sizeof(this->fall2);
      union {
        bool real;
        uint8_t base;
      } u_fall3;
      u_fall3.base = 0;
      u_fall3.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->fall3 = u_fall3.real;
      offset += sizeof(this->fall3);
      union {
        bool real;
        uint8_t base;
      } u_fall4;
      u_fall4.base = 0;
      u_fall4.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->fall4 = u_fall4.real;
      offset += sizeof(this->fall4);
     return offset;
    }

    const char * getType(){ return "starrobot_msgs/Fall"; };
    const char * getMD5(){ return "ec4daf7144d18cc51c5e99829b72588d"; };

  };

}
#endif
