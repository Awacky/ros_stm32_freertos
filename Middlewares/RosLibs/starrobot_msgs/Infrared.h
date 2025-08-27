#ifndef _ROS_starrobot_msgs_Infrared_h
#define _ROS_starrobot_msgs_Infrared_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace starrobot_msgs
{

  class Infrared : public ros::Msg
  {
    public:
      typedef bool _infrared1_type;
      _infrared1_type infrared1;
      typedef bool _infrared2_type;
      _infrared2_type infrared2;
      typedef bool _infrared3_type;
      _infrared3_type infrared3;
      typedef bool _infrared4_type;
      _infrared4_type infrared4;

    Infrared():
      infrared1(0),
      infrared2(0),
      infrared3(0),
      infrared4(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_infrared1;
      u_infrared1.real = this->infrared1;
      *(outbuffer + offset + 0) = (u_infrared1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->infrared1);
      union {
        bool real;
        uint8_t base;
      } u_infrared2;
      u_infrared2.real = this->infrared2;
      *(outbuffer + offset + 0) = (u_infrared2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->infrared2);
      union {
        bool real;
        uint8_t base;
      } u_infrared3;
      u_infrared3.real = this->infrared3;
      *(outbuffer + offset + 0) = (u_infrared3.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->infrared3);
      union {
        bool real;
        uint8_t base;
      } u_infrared4;
      u_infrared4.real = this->infrared4;
      *(outbuffer + offset + 0) = (u_infrared4.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->infrared4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_infrared1;
      u_infrared1.base = 0;
      u_infrared1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->infrared1 = u_infrared1.real;
      offset += sizeof(this->infrared1);
      union {
        bool real;
        uint8_t base;
      } u_infrared2;
      u_infrared2.base = 0;
      u_infrared2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->infrared2 = u_infrared2.real;
      offset += sizeof(this->infrared2);
      union {
        bool real;
        uint8_t base;
      } u_infrared3;
      u_infrared3.base = 0;
      u_infrared3.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->infrared3 = u_infrared3.real;
      offset += sizeof(this->infrared3);
      union {
        bool real;
        uint8_t base;
      } u_infrared4;
      u_infrared4.base = 0;
      u_infrared4.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->infrared4 = u_infrared4.real;
      offset += sizeof(this->infrared4);
     return offset;
    }

    const char * getType(){ return "starrobot_msgs/Infrared"; };
    const char * getMD5(){ return "4686eff1b895a48c46b472d5af75db05"; };

  };

}
#endif
