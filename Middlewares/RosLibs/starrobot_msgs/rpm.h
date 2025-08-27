#ifndef _ROS_starrobot_msgs_rpm_h
#define _ROS_starrobot_msgs_rpm_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace starrobot_msgs
{

  class rpm : public ros::Msg
  {
    public:
      typedef int32_t _MotorRpm1_type;
      _MotorRpm1_type MotorRpm1;
      typedef int32_t _MotorRpm2_type;
      _MotorRpm2_type MotorRpm2;
      typedef int32_t _MotorRpm3_type;
      _MotorRpm3_type MotorRpm3;
      typedef int32_t _MotorRpm4_type;
      _MotorRpm4_type MotorRpm4;

    rpm():
      MotorRpm1(0),
      MotorRpm2(0),
      MotorRpm3(0),
      MotorRpm4(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_MotorRpm1;
      u_MotorRpm1.real = this->MotorRpm1;
      *(outbuffer + offset + 0) = (u_MotorRpm1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_MotorRpm1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_MotorRpm1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_MotorRpm1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->MotorRpm1);
      union {
        int32_t real;
        uint32_t base;
      } u_MotorRpm2;
      u_MotorRpm2.real = this->MotorRpm2;
      *(outbuffer + offset + 0) = (u_MotorRpm2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_MotorRpm2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_MotorRpm2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_MotorRpm2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->MotorRpm2);
      union {
        int32_t real;
        uint32_t base;
      } u_MotorRpm3;
      u_MotorRpm3.real = this->MotorRpm3;
      *(outbuffer + offset + 0) = (u_MotorRpm3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_MotorRpm3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_MotorRpm3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_MotorRpm3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->MotorRpm3);
      union {
        int32_t real;
        uint32_t base;
      } u_MotorRpm4;
      u_MotorRpm4.real = this->MotorRpm4;
      *(outbuffer + offset + 0) = (u_MotorRpm4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_MotorRpm4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_MotorRpm4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_MotorRpm4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->MotorRpm4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_MotorRpm1;
      u_MotorRpm1.base = 0;
      u_MotorRpm1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_MotorRpm1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_MotorRpm1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_MotorRpm1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->MotorRpm1 = u_MotorRpm1.real;
      offset += sizeof(this->MotorRpm1);
      union {
        int32_t real;
        uint32_t base;
      } u_MotorRpm2;
      u_MotorRpm2.base = 0;
      u_MotorRpm2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_MotorRpm2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_MotorRpm2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_MotorRpm2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->MotorRpm2 = u_MotorRpm2.real;
      offset += sizeof(this->MotorRpm2);
      union {
        int32_t real;
        uint32_t base;
      } u_MotorRpm3;
      u_MotorRpm3.base = 0;
      u_MotorRpm3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_MotorRpm3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_MotorRpm3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_MotorRpm3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->MotorRpm3 = u_MotorRpm3.real;
      offset += sizeof(this->MotorRpm3);
      union {
        int32_t real;
        uint32_t base;
      } u_MotorRpm4;
      u_MotorRpm4.base = 0;
      u_MotorRpm4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_MotorRpm4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_MotorRpm4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_MotorRpm4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->MotorRpm4 = u_MotorRpm4.real;
      offset += sizeof(this->MotorRpm4);
     return offset;
    }

    const char * getType(){ return "starrobot_msgs/rpm"; };
    const char * getMD5(){ return "73b53ebe68c9f903353f1f4c0eba54e4"; };

  };

}
#endif
