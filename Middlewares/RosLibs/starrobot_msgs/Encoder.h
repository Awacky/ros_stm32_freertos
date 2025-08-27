#ifndef _ROS_starrobot_msgs_Encoder_h
#define _ROS_starrobot_msgs_Encoder_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace starrobot_msgs
{

  class Encoder : public ros::Msg
  {
    public:
      typedef int64_t _MotorEncoder1_type;
      _MotorEncoder1_type MotorEncoder1;
      typedef int64_t _MotorEncoder2_type;
      _MotorEncoder2_type MotorEncoder2;
      typedef int64_t _MotorEncoder3_type;
      _MotorEncoder3_type MotorEncoder3;
      typedef int64_t _MotorEncoder4_type;
      _MotorEncoder4_type MotorEncoder4;

    Encoder():
      MotorEncoder1(0),
      MotorEncoder2(0),
      MotorEncoder3(0),
      MotorEncoder4(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_MotorEncoder1;
      u_MotorEncoder1.real = this->MotorEncoder1;
      *(outbuffer + offset + 0) = (u_MotorEncoder1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_MotorEncoder1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_MotorEncoder1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_MotorEncoder1.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_MotorEncoder1.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_MotorEncoder1.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_MotorEncoder1.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_MotorEncoder1.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->MotorEncoder1);
      union {
        int64_t real;
        uint64_t base;
      } u_MotorEncoder2;
      u_MotorEncoder2.real = this->MotorEncoder2;
      *(outbuffer + offset + 0) = (u_MotorEncoder2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_MotorEncoder2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_MotorEncoder2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_MotorEncoder2.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_MotorEncoder2.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_MotorEncoder2.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_MotorEncoder2.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_MotorEncoder2.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->MotorEncoder2);
      union {
        int64_t real;
        uint64_t base;
      } u_MotorEncoder3;
      u_MotorEncoder3.real = this->MotorEncoder3;
      *(outbuffer + offset + 0) = (u_MotorEncoder3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_MotorEncoder3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_MotorEncoder3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_MotorEncoder3.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_MotorEncoder3.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_MotorEncoder3.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_MotorEncoder3.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_MotorEncoder3.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->MotorEncoder3);
      union {
        int64_t real;
        uint64_t base;
      } u_MotorEncoder4;
      u_MotorEncoder4.real = this->MotorEncoder4;
      *(outbuffer + offset + 0) = (u_MotorEncoder4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_MotorEncoder4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_MotorEncoder4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_MotorEncoder4.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_MotorEncoder4.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_MotorEncoder4.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_MotorEncoder4.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_MotorEncoder4.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->MotorEncoder4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_MotorEncoder1;
      u_MotorEncoder1.base = 0;
      u_MotorEncoder1.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_MotorEncoder1.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_MotorEncoder1.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_MotorEncoder1.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_MotorEncoder1.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_MotorEncoder1.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_MotorEncoder1.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_MotorEncoder1.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->MotorEncoder1 = u_MotorEncoder1.real;
      offset += sizeof(this->MotorEncoder1);
      union {
        int64_t real;
        uint64_t base;
      } u_MotorEncoder2;
      u_MotorEncoder2.base = 0;
      u_MotorEncoder2.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_MotorEncoder2.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_MotorEncoder2.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_MotorEncoder2.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_MotorEncoder2.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_MotorEncoder2.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_MotorEncoder2.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_MotorEncoder2.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->MotorEncoder2 = u_MotorEncoder2.real;
      offset += sizeof(this->MotorEncoder2);
      union {
        int64_t real;
        uint64_t base;
      } u_MotorEncoder3;
      u_MotorEncoder3.base = 0;
      u_MotorEncoder3.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_MotorEncoder3.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_MotorEncoder3.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_MotorEncoder3.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_MotorEncoder3.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_MotorEncoder3.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_MotorEncoder3.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_MotorEncoder3.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->MotorEncoder3 = u_MotorEncoder3.real;
      offset += sizeof(this->MotorEncoder3);
      union {
        int64_t real;
        uint64_t base;
      } u_MotorEncoder4;
      u_MotorEncoder4.base = 0;
      u_MotorEncoder4.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_MotorEncoder4.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_MotorEncoder4.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_MotorEncoder4.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_MotorEncoder4.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_MotorEncoder4.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_MotorEncoder4.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_MotorEncoder4.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->MotorEncoder4 = u_MotorEncoder4.real;
      offset += sizeof(this->MotorEncoder4);
     return offset;
    }

    const char * getType(){ return "starrobot_msgs/Encoder"; };
    const char * getMD5(){ return "5fcb479f75c8295a0270fd74c95e7c94"; };

  };

}
#endif
