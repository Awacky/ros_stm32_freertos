#ifndef _ROS_starrobot_msgs_Servo_h
#define _ROS_starrobot_msgs_Servo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace starrobot_msgs
{

  class Servo : public ros::Msg
  {
    public:
      typedef float _servo1_type;
      _servo1_type servo1;
      typedef float _servo2_type;
      _servo2_type servo2;
      typedef float _servo3_type;
      _servo3_type servo3;
      typedef float _servo4_type;
      _servo4_type servo4;

    Servo():
      servo1(0),
      servo2(0),
      servo3(0),
      servo4(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_servo1;
      u_servo1.real = this->servo1;
      *(outbuffer + offset + 0) = (u_servo1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servo1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_servo1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_servo1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->servo1);
      union {
        float real;
        uint32_t base;
      } u_servo2;
      u_servo2.real = this->servo2;
      *(outbuffer + offset + 0) = (u_servo2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servo2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_servo2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_servo2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->servo2);
      union {
        float real;
        uint32_t base;
      } u_servo3;
      u_servo3.real = this->servo3;
      *(outbuffer + offset + 0) = (u_servo3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servo3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_servo3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_servo3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->servo3);
      union {
        float real;
        uint32_t base;
      } u_servo4;
      u_servo4.real = this->servo4;
      *(outbuffer + offset + 0) = (u_servo4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servo4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_servo4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_servo4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->servo4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_servo1;
      u_servo1.base = 0;
      u_servo1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servo1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_servo1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_servo1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->servo1 = u_servo1.real;
      offset += sizeof(this->servo1);
      union {
        float real;
        uint32_t base;
      } u_servo2;
      u_servo2.base = 0;
      u_servo2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servo2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_servo2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_servo2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->servo2 = u_servo2.real;
      offset += sizeof(this->servo2);
      union {
        float real;
        uint32_t base;
      } u_servo3;
      u_servo3.base = 0;
      u_servo3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servo3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_servo3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_servo3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->servo3 = u_servo3.real;
      offset += sizeof(this->servo3);
      union {
        float real;
        uint32_t base;
      } u_servo4;
      u_servo4.base = 0;
      u_servo4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servo4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_servo4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_servo4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->servo4 = u_servo4.real;
      offset += sizeof(this->servo4);
     return offset;
    }

    const char * getType(){ return "starrobot_msgs/Servo"; };
    const char * getMD5(){ return "e8a9ccaa500241e3aac1566f1f2857aa"; };

  };

}
#endif
