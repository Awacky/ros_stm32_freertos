#ifndef _ROS_starrobot_msgs_PowerKey_h
#define _ROS_starrobot_msgs_PowerKey_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace starrobot_msgs
{

  class PowerKey : public ros::Msg
  {
    public:
      typedef bool _powerkey1_type;
      _powerkey1_type powerkey1;
      typedef bool _powerkey2_type;
      _powerkey2_type powerkey2;
      typedef bool _powerkey3_type;
      _powerkey3_type powerkey3;
      typedef bool _powerkey4_type;
      _powerkey4_type powerkey4;

    PowerKey():
      powerkey1(0),
      powerkey2(0),
      powerkey3(0),
      powerkey4(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_powerkey1;
      u_powerkey1.real = this->powerkey1;
      *(outbuffer + offset + 0) = (u_powerkey1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->powerkey1);
      union {
        bool real;
        uint8_t base;
      } u_powerkey2;
      u_powerkey2.real = this->powerkey2;
      *(outbuffer + offset + 0) = (u_powerkey2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->powerkey2);
      union {
        bool real;
        uint8_t base;
      } u_powerkey3;
      u_powerkey3.real = this->powerkey3;
      *(outbuffer + offset + 0) = (u_powerkey3.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->powerkey3);
      union {
        bool real;
        uint8_t base;
      } u_powerkey4;
      u_powerkey4.real = this->powerkey4;
      *(outbuffer + offset + 0) = (u_powerkey4.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->powerkey4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_powerkey1;
      u_powerkey1.base = 0;
      u_powerkey1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->powerkey1 = u_powerkey1.real;
      offset += sizeof(this->powerkey1);
      union {
        bool real;
        uint8_t base;
      } u_powerkey2;
      u_powerkey2.base = 0;
      u_powerkey2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->powerkey2 = u_powerkey2.real;
      offset += sizeof(this->powerkey2);
      union {
        bool real;
        uint8_t base;
      } u_powerkey3;
      u_powerkey3.base = 0;
      u_powerkey3.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->powerkey3 = u_powerkey3.real;
      offset += sizeof(this->powerkey3);
      union {
        bool real;
        uint8_t base;
      } u_powerkey4;
      u_powerkey4.base = 0;
      u_powerkey4.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->powerkey4 = u_powerkey4.real;
      offset += sizeof(this->powerkey4);
     return offset;
    }

    const char * getType(){ return "starrobot_msgs/PowerKey"; };
    const char * getMD5(){ return "b797b23d4e603a9d30a243a455a1cc76"; };

  };

}
#endif
