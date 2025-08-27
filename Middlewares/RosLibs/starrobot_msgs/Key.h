#ifndef _ROS_starrobot_msgs_Key_h
#define _ROS_starrobot_msgs_Key_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace starrobot_msgs
{

  class Key : public ros::Msg
  {
    public:
      typedef bool _key1_type;
      _key1_type key1;
      typedef bool _key2_type;
      _key2_type key2;
      typedef bool _key3_type;
      _key3_type key3;
      typedef bool _key4_type;
      _key4_type key4;

    Key():
      key1(0),
      key2(0),
      key3(0),
      key4(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_key1;
      u_key1.real = this->key1;
      *(outbuffer + offset + 0) = (u_key1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->key1);
      union {
        bool real;
        uint8_t base;
      } u_key2;
      u_key2.real = this->key2;
      *(outbuffer + offset + 0) = (u_key2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->key2);
      union {
        bool real;
        uint8_t base;
      } u_key3;
      u_key3.real = this->key3;
      *(outbuffer + offset + 0) = (u_key3.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->key3);
      union {
        bool real;
        uint8_t base;
      } u_key4;
      u_key4.real = this->key4;
      *(outbuffer + offset + 0) = (u_key4.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->key4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_key1;
      u_key1.base = 0;
      u_key1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->key1 = u_key1.real;
      offset += sizeof(this->key1);
      union {
        bool real;
        uint8_t base;
      } u_key2;
      u_key2.base = 0;
      u_key2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->key2 = u_key2.real;
      offset += sizeof(this->key2);
      union {
        bool real;
        uint8_t base;
      } u_key3;
      u_key3.base = 0;
      u_key3.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->key3 = u_key3.real;
      offset += sizeof(this->key3);
      union {
        bool real;
        uint8_t base;
      } u_key4;
      u_key4.base = 0;
      u_key4.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->key4 = u_key4.real;
      offset += sizeof(this->key4);
     return offset;
    }

    const char * getType(){ return "starrobot_msgs/Key"; };
    const char * getMD5(){ return "3171c264b80fece5877bbb1adde04ad3"; };

  };

}
#endif
