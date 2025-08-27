#ifndef _ROS_starrobot_msgs_Sonar_h
#define _ROS_starrobot_msgs_Sonar_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace starrobot_msgs
{

  class Sonar : public ros::Msg
  {
    public:
      typedef float _sonar1_type;
      _sonar1_type sonar1;
      typedef float _sonar2_type;
      _sonar2_type sonar2;
      typedef float _sonar3_type;
      _sonar3_type sonar3;
      typedef float _sonar4_type;
      _sonar4_type sonar4;

    Sonar():
      sonar1(0),
      sonar2(0),
      sonar3(0),
      sonar4(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_sonar1;
      u_sonar1.real = this->sonar1;
      *(outbuffer + offset + 0) = (u_sonar1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sonar1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sonar1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sonar1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sonar1);
      union {
        float real;
        uint32_t base;
      } u_sonar2;
      u_sonar2.real = this->sonar2;
      *(outbuffer + offset + 0) = (u_sonar2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sonar2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sonar2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sonar2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sonar2);
      union {
        float real;
        uint32_t base;
      } u_sonar3;
      u_sonar3.real = this->sonar3;
      *(outbuffer + offset + 0) = (u_sonar3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sonar3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sonar3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sonar3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sonar3);
      union {
        float real;
        uint32_t base;
      } u_sonar4;
      u_sonar4.real = this->sonar4;
      *(outbuffer + offset + 0) = (u_sonar4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sonar4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sonar4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sonar4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sonar4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_sonar1;
      u_sonar1.base = 0;
      u_sonar1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sonar1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sonar1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sonar1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sonar1 = u_sonar1.real;
      offset += sizeof(this->sonar1);
      union {
        float real;
        uint32_t base;
      } u_sonar2;
      u_sonar2.base = 0;
      u_sonar2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sonar2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sonar2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sonar2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sonar2 = u_sonar2.real;
      offset += sizeof(this->sonar2);
      union {
        float real;
        uint32_t base;
      } u_sonar3;
      u_sonar3.base = 0;
      u_sonar3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sonar3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sonar3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sonar3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sonar3 = u_sonar3.real;
      offset += sizeof(this->sonar3);
      union {
        float real;
        uint32_t base;
      } u_sonar4;
      u_sonar4.base = 0;
      u_sonar4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sonar4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sonar4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sonar4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sonar4 = u_sonar4.real;
      offset += sizeof(this->sonar4);
     return offset;
    }

    const char * getType(){ return "starrobot_msgs/Sonar"; };
    const char * getMD5(){ return "2f513f8756037fca86f700a692eb93b6"; };

  };

}
#endif
