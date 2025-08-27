#ifndef _ROS_starrobot_msgs_Battery_h
#define _ROS_starrobot_msgs_Battery_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace starrobot_msgs
{

  class Battery : public ros::Msg
  {
    public:
      typedef float _voltage_type;
      _voltage_type voltage;
      typedef float _electricity_type;
      _electricity_type electricity;

    Battery():
      voltage(0),
      electricity(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.real = this->voltage;
      *(outbuffer + offset + 0) = (u_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->voltage);
      union {
        float real;
        uint32_t base;
      } u_electricity;
      u_electricity.real = this->electricity;
      *(outbuffer + offset + 0) = (u_electricity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_electricity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_electricity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_electricity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->electricity);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.base = 0;
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->voltage = u_voltage.real;
      offset += sizeof(this->voltage);
      union {
        float real;
        uint32_t base;
      } u_electricity;
      u_electricity.base = 0;
      u_electricity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_electricity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_electricity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_electricity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->electricity = u_electricity.real;
      offset += sizeof(this->electricity);
     return offset;
    }

    const char * getType(){ return "starrobot_msgs/Battery"; };
    const char * getMD5(){ return "d34fbd3d575d58d32a3790ef4eab2aca"; };

  };

}
#endif
