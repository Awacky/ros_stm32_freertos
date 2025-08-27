#ifndef _ROS_starrobot_msgs_analog_h
#define _ROS_starrobot_msgs_analog_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace starrobot_msgs
{

  class analog : public ros::Msg
  {
    public:
      typedef float _temp_Mcu_type;
      _temp_Mcu_type temp_Mcu;
      typedef float _temp_M1_type;
      _temp_M1_type temp_M1;
      typedef float _temp_M2_type;
      _temp_M2_type temp_M2;
      typedef float _temp_M3_type;
      _temp_M3_type temp_M3;
      typedef float _temp_M4_type;
      _temp_M4_type temp_M4;
      typedef float _voltage_Bus_type;
      _voltage_Bus_type voltage_Bus;
      typedef float _current_M1_type;
      _current_M1_type current_M1;
      typedef float _current_M2_type;
      _current_M2_type current_M2;
      typedef float _current_M3_type;
      _current_M3_type current_M3;
      typedef float _current_M4_type;
      _current_M4_type current_M4;

    analog():
      temp_Mcu(0),
      temp_M1(0),
      temp_M2(0),
      temp_M3(0),
      temp_M4(0),
      voltage_Bus(0),
      current_M1(0),
      current_M2(0),
      current_M3(0),
      current_M4(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_temp_Mcu;
      u_temp_Mcu.real = this->temp_Mcu;
      *(outbuffer + offset + 0) = (u_temp_Mcu.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temp_Mcu.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temp_Mcu.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temp_Mcu.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temp_Mcu);
      union {
        float real;
        uint32_t base;
      } u_temp_M1;
      u_temp_M1.real = this->temp_M1;
      *(outbuffer + offset + 0) = (u_temp_M1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temp_M1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temp_M1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temp_M1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temp_M1);
      union {
        float real;
        uint32_t base;
      } u_temp_M2;
      u_temp_M2.real = this->temp_M2;
      *(outbuffer + offset + 0) = (u_temp_M2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temp_M2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temp_M2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temp_M2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temp_M2);
      union {
        float real;
        uint32_t base;
      } u_temp_M3;
      u_temp_M3.real = this->temp_M3;
      *(outbuffer + offset + 0) = (u_temp_M3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temp_M3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temp_M3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temp_M3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temp_M3);
      union {
        float real;
        uint32_t base;
      } u_temp_M4;
      u_temp_M4.real = this->temp_M4;
      *(outbuffer + offset + 0) = (u_temp_M4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temp_M4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temp_M4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temp_M4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temp_M4);
      union {
        float real;
        uint32_t base;
      } u_voltage_Bus;
      u_voltage_Bus.real = this->voltage_Bus;
      *(outbuffer + offset + 0) = (u_voltage_Bus.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_voltage_Bus.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_voltage_Bus.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_voltage_Bus.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->voltage_Bus);
      union {
        float real;
        uint32_t base;
      } u_current_M1;
      u_current_M1.real = this->current_M1;
      *(outbuffer + offset + 0) = (u_current_M1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_M1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_M1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_M1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_M1);
      union {
        float real;
        uint32_t base;
      } u_current_M2;
      u_current_M2.real = this->current_M2;
      *(outbuffer + offset + 0) = (u_current_M2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_M2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_M2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_M2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_M2);
      union {
        float real;
        uint32_t base;
      } u_current_M3;
      u_current_M3.real = this->current_M3;
      *(outbuffer + offset + 0) = (u_current_M3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_M3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_M3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_M3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_M3);
      union {
        float real;
        uint32_t base;
      } u_current_M4;
      u_current_M4.real = this->current_M4;
      *(outbuffer + offset + 0) = (u_current_M4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_M4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_M4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_M4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_M4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_temp_Mcu;
      u_temp_Mcu.base = 0;
      u_temp_Mcu.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temp_Mcu.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temp_Mcu.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temp_Mcu.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temp_Mcu = u_temp_Mcu.real;
      offset += sizeof(this->temp_Mcu);
      union {
        float real;
        uint32_t base;
      } u_temp_M1;
      u_temp_M1.base = 0;
      u_temp_M1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temp_M1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temp_M1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temp_M1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temp_M1 = u_temp_M1.real;
      offset += sizeof(this->temp_M1);
      union {
        float real;
        uint32_t base;
      } u_temp_M2;
      u_temp_M2.base = 0;
      u_temp_M2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temp_M2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temp_M2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temp_M2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temp_M2 = u_temp_M2.real;
      offset += sizeof(this->temp_M2);
      union {
        float real;
        uint32_t base;
      } u_temp_M3;
      u_temp_M3.base = 0;
      u_temp_M3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temp_M3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temp_M3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temp_M3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temp_M3 = u_temp_M3.real;
      offset += sizeof(this->temp_M3);
      union {
        float real;
        uint32_t base;
      } u_temp_M4;
      u_temp_M4.base = 0;
      u_temp_M4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temp_M4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temp_M4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temp_M4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temp_M4 = u_temp_M4.real;
      offset += sizeof(this->temp_M4);
      union {
        float real;
        uint32_t base;
      } u_voltage_Bus;
      u_voltage_Bus.base = 0;
      u_voltage_Bus.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_voltage_Bus.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_voltage_Bus.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_voltage_Bus.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->voltage_Bus = u_voltage_Bus.real;
      offset += sizeof(this->voltage_Bus);
      union {
        float real;
        uint32_t base;
      } u_current_M1;
      u_current_M1.base = 0;
      u_current_M1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_M1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_M1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_M1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_M1 = u_current_M1.real;
      offset += sizeof(this->current_M1);
      union {
        float real;
        uint32_t base;
      } u_current_M2;
      u_current_M2.base = 0;
      u_current_M2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_M2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_M2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_M2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_M2 = u_current_M2.real;
      offset += sizeof(this->current_M2);
      union {
        float real;
        uint32_t base;
      } u_current_M3;
      u_current_M3.base = 0;
      u_current_M3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_M3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_M3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_M3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_M3 = u_current_M3.real;
      offset += sizeof(this->current_M3);
      union {
        float real;
        uint32_t base;
      } u_current_M4;
      u_current_M4.base = 0;
      u_current_M4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_M4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_M4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_M4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_M4 = u_current_M4.real;
      offset += sizeof(this->current_M4);
     return offset;
    }

    const char * getType(){ return "starrobot_msgs/analog"; };
    const char * getMD5(){ return "d7972d54e21bbe3cd2ef75a2fb30c1d1"; };

  };

}
#endif
