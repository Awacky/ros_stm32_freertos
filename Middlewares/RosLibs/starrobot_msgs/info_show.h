#ifndef _ROS_starrobot_msgs_info_show_h
#define _ROS_starrobot_msgs_info_show_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace starrobot_msgs
{

  class info_show : public ros::Msg
  {
    public:
      typedef int32_t _Motor1_Out_Pwm_type;
      _Motor1_Out_Pwm_type Motor1_Out_Pwm;
      typedef int32_t _Motor2_Out_Pwm_type;
      _Motor2_Out_Pwm_type Motor2_Out_Pwm;
      typedef int32_t _Motor3_Out_Pwm_type;
      _Motor3_Out_Pwm_type Motor3_Out_Pwm;
      typedef int32_t _Motor4_Out_Pwm_type;
      _Motor4_Out_Pwm_type Motor4_Out_Pwm;
      typedef int32_t _Motor1_Expectations_type;
      _Motor1_Expectations_type Motor1_Expectations;
      typedef int32_t _Motor2_Expectations_type;
      _Motor2_Expectations_type Motor2_Expectations;
      typedef int32_t _Motor3_Expectations_type;
      _Motor3_Expectations_type Motor3_Expectations;
      typedef int32_t _Motor4_Expectations_type;
      _Motor4_Expectations_type Motor4_Expectations;
      typedef int32_t _Motor1_Feedback_type;
      _Motor1_Feedback_type Motor1_Feedback;
      typedef int32_t _Motor2_Feedback_type;
      _Motor2_Feedback_type Motor2_Feedback;
      typedef int32_t _Motor3_Feedback_type;
      _Motor3_Feedback_type Motor3_Feedback;
      typedef int32_t _Motor4_Feedback_type;
      _Motor4_Feedback_type Motor4_Feedback;

    info_show():
      Motor1_Out_Pwm(0),
      Motor2_Out_Pwm(0),
      Motor3_Out_Pwm(0),
      Motor4_Out_Pwm(0),
      Motor1_Expectations(0),
      Motor2_Expectations(0),
      Motor3_Expectations(0),
      Motor4_Expectations(0),
      Motor1_Feedback(0),
      Motor2_Feedback(0),
      Motor3_Feedback(0),
      Motor4_Feedback(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_Motor1_Out_Pwm;
      u_Motor1_Out_Pwm.real = this->Motor1_Out_Pwm;
      *(outbuffer + offset + 0) = (u_Motor1_Out_Pwm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Motor1_Out_Pwm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Motor1_Out_Pwm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Motor1_Out_Pwm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Motor1_Out_Pwm);
      union {
        int32_t real;
        uint32_t base;
      } u_Motor2_Out_Pwm;
      u_Motor2_Out_Pwm.real = this->Motor2_Out_Pwm;
      *(outbuffer + offset + 0) = (u_Motor2_Out_Pwm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Motor2_Out_Pwm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Motor2_Out_Pwm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Motor2_Out_Pwm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Motor2_Out_Pwm);
      union {
        int32_t real;
        uint32_t base;
      } u_Motor3_Out_Pwm;
      u_Motor3_Out_Pwm.real = this->Motor3_Out_Pwm;
      *(outbuffer + offset + 0) = (u_Motor3_Out_Pwm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Motor3_Out_Pwm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Motor3_Out_Pwm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Motor3_Out_Pwm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Motor3_Out_Pwm);
      union {
        int32_t real;
        uint32_t base;
      } u_Motor4_Out_Pwm;
      u_Motor4_Out_Pwm.real = this->Motor4_Out_Pwm;
      *(outbuffer + offset + 0) = (u_Motor4_Out_Pwm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Motor4_Out_Pwm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Motor4_Out_Pwm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Motor4_Out_Pwm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Motor4_Out_Pwm);
      union {
        int32_t real;
        uint32_t base;
      } u_Motor1_Expectations;
      u_Motor1_Expectations.real = this->Motor1_Expectations;
      *(outbuffer + offset + 0) = (u_Motor1_Expectations.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Motor1_Expectations.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Motor1_Expectations.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Motor1_Expectations.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Motor1_Expectations);
      union {
        int32_t real;
        uint32_t base;
      } u_Motor2_Expectations;
      u_Motor2_Expectations.real = this->Motor2_Expectations;
      *(outbuffer + offset + 0) = (u_Motor2_Expectations.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Motor2_Expectations.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Motor2_Expectations.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Motor2_Expectations.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Motor2_Expectations);
      union {
        int32_t real;
        uint32_t base;
      } u_Motor3_Expectations;
      u_Motor3_Expectations.real = this->Motor3_Expectations;
      *(outbuffer + offset + 0) = (u_Motor3_Expectations.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Motor3_Expectations.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Motor3_Expectations.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Motor3_Expectations.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Motor3_Expectations);
      union {
        int32_t real;
        uint32_t base;
      } u_Motor4_Expectations;
      u_Motor4_Expectations.real = this->Motor4_Expectations;
      *(outbuffer + offset + 0) = (u_Motor4_Expectations.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Motor4_Expectations.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Motor4_Expectations.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Motor4_Expectations.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Motor4_Expectations);
      union {
        int32_t real;
        uint32_t base;
      } u_Motor1_Feedback;
      u_Motor1_Feedback.real = this->Motor1_Feedback;
      *(outbuffer + offset + 0) = (u_Motor1_Feedback.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Motor1_Feedback.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Motor1_Feedback.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Motor1_Feedback.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Motor1_Feedback);
      union {
        int32_t real;
        uint32_t base;
      } u_Motor2_Feedback;
      u_Motor2_Feedback.real = this->Motor2_Feedback;
      *(outbuffer + offset + 0) = (u_Motor2_Feedback.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Motor2_Feedback.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Motor2_Feedback.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Motor2_Feedback.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Motor2_Feedback);
      union {
        int32_t real;
        uint32_t base;
      } u_Motor3_Feedback;
      u_Motor3_Feedback.real = this->Motor3_Feedback;
      *(outbuffer + offset + 0) = (u_Motor3_Feedback.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Motor3_Feedback.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Motor3_Feedback.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Motor3_Feedback.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Motor3_Feedback);
      union {
        int32_t real;
        uint32_t base;
      } u_Motor4_Feedback;
      u_Motor4_Feedback.real = this->Motor4_Feedback;
      *(outbuffer + offset + 0) = (u_Motor4_Feedback.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Motor4_Feedback.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Motor4_Feedback.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Motor4_Feedback.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Motor4_Feedback);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_Motor1_Out_Pwm;
      u_Motor1_Out_Pwm.base = 0;
      u_Motor1_Out_Pwm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Motor1_Out_Pwm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Motor1_Out_Pwm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Motor1_Out_Pwm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Motor1_Out_Pwm = u_Motor1_Out_Pwm.real;
      offset += sizeof(this->Motor1_Out_Pwm);
      union {
        int32_t real;
        uint32_t base;
      } u_Motor2_Out_Pwm;
      u_Motor2_Out_Pwm.base = 0;
      u_Motor2_Out_Pwm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Motor2_Out_Pwm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Motor2_Out_Pwm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Motor2_Out_Pwm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Motor2_Out_Pwm = u_Motor2_Out_Pwm.real;
      offset += sizeof(this->Motor2_Out_Pwm);
      union {
        int32_t real;
        uint32_t base;
      } u_Motor3_Out_Pwm;
      u_Motor3_Out_Pwm.base = 0;
      u_Motor3_Out_Pwm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Motor3_Out_Pwm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Motor3_Out_Pwm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Motor3_Out_Pwm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Motor3_Out_Pwm = u_Motor3_Out_Pwm.real;
      offset += sizeof(this->Motor3_Out_Pwm);
      union {
        int32_t real;
        uint32_t base;
      } u_Motor4_Out_Pwm;
      u_Motor4_Out_Pwm.base = 0;
      u_Motor4_Out_Pwm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Motor4_Out_Pwm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Motor4_Out_Pwm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Motor4_Out_Pwm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Motor4_Out_Pwm = u_Motor4_Out_Pwm.real;
      offset += sizeof(this->Motor4_Out_Pwm);
      union {
        int32_t real;
        uint32_t base;
      } u_Motor1_Expectations;
      u_Motor1_Expectations.base = 0;
      u_Motor1_Expectations.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Motor1_Expectations.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Motor1_Expectations.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Motor1_Expectations.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Motor1_Expectations = u_Motor1_Expectations.real;
      offset += sizeof(this->Motor1_Expectations);
      union {
        int32_t real;
        uint32_t base;
      } u_Motor2_Expectations;
      u_Motor2_Expectations.base = 0;
      u_Motor2_Expectations.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Motor2_Expectations.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Motor2_Expectations.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Motor2_Expectations.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Motor2_Expectations = u_Motor2_Expectations.real;
      offset += sizeof(this->Motor2_Expectations);
      union {
        int32_t real;
        uint32_t base;
      } u_Motor3_Expectations;
      u_Motor3_Expectations.base = 0;
      u_Motor3_Expectations.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Motor3_Expectations.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Motor3_Expectations.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Motor3_Expectations.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Motor3_Expectations = u_Motor3_Expectations.real;
      offset += sizeof(this->Motor3_Expectations);
      union {
        int32_t real;
        uint32_t base;
      } u_Motor4_Expectations;
      u_Motor4_Expectations.base = 0;
      u_Motor4_Expectations.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Motor4_Expectations.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Motor4_Expectations.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Motor4_Expectations.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Motor4_Expectations = u_Motor4_Expectations.real;
      offset += sizeof(this->Motor4_Expectations);
      union {
        int32_t real;
        uint32_t base;
      } u_Motor1_Feedback;
      u_Motor1_Feedback.base = 0;
      u_Motor1_Feedback.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Motor1_Feedback.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Motor1_Feedback.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Motor1_Feedback.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Motor1_Feedback = u_Motor1_Feedback.real;
      offset += sizeof(this->Motor1_Feedback);
      union {
        int32_t real;
        uint32_t base;
      } u_Motor2_Feedback;
      u_Motor2_Feedback.base = 0;
      u_Motor2_Feedback.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Motor2_Feedback.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Motor2_Feedback.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Motor2_Feedback.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Motor2_Feedback = u_Motor2_Feedback.real;
      offset += sizeof(this->Motor2_Feedback);
      union {
        int32_t real;
        uint32_t base;
      } u_Motor3_Feedback;
      u_Motor3_Feedback.base = 0;
      u_Motor3_Feedback.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Motor3_Feedback.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Motor3_Feedback.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Motor3_Feedback.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Motor3_Feedback = u_Motor3_Feedback.real;
      offset += sizeof(this->Motor3_Feedback);
      union {
        int32_t real;
        uint32_t base;
      } u_Motor4_Feedback;
      u_Motor4_Feedback.base = 0;
      u_Motor4_Feedback.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Motor4_Feedback.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Motor4_Feedback.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Motor4_Feedback.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Motor4_Feedback = u_Motor4_Feedback.real;
      offset += sizeof(this->Motor4_Feedback);
     return offset;
    }

    const char * getType(){ return "starrobot_msgs/info_show"; };
    const char * getMD5(){ return "6901243cc3edf28eb57146f6c3eeb9c6"; };

  };

}
#endif
