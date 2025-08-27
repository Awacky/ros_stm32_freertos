#ifndef _ROS_starrobot_msgs_Relaid_h
#define _ROS_starrobot_msgs_Relaid_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace starrobot_msgs
{

  class Relaid : public ros::Msg
  {
    public:
      typedef uint8_t _reType_type;
      _reType_type reType;
      typedef uint8_t _reDLC_type;
      _reDLC_type reDLC;
      typedef uint32_t _reId_type;
      _reId_type reId;
      uint32_t redata_length;
      typedef uint8_t _redata_type;
      _redata_type st_redata;
      _redata_type * redata;

    Relaid():
      reType(0),
      reDLC(0),
      reId(0),
      redata_length(0), redata(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->reType >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reType);
      *(outbuffer + offset + 0) = (this->reDLC >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reDLC);
      *(outbuffer + offset + 0) = (this->reId >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->reId >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->reId >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->reId >> (8 * 3)) & 0xFF;
      offset += sizeof(this->reId);
      *(outbuffer + offset + 0) = (this->redata_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->redata_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->redata_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->redata_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->redata_length);
      for( uint32_t i = 0; i < redata_length; i++){
      *(outbuffer + offset + 0) = (this->redata[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->redata[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->reType =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reType);
      this->reDLC =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reDLC);
      this->reId =  ((uint32_t) (*(inbuffer + offset)));
      this->reId |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->reId |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->reId |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->reId);
      uint32_t redata_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      redata_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      redata_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      redata_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->redata_length);
      if(redata_lengthT > redata_length)
        this->redata = (uint8_t*)realloc(this->redata, redata_lengthT * sizeof(uint8_t));
      redata_length = redata_lengthT;
      for( uint32_t i = 0; i < redata_length; i++){
      this->st_redata =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_redata);
        memcpy( &(this->redata[i]), &(this->st_redata), sizeof(uint8_t));
      }
     return offset;
    }

    const char * getType(){ return "starrobot_msgs/Relaid"; };
    const char * getMD5(){ return "45a445dee17ea524e541cb5d656c4aa8"; };

  };

}
#endif
