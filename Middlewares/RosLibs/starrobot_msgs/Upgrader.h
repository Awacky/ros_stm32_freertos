#ifndef _ROS_starrobot_msgs_Upgrader_h
#define _ROS_starrobot_msgs_Upgrader_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace starrobot_msgs
{

  class Upgrader : public ros::Msg
  {
    public:
      typedef uint16_t _upCmd_type;
      _upCmd_type upCmd;
      typedef uint16_t _upFrameNum_type;
      _upFrameNum_type upFrameNum;
      typedef uint16_t _upCrc_type;
      _upCrc_type upCrc;
      typedef uint32_t _upPayLen_type;
      _upPayLen_type upPayLen;
      uint32_t upData_length;
      typedef uint8_t _upData_type;
      _upData_type st_upData;
      _upData_type * upData;

    Upgrader():
      upCmd(0),
      upFrameNum(0),
      upCrc(0),
      upPayLen(0),
      upData_length(0), upData(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->upCmd >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->upCmd >> (8 * 1)) & 0xFF;
      offset += sizeof(this->upCmd);
      *(outbuffer + offset + 0) = (this->upFrameNum >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->upFrameNum >> (8 * 1)) & 0xFF;
      offset += sizeof(this->upFrameNum);
      *(outbuffer + offset + 0) = (this->upCrc >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->upCrc >> (8 * 1)) & 0xFF;
      offset += sizeof(this->upCrc);
      *(outbuffer + offset + 0) = (this->upPayLen >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->upPayLen >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->upPayLen >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->upPayLen >> (8 * 3)) & 0xFF;
      offset += sizeof(this->upPayLen);
      *(outbuffer + offset + 0) = (this->upData_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->upData_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->upData_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->upData_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->upData_length);
      for( uint32_t i = 0; i < upData_length; i++){
      *(outbuffer + offset + 0) = (this->upData[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->upData[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->upCmd =  ((uint16_t) (*(inbuffer + offset)));
      this->upCmd |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->upCmd);
      this->upFrameNum =  ((uint16_t) (*(inbuffer + offset)));
      this->upFrameNum |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->upFrameNum);
      this->upCrc =  ((uint16_t) (*(inbuffer + offset)));
      this->upCrc |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->upCrc);
      this->upPayLen =  ((uint32_t) (*(inbuffer + offset)));
      this->upPayLen |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->upPayLen |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->upPayLen |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->upPayLen);
      uint32_t upData_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      upData_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      upData_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      upData_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->upData_length);
      if(upData_lengthT > upData_length)
        this->upData = (uint8_t*)realloc(this->upData, upData_lengthT * sizeof(uint8_t));
      upData_length = upData_lengthT;
      for( uint32_t i = 0; i < upData_length; i++){
      this->st_upData =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_upData);
        memcpy( &(this->upData[i]), &(this->st_upData), sizeof(uint8_t));
      }
     return offset;
    }

    const char * getType(){ return "starrobot_msgs/Upgrader"; };
    const char * getMD5(){ return "42c5dd98002153e141b7375d31263b65"; };

  };

}
#endif
