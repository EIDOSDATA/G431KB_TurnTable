#ifndef _ROS_encoder_msgs_EncoderInfo_h
#define _ROS_encoder_msgs_EncoderInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace encoder_msgs
{

  class EncoderInfo : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint16_t _encoderValue_type;
      _encoderValue_type encoderValue;
      typedef uint32_t _timeStamp_type;
      _timeStamp_type timeStamp;

    EncoderInfo():
      header(),
      encoderValue(0),
      timeStamp(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->encoderValue >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->encoderValue >> (8 * 1)) & 0xFF;
      offset += sizeof(this->encoderValue);
      *(outbuffer + offset + 0) = (this->timeStamp >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timeStamp >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timeStamp >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timeStamp >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timeStamp);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->encoderValue =  ((uint16_t) (*(inbuffer + offset)));
      this->encoderValue |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->encoderValue);
      this->timeStamp =  ((uint32_t) (*(inbuffer + offset)));
      this->timeStamp |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timeStamp |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timeStamp |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timeStamp);
     return offset;
    }

    const char * getType(){ return "encoder_msgs/EncoderInfo"; };
    const char * getMD5(){ return "27829a094d6154f18e7aa7089e773b7a"; };

  };

}
#endif
