#ifndef _ROS_stryx_localization_ShapeData_h
#define _ROS_stryx_localization_ShapeData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace stryx_localization
{

  class ShapeData : public ros::Msg
  {
    public:
      uint32_t data_length;
      typedef float _data_type;
      _data_type st_data;
      _data_type * data;
      uint32_t index_length;
      typedef int32_t _index_type;
      _index_type st_index;
      _index_type * index;

    ShapeData():
      data_length(0), data(NULL),
      index_length(0), index(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_length);
      for( uint32_t i = 0; i < data_length; i++){
      union {
        float real;
        uint32_t base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_datai.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_datai.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_datai.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      *(outbuffer + offset + 0) = (this->index_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->index_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->index_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->index_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->index_length);
      for( uint32_t i = 0; i < index_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_indexi;
      u_indexi.real = this->index[i];
      *(outbuffer + offset + 0) = (u_indexi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_indexi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_indexi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_indexi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->index[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->data_length);
      if(data_lengthT > data_length)
        this->data = (float*)realloc(this->data, data_lengthT * sizeof(float));
      data_length = data_lengthT;
      for( uint32_t i = 0; i < data_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_data;
      u_st_data.base = 0;
      u_st_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_data = u_st_data.real;
      offset += sizeof(this->st_data);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(float));
      }
      uint32_t index_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      index_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      index_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      index_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->index_length);
      if(index_lengthT > index_length)
        this->index = (int32_t*)realloc(this->index, index_lengthT * sizeof(int32_t));
      index_length = index_lengthT;
      for( uint32_t i = 0; i < index_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_index;
      u_st_index.base = 0;
      u_st_index.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_index.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_index.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_index.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_index = u_st_index.real;
      offset += sizeof(this->st_index);
        memcpy( &(this->index[i]), &(this->st_index), sizeof(int32_t));
      }
     return offset;
    }

    const char * getType(){ return "stryx_localization/ShapeData"; };
    const char * getMD5(){ return "07b7dafaf9ae702ad0ca6ba2507a5a3d"; };

  };

}
#endif
