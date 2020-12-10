#ifndef _ROS_stryx_localization_Extracts_h
#define _ROS_stryx_localization_Extracts_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/PointCloud2.h"
#include "stryx_localization/ShapeData.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

namespace stryx_localization
{

  class Extracts : public ros::Msg
  {
    public:
      typedef sensor_msgs::PointCloud2 _lidar_type;
      _lidar_type lidar;
      uint32_t shape_length;
      typedef stryx_localization::ShapeData _shape_type;
      _shape_type st_shape;
      _shape_type * shape;
      typedef geometry_msgs::Vector3 _position_type;
      _position_type position;
      typedef geometry_msgs::Quaternion _orientation_type;
      _orientation_type orientation;
      uint32_t ring_start_idx_length;
      typedef int32_t _ring_start_idx_type;
      _ring_start_idx_type st_ring_start_idx;
      _ring_start_idx_type * ring_start_idx;

    Extracts():
      lidar(),
      shape_length(0), shape(NULL),
      position(),
      orientation(),
      ring_start_idx_length(0), ring_start_idx(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->lidar.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->shape_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->shape_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->shape_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->shape_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->shape_length);
      for( uint32_t i = 0; i < shape_length; i++){
      offset += this->shape[i].serialize(outbuffer + offset);
      }
      offset += this->position.serialize(outbuffer + offset);
      offset += this->orientation.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->ring_start_idx_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ring_start_idx_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ring_start_idx_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ring_start_idx_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ring_start_idx_length);
      for( uint32_t i = 0; i < ring_start_idx_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_ring_start_idxi;
      u_ring_start_idxi.real = this->ring_start_idx[i];
      *(outbuffer + offset + 0) = (u_ring_start_idxi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ring_start_idxi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ring_start_idxi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ring_start_idxi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ring_start_idx[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->lidar.deserialize(inbuffer + offset);
      uint32_t shape_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      shape_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      shape_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      shape_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->shape_length);
      if(shape_lengthT > shape_length)
        this->shape = (stryx_localization::ShapeData*)realloc(this->shape, shape_lengthT * sizeof(stryx_localization::ShapeData));
      shape_length = shape_lengthT;
      for( uint32_t i = 0; i < shape_length; i++){
      offset += this->st_shape.deserialize(inbuffer + offset);
        memcpy( &(this->shape[i]), &(this->st_shape), sizeof(stryx_localization::ShapeData));
      }
      offset += this->position.deserialize(inbuffer + offset);
      offset += this->orientation.deserialize(inbuffer + offset);
      uint32_t ring_start_idx_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      ring_start_idx_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      ring_start_idx_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      ring_start_idx_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->ring_start_idx_length);
      if(ring_start_idx_lengthT > ring_start_idx_length)
        this->ring_start_idx = (int32_t*)realloc(this->ring_start_idx, ring_start_idx_lengthT * sizeof(int32_t));
      ring_start_idx_length = ring_start_idx_lengthT;
      for( uint32_t i = 0; i < ring_start_idx_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_ring_start_idx;
      u_st_ring_start_idx.base = 0;
      u_st_ring_start_idx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_ring_start_idx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_ring_start_idx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_ring_start_idx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_ring_start_idx = u_st_ring_start_idx.real;
      offset += sizeof(this->st_ring_start_idx);
        memcpy( &(this->ring_start_idx[i]), &(this->st_ring_start_idx), sizeof(int32_t));
      }
     return offset;
    }

    const char * getType(){ return "stryx_localization/Extracts"; };
    const char * getMD5(){ return "303d6bdafba2b05287b680b98c1b148d"; };

  };

}
#endif
