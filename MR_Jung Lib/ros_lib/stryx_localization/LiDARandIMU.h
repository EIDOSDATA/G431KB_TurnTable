#ifndef _ROS_stryx_localization_LiDARandIMU_h
#define _ROS_stryx_localization_LiDARandIMU_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "stryx_localization/VelodyneCustom.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

namespace stryx_localization
{

  class LiDARandIMU : public ros::Msg
  {
    public:
      typedef stryx_localization::VelodyneCustom _lidar_type;
      _lidar_type lidar;
      typedef float _prev_lidar_last_time_type;
      _prev_lidar_last_time_type prev_lidar_last_time;
      uint32_t acceleration_length;
      typedef geometry_msgs::Vector3 _acceleration_type;
      _acceleration_type st_acceleration;
      _acceleration_type * acceleration;
      uint32_t orientation_length;
      typedef geometry_msgs::Quaternion _orientation_type;
      _orientation_type st_orientation;
      _orientation_type * orientation;
      uint32_t acceleration_time_length;
      typedef float _acceleration_time_type;
      _acceleration_time_type st_acceleration_time;
      _acceleration_time_type * acceleration_time;
      uint32_t orientation_time_length;
      typedef float _orientation_time_type;
      _orientation_time_type st_orientation_time;
      _orientation_time_type * orientation_time;
      typedef geometry_msgs::Vector3 _acc_at_stamp_type;
      _acc_at_stamp_type acc_at_stamp;
      typedef geometry_msgs::Quaternion _ori_at_stamp_type;
      _ori_at_stamp_type ori_at_stamp;

    LiDARandIMU():
      lidar(),
      prev_lidar_last_time(0),
      acceleration_length(0), acceleration(NULL),
      orientation_length(0), orientation(NULL),
      acceleration_time_length(0), acceleration_time(NULL),
      orientation_time_length(0), orientation_time(NULL),
      acc_at_stamp(),
      ori_at_stamp()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->lidar.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->prev_lidar_last_time);
      *(outbuffer + offset + 0) = (this->acceleration_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->acceleration_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->acceleration_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->acceleration_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->acceleration_length);
      for( uint32_t i = 0; i < acceleration_length; i++){
      offset += this->acceleration[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->orientation_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->orientation_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->orientation_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->orientation_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->orientation_length);
      for( uint32_t i = 0; i < orientation_length; i++){
      offset += this->orientation[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->acceleration_time_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->acceleration_time_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->acceleration_time_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->acceleration_time_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->acceleration_time_length);
      for( uint32_t i = 0; i < acceleration_time_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->acceleration_time[i]);
      }
      *(outbuffer + offset + 0) = (this->orientation_time_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->orientation_time_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->orientation_time_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->orientation_time_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->orientation_time_length);
      for( uint32_t i = 0; i < orientation_time_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->orientation_time[i]);
      }
      offset += this->acc_at_stamp.serialize(outbuffer + offset);
      offset += this->ori_at_stamp.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->lidar.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->prev_lidar_last_time));
      uint32_t acceleration_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      acceleration_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      acceleration_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      acceleration_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->acceleration_length);
      if(acceleration_lengthT > acceleration_length)
        this->acceleration = (geometry_msgs::Vector3*)realloc(this->acceleration, acceleration_lengthT * sizeof(geometry_msgs::Vector3));
      acceleration_length = acceleration_lengthT;
      for( uint32_t i = 0; i < acceleration_length; i++){
      offset += this->st_acceleration.deserialize(inbuffer + offset);
        memcpy( &(this->acceleration[i]), &(this->st_acceleration), sizeof(geometry_msgs::Vector3));
      }
      uint32_t orientation_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      orientation_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      orientation_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      orientation_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->orientation_length);
      if(orientation_lengthT > orientation_length)
        this->orientation = (geometry_msgs::Quaternion*)realloc(this->orientation, orientation_lengthT * sizeof(geometry_msgs::Quaternion));
      orientation_length = orientation_lengthT;
      for( uint32_t i = 0; i < orientation_length; i++){
      offset += this->st_orientation.deserialize(inbuffer + offset);
        memcpy( &(this->orientation[i]), &(this->st_orientation), sizeof(geometry_msgs::Quaternion));
      }
      uint32_t acceleration_time_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      acceleration_time_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      acceleration_time_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      acceleration_time_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->acceleration_time_length);
      if(acceleration_time_lengthT > acceleration_time_length)
        this->acceleration_time = (float*)realloc(this->acceleration_time, acceleration_time_lengthT * sizeof(float));
      acceleration_time_length = acceleration_time_lengthT;
      for( uint32_t i = 0; i < acceleration_time_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_acceleration_time));
        memcpy( &(this->acceleration_time[i]), &(this->st_acceleration_time), sizeof(float));
      }
      uint32_t orientation_time_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      orientation_time_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      orientation_time_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      orientation_time_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->orientation_time_length);
      if(orientation_time_lengthT > orientation_time_length)
        this->orientation_time = (float*)realloc(this->orientation_time, orientation_time_lengthT * sizeof(float));
      orientation_time_length = orientation_time_lengthT;
      for( uint32_t i = 0; i < orientation_time_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_orientation_time));
        memcpy( &(this->orientation_time[i]), &(this->st_orientation_time), sizeof(float));
      }
      offset += this->acc_at_stamp.deserialize(inbuffer + offset);
      offset += this->ori_at_stamp.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "stryx_localization/LiDARandIMU"; };
    const char * getMD5(){ return "dd60d3bf4112737f1cee329d6e91af5c"; };

  };

}
#endif
