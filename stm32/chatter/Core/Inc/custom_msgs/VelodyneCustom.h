#ifndef _ROS_custom_msgs_VelodyneCustom_h
#define _ROS_custom_msgs_VelodyneCustom_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/PointCloud2.h"
#include "custom_msgs/RingIndex.h"

namespace custom_msgs
{

  class VelodyneCustom : public ros::Msg
  {
    public:
      typedef sensor_msgs::PointCloud2 _pointcloud_type;
      _pointcloud_type pointcloud;
      uint32_t distance_length;
      typedef float _distance_type;
      _distance_type st_distance;
      _distance_type * distance;
      uint32_t azimuth_length;
      typedef float _azimuth_type;
      _azimuth_type st_azimuth;
      _azimuth_type * azimuth;
      uint32_t time_length;
      typedef double _time_type;
      _time_type st_time;
      _time_type * time;
      uint32_t ring_length;
      typedef uint8_t _ring_type;
      _ring_type st_ring;
      _ring_type * ring;
      custom_msgs::RingIndex index[16];
      uint32_t fire_length;
      typedef uint16_t _fire_type;
      _fire_type st_fire;
      _fire_type * fire;
      typedef uint8_t _frequency_type;
      _frequency_type frequency;

    VelodyneCustom():
      pointcloud(),
      distance_length(0), distance(NULL),
      azimuth_length(0), azimuth(NULL),
      time_length(0), time(NULL),
      ring_length(0), ring(NULL),
      index(),
      fire_length(0), fire(NULL),
      frequency(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pointcloud.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->distance_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->distance_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->distance_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->distance_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distance_length);
      for( uint32_t i = 0; i < distance_length; i++){
      union {
        float real;
        uint32_t base;
      } u_distancei;
      u_distancei.real = this->distance[i];
      *(outbuffer + offset + 0) = (u_distancei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_distancei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_distancei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_distancei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distance[i]);
      }
      *(outbuffer + offset + 0) = (this->azimuth_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->azimuth_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->azimuth_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->azimuth_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->azimuth_length);
      for( uint32_t i = 0; i < azimuth_length; i++){
      union {
        float real;
        uint32_t base;
      } u_azimuthi;
      u_azimuthi.real = this->azimuth[i];
      *(outbuffer + offset + 0) = (u_azimuthi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_azimuthi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_azimuthi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_azimuthi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->azimuth[i]);
      }
      *(outbuffer + offset + 0) = (this->time_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_length);
      for( uint32_t i = 0; i < time_length; i++){
      union {
        double real;
        uint64_t base;
      } u_timei;
      u_timei.real = this->time[i];
      *(outbuffer + offset + 0) = (u_timei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_timei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_timei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_timei.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_timei.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_timei.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_timei.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_timei.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->time[i]);
      }
      *(outbuffer + offset + 0) = (this->ring_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ring_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ring_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ring_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ring_length);
      for( uint32_t i = 0; i < ring_length; i++){
      *(outbuffer + offset + 0) = (this->ring[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ring[i]);
      }
      for( uint32_t i = 0; i < 16; i++){
      offset += this->index[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->fire_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fire_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->fire_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->fire_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fire_length);
      for( uint32_t i = 0; i < fire_length; i++){
      *(outbuffer + offset + 0) = (this->fire[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fire[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->fire[i]);
      }
      *(outbuffer + offset + 0) = (this->frequency >> (8 * 0)) & 0xFF;
      offset += sizeof(this->frequency);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pointcloud.deserialize(inbuffer + offset);
      uint32_t distance_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      distance_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      distance_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      distance_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->distance_length);
      if(distance_lengthT > distance_length)
        this->distance = (float*)realloc(this->distance, distance_lengthT * sizeof(float));
      distance_length = distance_lengthT;
      for( uint32_t i = 0; i < distance_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_distance;
      u_st_distance.base = 0;
      u_st_distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_distance = u_st_distance.real;
      offset += sizeof(this->st_distance);
        memcpy( &(this->distance[i]), &(this->st_distance), sizeof(float));
      }
      uint32_t azimuth_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      azimuth_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      azimuth_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      azimuth_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->azimuth_length);
      if(azimuth_lengthT > azimuth_length)
        this->azimuth = (float*)realloc(this->azimuth, azimuth_lengthT * sizeof(float));
      azimuth_length = azimuth_lengthT;
      for( uint32_t i = 0; i < azimuth_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_azimuth;
      u_st_azimuth.base = 0;
      u_st_azimuth.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_azimuth.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_azimuth.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_azimuth.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_azimuth = u_st_azimuth.real;
      offset += sizeof(this->st_azimuth);
        memcpy( &(this->azimuth[i]), &(this->st_azimuth), sizeof(float));
      }
      uint32_t time_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      time_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      time_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      time_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->time_length);
      if(time_lengthT > time_length)
        this->time = (double*)realloc(this->time, time_lengthT * sizeof(double));
      time_length = time_lengthT;
      for( uint32_t i = 0; i < time_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_time;
      u_st_time.base = 0;
      u_st_time.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_time.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_time.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_time.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_time.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_time.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_time.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_time.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_time = u_st_time.real;
      offset += sizeof(this->st_time);
        memcpy( &(this->time[i]), &(this->st_time), sizeof(double));
      }
      uint32_t ring_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      ring_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      ring_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      ring_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->ring_length);
      if(ring_lengthT > ring_length)
        this->ring = (uint8_t*)realloc(this->ring, ring_lengthT * sizeof(uint8_t));
      ring_length = ring_lengthT;
      for( uint32_t i = 0; i < ring_length; i++){
      this->st_ring =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_ring);
        memcpy( &(this->ring[i]), &(this->st_ring), sizeof(uint8_t));
      }
      for( uint32_t i = 0; i < 16; i++){
      offset += this->index[i].deserialize(inbuffer + offset);
      }
      uint32_t fire_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      fire_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      fire_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      fire_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->fire_length);
      if(fire_lengthT > fire_length)
        this->fire = (uint16_t*)realloc(this->fire, fire_lengthT * sizeof(uint16_t));
      fire_length = fire_lengthT;
      for( uint32_t i = 0; i < fire_length; i++){
      this->st_fire =  ((uint16_t) (*(inbuffer + offset)));
      this->st_fire |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->st_fire);
        memcpy( &(this->fire[i]), &(this->st_fire), sizeof(uint16_t));
      }
      this->frequency =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->frequency);
     return offset;
    }

    const char * getType(){ return "custom_msgs/VelodyneCustom"; };
    const char * getMD5(){ return "f6a5c89e46ccd723fa585f19027bdca2"; };

  };

}
#endif
