#ifndef _ROS_novatel_gps_msgs_NovatelRawImu_h
#define _ROS_novatel_gps_msgs_NovatelRawImu_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "novatel_gps_msgs/NovatelMessageHeader.h"

namespace novatel_gps_msgs
{

  class NovatelRawImu : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef novatel_gps_msgs::NovatelMessageHeader _novatel_msg_header_type;
      _novatel_msg_header_type novatel_msg_header;
      typedef uint32_t _gps_week_num_type;
      _gps_week_num_type gps_week_num;
      typedef float _gps_seconds_type;
      _gps_seconds_type gps_seconds;
      typedef int32_t _z_acceleration_type;
      _z_acceleration_type z_acceleration;
      typedef int32_t _y_acceleration_type;
      _y_acceleration_type y_acceleration;
      typedef int32_t _x_acceleration_type;
      _x_acceleration_type x_acceleration;
      typedef int32_t _yaw_rate_type;
      _yaw_rate_type yaw_rate;
      typedef int32_t _roll_rate_type;
      _roll_rate_type roll_rate;
      typedef int32_t _pitch_rate_type;
      _pitch_rate_type pitch_rate;

    NovatelRawImu():
      header(),
      novatel_msg_header(),
      gps_week_num(0),
      gps_seconds(0),
      z_acceleration(0),
      y_acceleration(0),
      x_acceleration(0),
      yaw_rate(0),
      roll_rate(0),
      pitch_rate(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->novatel_msg_header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->gps_week_num >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->gps_week_num >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->gps_week_num >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->gps_week_num >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gps_week_num);
      offset += serializeAvrFloat64(outbuffer + offset, this->gps_seconds);
      union {
        int32_t real;
        uint32_t base;
      } u_z_acceleration;
      u_z_acceleration.real = this->z_acceleration;
      *(outbuffer + offset + 0) = (u_z_acceleration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_z_acceleration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_z_acceleration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_z_acceleration.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z_acceleration);
      union {
        int32_t real;
        uint32_t base;
      } u_y_acceleration;
      u_y_acceleration.real = this->y_acceleration;
      *(outbuffer + offset + 0) = (u_y_acceleration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y_acceleration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y_acceleration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y_acceleration.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y_acceleration);
      union {
        int32_t real;
        uint32_t base;
      } u_x_acceleration;
      u_x_acceleration.real = this->x_acceleration;
      *(outbuffer + offset + 0) = (u_x_acceleration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x_acceleration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x_acceleration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x_acceleration.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x_acceleration);
      union {
        int32_t real;
        uint32_t base;
      } u_yaw_rate;
      u_yaw_rate.real = this->yaw_rate;
      *(outbuffer + offset + 0) = (u_yaw_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw_rate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw_rate);
      union {
        int32_t real;
        uint32_t base;
      } u_roll_rate;
      u_roll_rate.real = this->roll_rate;
      *(outbuffer + offset + 0) = (u_roll_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll_rate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll_rate);
      union {
        int32_t real;
        uint32_t base;
      } u_pitch_rate;
      u_pitch_rate.real = this->pitch_rate;
      *(outbuffer + offset + 0) = (u_pitch_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pitch_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pitch_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pitch_rate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pitch_rate);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->novatel_msg_header.deserialize(inbuffer + offset);
      this->gps_week_num =  ((uint32_t) (*(inbuffer + offset)));
      this->gps_week_num |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->gps_week_num |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->gps_week_num |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->gps_week_num);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->gps_seconds));
      union {
        int32_t real;
        uint32_t base;
      } u_z_acceleration;
      u_z_acceleration.base = 0;
      u_z_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_z_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_z_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_z_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->z_acceleration = u_z_acceleration.real;
      offset += sizeof(this->z_acceleration);
      union {
        int32_t real;
        uint32_t base;
      } u_y_acceleration;
      u_y_acceleration.base = 0;
      u_y_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y_acceleration = u_y_acceleration.real;
      offset += sizeof(this->y_acceleration);
      union {
        int32_t real;
        uint32_t base;
      } u_x_acceleration;
      u_x_acceleration.base = 0;
      u_x_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x_acceleration = u_x_acceleration.real;
      offset += sizeof(this->x_acceleration);
      union {
        int32_t real;
        uint32_t base;
      } u_yaw_rate;
      u_yaw_rate.base = 0;
      u_yaw_rate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw_rate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw_rate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw_rate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yaw_rate = u_yaw_rate.real;
      offset += sizeof(this->yaw_rate);
      union {
        int32_t real;
        uint32_t base;
      } u_roll_rate;
      u_roll_rate.base = 0;
      u_roll_rate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll_rate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll_rate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll_rate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->roll_rate = u_roll_rate.real;
      offset += sizeof(this->roll_rate);
      union {
        int32_t real;
        uint32_t base;
      } u_pitch_rate;
      u_pitch_rate.base = 0;
      u_pitch_rate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pitch_rate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pitch_rate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pitch_rate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pitch_rate = u_pitch_rate.real;
      offset += sizeof(this->pitch_rate);
     return offset;
    }

    const char * getType(){ return "novatel_gps_msgs/NovatelRawImu"; };
    const char * getMD5(){ return "6b45a1a7bea7912903601d1cdb0ea4cc"; };

  };

}
#endif
