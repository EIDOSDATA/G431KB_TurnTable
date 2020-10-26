#ifndef custom_driver_CUSTOM_DRIVER_H_
#define custom_driver_CUSTOM_DRIVER_H_

#include <ros/ros.h>

#include <std_msgs/Time.h>
#include <sensor_msgs/PointCloud2.h>
#include <encoder_msgs/EncoderInfo.h>
#include <custom_msgs/VelodyneCustom.h>
#include <velodyne_msgs/VelodynePacket.h>
#include <velodyne_msgs/VelodyneScan.h>

#include "socket.h"
#include "lidar_struct.h"

class VelodyneCustom {

public:
  VelodyneCustom();
  ~VelodyneCustom();

  void LoadParameters();

  void AllocateMemory();

  void SubscribeAndPublish();

  void ThreadInit();

  void CheckEncoderState();

  void EncoderHandler(const encoder_msgs::EncoderInfoConstPtr& EncoderMsg);

  void VelodynePacketHandler(const velodyne_msgs::VelodyneScanConstPtr& VelodynePacketMsg);

  void VelodyneUDPHandler();

  void readData();

  void checkLiDARError(int);

  void ResetLiDARData();

  void PacketToPointcloud(uint8_t packet[1206]);

  void TimeParser(uint8_t time_stamp_data[4], double& toth_time);

  void CreateTimestamp(double toth_time, double& time_stamp);

  void DataBlockParser(uint8_t data_block[100], float& azimuth, std::array<float, 32>& distance, std::array<uint8_t, 32>& intensity);

  void DataPointParser(uint8_t data_point[3], float& distance, uint8_t& intensity);

  void CreateTimetable(double time_stamp, int data_block_idx, std::array<double, 32>& timetable);

  void InterpolateAzimuth(float prev_azimuth, float azimuth, std::array<double, 32> prev_timetable, std::array<double, 32> timetable, std::array<float, 32>& azimuth_itp);

  void CreatePointcloud(std::array<float, 32> prev_distance, std::array<uint8_t, 32> prev_intensity, std::array<float, 32> azimuth_itp, std::array<double, 32> prev_timetable);

  void PublishLiDAR();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  ros::Subscriber subVelodynePackets_;
  ros::Subscriber subEncoderValue_;
  ros::Publisher  pubVelodyneData_;
  ros::Publisher  pubVelodynePointcloud_;

  // Get parameters
  bool                 use_sim_;
  std::string          frame_id_;
  int                  port_;
  float                LiDAR_time_offset_;
  float                LiDAR_roll_offset_;
  float                LiDAR_pitch_offset_;
  float                LiDAR_azimuth_offset_;
  std::vector<float>   vert_angles_;
  std::vector<int>     ring_idx_;

  // LiDAR raw data
  pcl::PointCloud<pcl::PointXYZI>::Ptr RawLiDAR_;
  std::vector<float>                   RawLiDAR_dist_;
  std::vector<float>                   RawLiDAR_azimuth_;
  std::vector<double>                  RawLiDAR_time_;
  std::vector<uint8_t>                 RawLiDAR_ring_;
  std::vector<uint16_t>                RawLiDAR_fire_;
  std::vector<int>                     RawLiDAR_idx_[16];
  uint8_t                              RawLiDAR_hz_;

  // Variables for parsing LiDAR packets and pointcloud creation
  std::array<float, 32>   prev_distance_;
  std::array<float, 32>   azimuth_itp_;
  std::array<double, 32>  prev_timetable_;
  std::array<uint8_t, 32> prev_intensity_;
  float                   prev_azimuth_;
  double                  prev_toth_;
  double                  prev_angle_;

  bool     firstrun_;
  double   time_stamp_;
  double   ros_first_time_stamp_;
  uint16_t firing_num_;

  // LiDAR raw packet variables
  bool device_connected_;
  int  velo_data_;
  int  is_connected_; //0: disconnect, 1: connect, 2: reconnect

  // Encoder and Motor variables
  bool encoder_state_;
  bool encoder_firstrun_;
  bool encoder_update_flag_;

  double encoder_time_stamp_;
  double encoder_prev_time_;
  double encoder_first_time_;
  int    encoder_value_;
  int    encoder_prev_value_;
  int    encoder_first_value_ ;
  int    encoder_revolution_num_;
  double encoder_angular_velocity_;

  bool stop_firstrun_ = true;
  unsigned long azimuth_len = 0;
};

#endif /* custom_driver_CUSTOM_DRIVER_H_ */
