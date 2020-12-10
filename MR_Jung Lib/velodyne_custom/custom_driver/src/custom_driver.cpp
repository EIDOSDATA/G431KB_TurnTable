#include "../include/custom_driver/custom_driver.h"

VelodyneCustom::VelodyneCustom() :
  priv_nh_("~"),
  use_sim_(false),
  frame_id_(""),
  port_(0),
  LiDAR_time_offset_(0),
  LiDAR_roll_offset_(0),
  LiDAR_pitch_offset_(0),
  LiDAR_azimuth_offset_(0),
  prev_toth_(-1),
  prev_angle_(0),
  firstrun_(true),
  device_connected_(false),
  is_connected_(0),
  encoder_state_(true),
  encoder_firstrun_(true),
  encoder_update_flag_(false),
  encoder_time_stamp_ (0),
  encoder_prev_time_  (0),
  encoder_first_time_ (0),
  encoder_value_      (0),
  encoder_prev_value_ (0),
  encoder_first_value_(0),
  encoder_revolution_num_  (0),
  encoder_angular_velocity_(0)
{
  LoadParameters();
  AllocateMemory();
  SubscribeAndPublish();
  ThreadInit();
}

VelodyneCustom::~VelodyneCustom()
{}

void VelodyneCustom::LoadParameters()
{
  nh_.getParam("VLP16VertAng", vert_angles_);
  nh_.getParam("VLP16ChId", ring_idx_);

  priv_nh_.getParam("use_sim", use_sim_);
  priv_nh_.getParam("frame_id", frame_id_);
  priv_nh_.getParam("port", port_);
  priv_nh_.getParam("LiDAR_time_offset", LiDAR_time_offset_);
  priv_nh_.getParam("LiDAR_roll_offset", LiDAR_roll_offset_);
  priv_nh_.getParam("LiDAR_pitch_offset", LiDAR_pitch_offset_);
  priv_nh_.getParam("LiDAR_azimuth_offset", LiDAR_azimuth_offset_);
}

void VelodyneCustom::AllocateMemory()
{
  RawLiDAR_.reset(new pcl::PointCloud<pcl::PointXYZI>());
}

void VelodyneCustom::SubscribeAndPublish()
{
  subVelodynePackets_ = nh_.subscribe<velodyne_msgs::VelodyneScan>("/velodyne_packets", 10, &VelodyneCustom::VelodynePacketHandler, this);
  subEncoderValue_ = nh_.subscribe<encoder_msgs::EncoderInfo>("/encoder_info", 10, &VelodyneCustom::EncoderHandler, this);

  pubVelodyneData_ = nh_.advertise<custom_msgs::VelodyneCustom> ("/velodyne_custom", 10);
  pubVelodynePointcloud_ = nh_.advertise<sensor_msgs::PointCloud2> ("/raw_lidar", 10);
}

void VelodyneCustom::ThreadInit()
{
  std::thread packet_thread(&VelodyneCustom::VelodyneUDPHandler, this);
  std::thread state_thread(&VelodyneCustom::CheckEncoderState, this);

  packet_thread.join();
  state_thread.join();
}

void VelodyneCustom::CheckEncoderState()
{
  int state_cnt = 0;
  int temp_value = 0;

  while( ros::ok() )
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if(temp_value == encoder_value_)
      state_cnt++;
    else
      state_cnt = 0;

    if(state_cnt >= 3)
    {
      encoder_angular_velocity_ = 0;
      encoder_state_ = false;
    }
    else
      encoder_state_ = true;

    temp_value = encoder_value_;
  }
}

void VelodyneCustom::EncoderHandler(const encoder_msgs::EncoderInfoConstPtr &EncoderMsg)
{
  uint32_t temp_encoder_time = EncoderMsg->timeStamp;
  double encoder_time_stamp = temp_encoder_time/1000.0;
  encoder_value_ = EncoderMsg->encoderValue;

  if (encoder_firstrun_)
  {
    encoder_first_value_ = encoder_value_;
    encoder_first_time_ = encoder_time_stamp;
    encoder_prev_time_ = encoder_first_time_;
    encoder_prev_value_ = encoder_first_value_;
    encoder_firstrun_ = false;
    return;
  }

  int value_diff = encoder_value_ - encoder_prev_value_;
  if (value_diff < 0)
  {
    value_diff += 2169;
    encoder_revolution_num_++;
  }

  encoder_angular_velocity_ *= 0.9;
  encoder_angular_velocity_ += static_cast<double>(((value_diff * 2. * M_PI / 2169.) / (encoder_time_stamp - encoder_prev_time_)) * 0.1);

  encoder_prev_value_ = encoder_value_;
  encoder_prev_time_ = encoder_time_stamp;
  encoder_update_flag_ = true;
}

void VelodyneCustom::VelodynePacketHandler(const velodyne_msgs::VelodyneScanConstPtr &VelodynePacketMsg)
{
  // Only handles VLP-16 single return mode
  if (firstrun_)
  {
    ros_first_time_stamp_ = (VelodynePacketMsg->packets[0].stamp.sec + VelodynePacketMsg->packets[0].stamp.nsec / pow(10, 9));
    ros_first_time_stamp_ += LiDAR_time_offset_;
    firstrun_ = false;
  }

  size_t total_packets = VelodynePacketMsg->packets.size();
  RawLiDAR_hz_ = (int) 18000 / ((float) ((total_packets - 1) * 24));

  ResetLiDARData();

  for (size_t packet_idx = 0; packet_idx < total_packets; ++packet_idx)
  {
    uint8_t packet[1206];
    for (size_t i = 0; i < 1206; ++i)
    {
      packet[i] = VelodynePacketMsg->packets[packet_idx].data[i];
    }
    PacketToPointcloud(packet);
  }

  std_msgs::Time time_publish;
  time_publish.data = VelodynePacketMsg->packets[0].stamp;
  PublishLiDAR();
}

void VelodyneCustom::VelodyneUDPHandler()
{
  //ros::Rate loop_rate(1000);
  while ( ros::ok() )
  {
    try
    {
      if (Network::createUDPServer(velo_data_, port_))
      {
        // Connected to device. Begin reading&processing data
        if(is_connected_ == 0)
        {
          ROS_INFO("Open UDP port %d", port_);
          is_connected_ = true;
        }

        while (is_connected_ && ros::ok())
        {
          // Read data from the device
          readData();

          ros::spinOnce();
          //loop_rate.sleep();
        }
      }
      else
      {
        ROS_ERROR("Error connecting to device");
      }
    }
    catch (std::string& error)
    {
      std::cout << error << std::endl;
      //return;
    }

    if (ros::ok())
    {
      //Reconnect delay
      ROS_WARN("reconnecting device..");
      ros::Duration(0.5).sleep();
      is_connected_ = 2;
    }

    ros::spinOnce();
    //loop_rate.sleep();
  }

  close(velo_data_);

  ROS_INFO("Disconnected and shut down");
}

void VelodyneCustom::readData()
{
  uint8_t raw_packet[1206];

  int len = recvfrom(velo_data_, &raw_packet, sizeof(raw_packet), 0, (struct sockaddr *)nullptr, nullptr);

  checkLiDARError(len);

  if (firstrun_)
  {
    ros::Time current = ros::Time::now();
    ros_first_time_stamp_ = (current.sec + current.nsec / pow(10, 9));
    ros_first_time_stamp_ += static_cast<double>(LiDAR_time_offset_);
    firstrun_ = false;
  }

  if(encoder_state_)
  {
    if(RawLiDAR_time_.size() > 2500000)
    {
      ResetLiDARData();
      encoder_firstrun_ = true;
    }

    PacketToPointcloud(raw_packet);

    if ( (abs(encoder_value_-encoder_first_value_) <= 10) && encoder_update_flag_ && RawLiDAR_time_.size() > 100000)
    {
      PublishLiDAR();
      ResetLiDARData();
      encoder_update_flag_ = false;
    }
  }
  else
  {
    if(stop_firstrun_)
    {
      ResetLiDARData();
      stop_firstrun_ = false;
    }

    PacketToPointcloud(raw_packet);

    if(azimuth_len != 0 && azimuth_len != RawLiDAR_azimuth_.size())
    {
      if( (RawLiDAR_azimuth_.at(azimuth_len-1) <= RawLiDAR_azimuth_.front()) && (RawLiDAR_azimuth_.back() >= RawLiDAR_azimuth_.front()))
      {
        PublishLiDAR();
        ResetLiDARData();
      }
    }

    azimuth_len = RawLiDAR_azimuth_.size();
  }
}

void VelodyneCustom::checkLiDARError(int len)
{
  if (len <= 0)
  {
    ROS_ERROR("Can't recieve from velo data");
    is_connected_ = 0;
    device_connected_ = false;
    return;
  }
  if (len != 1206)
  {
    ROS_ERROR("Lidar data packet doesn't matched");
    return;
  }

  if(!device_connected_)
  {
    ROS_INFO("Device connected");
    device_connected_ = true;
  }
}

void VelodyneCustom::ResetLiDARData()
{
  RawLiDAR_->clear();
  RawLiDAR_dist_.clear();
  RawLiDAR_azimuth_.clear();
  RawLiDAR_time_.clear();
  RawLiDAR_ring_.clear();
  RawLiDAR_fire_.clear();
  for (size_t i = 0; i < 16; ++i)
    RawLiDAR_idx_[i].clear();
  firing_num_ = 0;
}

void VelodyneCustom::PacketToPointcloud(uint8_t packet[])
{
  double toth_time;
  TimeParser(&packet[1200], toth_time);
  CreateTimestamp(toth_time, time_stamp_);

  for (size_t data_block_idx = 0; data_block_idx < 12; ++data_block_idx){ //12 data blocks in a single packet
    int starting_point = data_block_idx * 100;
    float azimuth;
    std::array<float, 32> distance;
    std::array<double, 32> timetable;
    std::array<uint8_t, 32> intensity;
    DataBlockParser(&packet[starting_point], azimuth, distance, intensity);
    CreateTimetable(time_stamp_, data_block_idx, timetable);
    InterpolateAzimuth(prev_azimuth_, azimuth, prev_timetable_, timetable, azimuth_itp_);
    CreatePointcloud(prev_distance_, prev_intensity_, azimuth_itp_, prev_timetable_);

    prev_azimuth_ = azimuth;
    prev_distance_ = distance;
    prev_intensity_ = intensity;
    prev_timetable_= timetable;
  }
}

void VelodyneCustom::TimeParser(uint8_t time_stamp_data[], double &toth_time)
{
  uint32_t time_int = time_stamp_data[0] + time_stamp_data[1] * pow(2, 8) + time_stamp_data[2] * pow(2, 16) + time_stamp_data[3] * pow(2, 24);
  toth_time = time_int / pow(10, 6);
}

void VelodyneCustom::CreateTimestamp(double toth_time, double &time_stamp)
{
  if (prev_toth_ == -1)
  {
    time_stamp = ros_first_time_stamp_;
    prev_toth_ = toth_time;
    return;
  }

  double time_diff = toth_time - prev_toth_;
  if (time_diff < 0) {time_diff += 3600;} // when top of the hour changes
  time_stamp += time_diff;
  prev_toth_ = toth_time;
}

void VelodyneCustom::DataBlockParser(uint8_t data_block[], float &azimuth, std::array<float, 32> &distance, std::array<uint8_t, 32> &intensity)
{
  azimuth = data_block[2] + data_block[3] * 256;
  azimuth /= 100;

  for (size_t data_point_idx = 0; data_point_idx < 32; ++data_point_idx){ //32 data points in a single data block
    int starting_point = data_point_idx * 3 + 4;
    DataPointParser(&data_block[starting_point], distance[data_point_idx], intensity[data_point_idx]);
  }
}

void VelodyneCustom::DataPointParser(uint8_t data_point[], float &distance, uint8_t &intensity)
{
  distance = data_point[0] + data_point[1] * 256;
  distance *= 2;
  distance /= 1000;

  intensity = data_point[2];
}

void VelodyneCustom::CreateTimetable(double time_stamp, int data_block_idx, std::array<double, 32> &timetable)
{
  double sequence_offset = 55296 / pow(10, 9), data_point_offset = 2304 / pow(10, 9);

  for (size_t data_seq_idx = 0; data_seq_idx < 2; ++data_seq_idx)
  {
    int firing_seq_idx = data_block_idx * 2 + data_seq_idx;
    for (size_t channel_idx = 0; channel_idx < 16; ++channel_idx)
    {
      int data_point_idx = channel_idx + 16 * data_seq_idx;
      timetable[data_point_idx] = time_stamp + sequence_offset * firing_seq_idx + data_point_offset * channel_idx;
    }
  }
}

void VelodyneCustom::InterpolateAzimuth(float prev_azimuth, float azimuth, std::array<double, 32> prev_timetable, std::array<double, 32> timetable, std::array<float, 32> &azimuth_itp)
{
  if (azimuth < prev_azimuth)
    azimuth += 360;

  for (size_t data_seq_idx = 0; data_seq_idx < 2; ++data_seq_idx)
  {
    for (size_t channel_idx = 0; channel_idx < 16; ++channel_idx)
    {
      int data_point_idx = channel_idx + 16 * data_seq_idx;
      float ratio[2];
      ratio[0] = (prev_timetable[data_point_idx] - prev_timetable[0]) / (timetable[0] - prev_timetable[0]);
      ratio[1] = (timetable[0] - prev_timetable[data_point_idx]) / (timetable[0] - prev_timetable[0]);

      azimuth_itp[data_point_idx] = prev_azimuth * ratio[1] + azimuth * ratio[0];

      if (azimuth_itp[data_point_idx] >= 360)
        azimuth_itp[data_point_idx] -= 360;
    }
  }
}

void VelodyneCustom::CreatePointcloud(std::array<float, 32> prev_distance, std::array<uint8_t, 32> prev_intensity, std::array<float, 32> azimuth_itp, std::array<double, 32> prev_timetable)
{
  for (size_t data_seq_idx = 0; data_seq_idx < 2; ++data_seq_idx)
  {
    for (size_t channel_idx = 0; channel_idx < 16; ++channel_idx)
    {
      int data_point_idx = channel_idx + 16 * data_seq_idx;

      if (prev_distance[data_point_idx] != 0)
      {
        pcl::PointXYZI point;
        point.x = prev_distance[data_point_idx] * cos(vert_angles_[channel_idx] * M_PI / 180.0) * sin((azimuth_itp[data_point_idx] + LiDAR_azimuth_offset_) * M_PI / 180.0);
        point.y = prev_distance[data_point_idx] * cos(vert_angles_[channel_idx] * M_PI / 180.0) * cos((azimuth_itp[data_point_idx] + LiDAR_azimuth_offset_) * M_PI / 180.0);
        point.z = prev_distance[data_point_idx] * sin(vert_angles_[channel_idx] * M_PI / 180.0);
        point.intensity = prev_intensity[data_point_idx];

        float temp = point.y;
        point.y = cos(LiDAR_pitch_offset_ * M_PI / 180.0) * point.y - sin(LiDAR_pitch_offset_ * M_PI / 180.0) * point.z;
        point.z = sin(LiDAR_pitch_offset_ * M_PI / 180.0) * temp + cos(LiDAR_pitch_offset_ * M_PI / 180.0) * point.z;

        temp = point.x;
        double angle;
        if (RawLiDAR_time_.size() == 0)
          angle = 0;
        else
          angle = encoder_angular_velocity_ * (prev_timetable[data_point_idx] - RawLiDAR_time_.back()) + prev_angle_;

        point.x = cos(-angle) * point.x - sin(-angle) * point.y;
        point.y = sin(-angle) * temp + cos(-angle) * point.y;
        prev_angle_ = angle;

        RawLiDAR_->push_back(point);
        RawLiDAR_dist_.push_back(prev_distance[data_point_idx]);
        RawLiDAR_azimuth_.push_back(azimuth_itp[data_point_idx]);
        RawLiDAR_time_.push_back(prev_timetable[data_point_idx]);
        RawLiDAR_ring_.push_back(ring_idx_[channel_idx]);
        RawLiDAR_fire_.push_back(firing_num_);
        RawLiDAR_idx_[ring_idx_[channel_idx]].push_back(RawLiDAR_->points.size() - 1);
      }
    }
    firing_num_++;
  }
}

void VelodyneCustom::PublishLiDAR()
{
  custom_msgs::VelodyneCustom result;
  sensor_msgs::PointCloud2 laserCloudTemp;
  pcl::toROSMsg(*RawLiDAR_, laserCloudTemp);

  laserCloudTemp.header.stamp = ros::Time::now();
  laserCloudTemp.header.frame_id = frame_id_;
  result.pointcloud = laserCloudTemp;
  result.distance = RawLiDAR_dist_;
  result.azimuth = RawLiDAR_azimuth_;
  result.time = RawLiDAR_time_;
  result.ring = RawLiDAR_ring_;

  for (int i = 0; i < 16; ++i)
    result.index[i].data = RawLiDAR_idx_[i];

  result.fire = RawLiDAR_fire_;
  result.frequency = RawLiDAR_hz_;

  pubVelodyneData_.publish(result);

  if (pubVelodynePointcloud_.getNumSubscribers() != 0)
  {
    pcl::toROSMsg(*RawLiDAR_, laserCloudTemp);
    laserCloudTemp.header.stamp = ros::Time::now();
    laserCloudTemp.header.frame_id = frame_id_;
    pubVelodynePointcloud_.publish(laserCloudTemp);
  }
}
