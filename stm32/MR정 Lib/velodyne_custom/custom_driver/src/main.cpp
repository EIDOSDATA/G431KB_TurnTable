#include "../include/custom_driver/custom_driver.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "custom_driver");

  ROS_INFO("\033[1;32m---->\033[0m Lidar Turn-Table node is Started.");

  VelodyneCustom vc;

  return 0;
}
