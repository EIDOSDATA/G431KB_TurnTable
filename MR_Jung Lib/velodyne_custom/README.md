  How to launch LIDAR turn-table

1. sudo apt-get install ros-VERSION-velodyne

2. sudo apt-get install ros-melodic-rosserial-arduino

3. sudo apt-get install ros-melodic-rosserial

3. sudo usermod -a -G dialout USERNAME (ex. stryx)

4. reboot

5. catkin_make

6. Turn on your lidar

7. Use static ip and connect to lidar (ex. 192.168.1.10)

8. roslaunch custom_driver run.launch

9. rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600 
   (Check your USB port and baudrate)

  Then... Finish!!


+ Opt
  If you want to control your motor, just following below commands.
  1. Stop --> rostopic pub /control_motor std_msgs/String stop
  2. Go   --> rostopic pub /control_motor std_msgs/String go
  3. Slow --> rostopic pub /control_motor std_msgs/String slow
  4. Init --> rostopic pub /control_motor std_msgs/String init  (Only when motor is moving)

