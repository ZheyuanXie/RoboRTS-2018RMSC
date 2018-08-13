/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef MODULES_DRIVER_SERIAL_SERIAL_COM_NODE_H
#define MODULES_DRIVER_SERIAL_SERIAL_COM_NODE_H

#include <ctime>
#include <cmath>
#include <sys/ioctl.h>
#include <sys/fcntl.h>
#include <sys/termios.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <fstream>
#include <string>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Trigger.h>

#include "common/io.h"
#include "common/rrts.h"
#include "common/log.h"
#include "common/error_code.h"
#include "common/node_state.h"
#include "common/main_interface.h"

#include "messages/GimbalAngle.h"
#include "messages/GameInfo.h"
#include "messages/RobotHurtData.h"
#include "messages/RfidInfo.h"
#include "messages/EnemyPos.h"
#include "messages/ShootState.h"
#include "messages/ShootInfo.h"
#include "rmsc_messages/GripperInfo.h"

#include "messages/GameBuffStatus.h"
#include "messages/ChassisMode.h"
#include "messages/GimbalMode.h"
#include "messages/ShootModeControl.h"
#include "messages/CheckStatus.h"
#include "rmsc_messages/GripperCmd.h"
#include "rmsc_messages/AggressiveGainBuffInfo.h"

#include "modules/driver/serial/infantry_info.h"
#include "modules/driver/serial/proto/serial_com_config.pb.h"

namespace rrts {
namespace driver {
namespace serial {

static const uint16_t kCrc = 0xffff;
static constexpr uint16_t kCrcTable[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
static const unsigned char kCrc8 = 0xff;
static constexpr unsigned char kCrcOctTable[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e,
    0x20, 0xa3, 0xfd, 0x1f, 0x41, 0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2,
    0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 0x23,
    0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03,
    0x80, 0xde, 0x3c, 0x62, 0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63,
    0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 0x46, 0x18,
    0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5,
    0xbb, 0x59, 0x07, 0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
    0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 0x65, 0x3b, 0xd9,
    0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98,
    0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a,
    0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 0x8c, 0xd2, 0x30, 0x6e,
    0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93,
    0xcd, 0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d,
    0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1, 0x13, 0x4d, 0xce,
    0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c,
    0x12, 0x91, 0xcf, 0x2d, 0x73, 0xca, 0x94, 0x76, 0x28, 0xab, 0xf5,
    0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 0x57,
    0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77,
    0xf4, 0xaa, 0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34,
    0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 0x74, 0x2a,
    0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7,
    0x89, 0x6b, 0x35
};

class SerialComNode : public rrts::common::RRTS {

 public:

  /**
   * @brief Constructor
   * @param module_name The name of the serial number
   */
  SerialComNode(std::string module_name);

  /**
   * @brief Run the main process
   */
  void Run();

  ~SerialComNode();

  /**
   * @brief Stop the main process
   */
  void Stop();

  /**
   * @brief Restart the main process
   */
  void Resume();

 private:

  /**
   * @brief Initialize the serial port
   * @return True if success, otherwise false
   */
  bool Initialization();

  /**
   * @brief Fetch the CRC data (1 Byte) from message
   */
  uint8_t GetCrcOctCheckSum(uint8_t *message, uint32_t length, uint8_t crc);

  /**
   * @brief Fetch the CRC data (2 Bytes) from message
   */
  uint16_t GetCrcHexCheckSum(uint8_t *message, uint32_t length, uint16_t crc);

  /**
   * @brief Verify the CRC (1 Byte)
   */
  bool VerifyCrcOctCheckSum(uint8_t *message, uint16_t length);

  /**
   * @brief Verify the CRC (2 Bytes)
   */
  bool VerifyCrcHexCheckSum(uint8_t *message, uint32_t length);

  /**
   * @brief Append CRC (1 Byte) to message
   */
  void AppendCrcOctCheckSum(uint8_t *message, uint16_t length);

  /**
   * @brief Append CRC (2 Bytes) to message
   */
  void AppendCrcHexCheckSum(uint8_t *message, uint32_t length);

  /**
   * @brief Serial port initialize.
   */
  bool SerialInitialization(std::string port, int baudrate, int flow_control, int data_bits, int stop_bits, int parity);

  /**
   * @brief Set the  boudrate of the serial port
   * @param boudrate The value to set
   */
  bool ConfigBaudrate(int boudrate);

  /**
   * @brief The thread function for getting data from embedded platform
   */
  void ReceiveLoop();

  /**
   * @brief Basic receive
   * @param fd The file discription to get data from.
   * @param data_length The max number of data to receive
   * @return The actual number of bytes received
   */
  int ReceiveData(int fd, int data_length);

  /**
   * @brief Unpacking the package message
   */
  void DataHandle();

  /**
   * @brief Pack raw information to message
   */
  void SendDataHandle(uint16_t cmd_id, uint8_t *topack_data, uint8_t *packed_data, uint16_t len);

  /**
   * @brief Send packed data
   */
  int SendData(int data_len);

  /**
   * @brief Callback of enemy position message
   */
  void GimbalRelativeControlCallback(const messages::EnemyPosConstPtr &msg);

  /**
   * @brief Callback of chassis control(cmd_vel) message
   */
  void ChassisControlCallback(const geometry_msgs::Twist::ConstPtr &vel);

  /**
   * @brief Callback of gripper control(cmd_grip) message
   */
  void GripperControlCallback(const rmsc_messages::GripperCmdConstPtr &msg);

  void SendChassisControl(const ChassisControl &chassis_control);

  void SendGimbalControl(const GimbalControl &gimbal_control);

  void SendGripperControl(const GripperControl &gripper_control);

  bool SetChassisMode(messages::ChassisMode::Request &req,
                      messages::ChassisMode::Response &res);

  bool SetGimbalMode(messages::GimbalMode::Request &req,
                      messages::GimbalMode::Response &res);

  bool ShootModeControl(messages::ShootModeControl::Request &req,
                    messages::ShootModeControl::Response &res);

  /**
   * @brief The thread function for sending data.
   */
  void SendPack();

  bool CheckStatusCallback(messages::CheckStatus::Request  &req,
                         messages::CheckStatus::Response &res);

  FILE * fp_;
  int fd_, baudrate_, length_, pack_length_, total_length_, free_length_, key_, valid_key_;
  double length_column_, length_row_, length_beam_;
  struct termios termios_options_, termios_options_original_;
  std::string port_;
  double time_start_;
  std::thread *receive_loop_thread_, *send_loop_thread_, *keyboard_in_;
  std::mutex mutex_receive_, mutex_send_, mutex_pack_;
  bool is_open_, stop_receive_, stop_send_, is_sim_, is_debug_, is_debug_tx_;
  ros::NodeHandle nh_;
  //TODO(krik): use actionlib and add more subscribers, more publishers.
  ros::Subscriber sub_cmd_vel_, sub_cmd_gim_, sub_cmd_shoot_, sub_cmd_grip_;
  ros::Publisher odom_pub_,
      gim_pub_,
      uwb_pose_pub_,
      game_info_pub_,
      robot_hurt_data_pub_,
      rfid_info_pub_,
      game_buff_info_pub_,
      shoot_info_pub_,
      gripper_info_pub_,
      aggressive_gain_buff_info_pub_;

  ros::ServiceClient game_buff_status_srv_;
  ros::ServiceServer chassis_mode_srv_,
                     gimbal_mode_srv_,
                     shoot_mode_srv_,
                     check_status_srv_;

  messages::GameInfo  game_info_msg_;
  messages::RobotHurtData robot_hurt_data_msg_;
  messages::RfidInfo rfid_info_msg_;
  messages::GimbalAngle gim_angle_;
  messages::ShootInfo shoot_info_msg_;
  rmsc_messages::GripperInfo gripper_info_msg_;
  rmsc_messages::AggressiveGainBuffInfo aggressive_gain_buff_info_msg_;
  messages::GameBuffStatus game_buff_status_;
  geometry_msgs::PoseStamped uwb_position_msg_;
  geometry_msgs::TransformStamped arm_tf_;
  tf::TransformBroadcaster tf_broadcaster_;

  ChassisMode chassis_mode_;
  GimbalMode gimbal_mode_;
  ShootControl shoot_control_;

  //TODO(krik): add the error code and node state
  UnpackStep unpack_step_e_;
  uint8_t byte_, rx_buf_[UART_BUFF_SIZE], tx_buf_[UART_BUFF_SIZE],
      protocol_packet_[PROTOCAL_FRAME_MAX_SIZE];
  uint16_t data_length_, computer_cmd_id_;
  int32_t read_len_, read_buff_index_, index_;
  GimbalControl gimbal_control_data_;
  ChassisControl chassis_control_data_;
  FrameHeader computer_frame_header_;
  RobotGameState robot_game_state_;
  RobotHurtData robot_hurt_data_;
  ShootData real_shoot_data_;
  RfidData rfid_data_;
  GameResult game_result_data_;
  RobotPosition robot_position_;
  ChassisInfo chassis_information_;
  GimbalInfo gimbal_information_;
  ShootInfo shoot_task_data_;
  InfantryError global_error_data_;
  GameBuff get_buff_data_;
  ServerToUser student_download_data_;
  ConfigMessage config_response_data_;
  CalibrateResponse cali_response_data_;
  RcInfo rc_info_data_;
  VersionInfo version_info_data_;
  GripperInfo gripper_info_data_;
  AggressiveGameBuff aggressive_gain_buff_info_data_;
};

} //namespace serial
} //namespace driver
} //namespace rrts

#endif //MODULES_DRIVER_SERIAL_SERIAL_COM_NODE_H
