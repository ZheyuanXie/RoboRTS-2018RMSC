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

#ifndef MODULE_DECISION_BEHAVIOR_TREE_BLACKBOARD_H
#define MODULE_DECISION_BEHAVIOR_TREE_BLACKBOARD_H

#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>

#include "messages/ArmorDetectionAction.h"
#include "messages/LocalizationAction.h"
#include "messages/EnemyDirectionAction.h"
#include "messages/GameInfo.h"
#include "messages/RobotHurtData.h"
#include "messages/RfidInfo.h"
#include "messages/ShootInfo.h"
#include "messages/GameBuffStatus.h"
#include "messages/ChassisMode.h"
#include "messages/GimbalMode.h"
#include "messages/ShootModeControl.h"
#include "messages/GoalTask.h"
#include "messages/SelfCheck.h"
#include "rmsc_messages/AmmoDetect.h"
#include "rmsc_messages/ObstacleScanAction.h"
#include "rmsc_messages/MapMuxAction.h"
#include "rmsc_messages/SeerAction.h"

#include "common/io.h"
#include "rmsc/decision_v1/proto/decision.pb.h"
#include "modules/perception/map/costmap/costmap_interface.h"
#include "sound_play/sound_play.h"

namespace rrts{
namespace decision {

enum class GameProcess{
  NOT_START = 0,
  PREPARATION = 1,
  SELF_CHECK = 2,
  COUNTDOWN = 3,
  FIGHT = 4,
  RESULT = 5
};

enum class ArmorAttacked{
  NONE = 0,
  FRONT = 1,
  LEFT = 2,
  BACK = 3,
  RIGHT = 4
};

enum class BuffStatus{
  NONE = 0,
  SELF = 1,
  ENEMY = 2
};

enum class GimbalMode{
  GIMBAL_RELAX = 0,
  GIMBAL_INIT = 1,
  GIMBAL_NO_ARTI_INPUT = 2,
  GIMBAL_FOLLOW_ZGYRO = 3,
  GIMBAL_TRACK_ARMOR = 4,
  GIMBAL_PATROL_MODE = 5,
  GIMBAL_SHOOT_BUFF = 6,
  GIMBAL_POSITION_MODE = 7,
  GIMBAL_RELATIVE_MODE = 8
};

enum class ChassisMode{
  CHASSIS_RELAX = 0,
  CHASSIS_STOP = 1,
  MANUAL_SEPARATE_GIMBAL = 2,
  MANUAL_FOLLOW_GIMBAL = 3,
  DODGE_MODE = 4,
  AUTO_SEPARATE_GIMBAL = 5,
  AUTO_FOLLOW_GIMBAL = 6,

};

enum class ColorDetected {
  NONE = 0,
  BACK = 1,
  LEFT = 2,
  RIGHT = 3,
  FRONT = 4
};


class Blackboard {
 public:
  typedef std::shared_ptr<Blackboard> Ptr;
  explicit Blackboard(const std::string &proto_file_path):
      game_process_(GameProcess::NOT_START),
      remain_hp_(2000),
      last_hp_(2000),
      dmp_(0),
      armor_attacked_(ArmorAttacked::NONE),
      color_detected_(ColorDetected::NONE),
      rfid_active_(false),
      buff_status_(BuffStatus::NONE),
      remain_bullet_(200),
      sent_bullet_(0),
      enemy_detected_(false),
      gimbal_mode_(GimbalMode::GIMBAL_RELAX),
      chassis_mode_(ChassisMode::CHASSIS_RELAX),
      shoot_control_(false),
      auxiliary_(false),
      arrive_(false),
      localization_actionlib_client_("localization_node_action", true),
      armor_detection_actionlib_client_("armor_detection_node_action", true),
      enemy_direction_actionlib_client_("color_detection_node_action", true),
      // obstacle_scan_actionlib_client_("obstacle_scan_node_action", true),
      map_mux_actionlib_client_("map_mux_node_action", true),
      seer_actionlib_client_("seer_action",true),
      ammobox_collected_cnt(0){
    
    rrts::decision::DecisionConfig decision_config;
    rrts::common::ReadProtoFromTextFile(proto_file_path, &decision_config);
    ros::NodeHandle nh;

    LoadInitialAmmoList(decision_config);

    last_get_hp_time_ = ros::Time::now();

    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
    // Enemy fake pose
    ros::NodeHandle rviz_nh("/move_base_simple");
    enemy_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &Blackboard::EnemyCallback, this);

    //goal task info
    ros::NodeHandle goal_task_nh;

    // Referee system inf
    ros::NodeHandle referee_nh("referee_system");

    /* game_info (fake_game_info if use_referee is set to false):
     *  game_process uint 0,1,2,3,4,5
     *  remain_hp uint 0 ~ max_hp
     */
    if (decision_config.use_referee()){
      game_info_sub_ = referee_nh.subscribe("game_info", 30, &Blackboard::GameInfoCallback, this);
    }
    else {
      game_info_sub_ = nh.subscribe("fake_game_info", 30, &Blackboard::GameInfoCallback, this);
    }

    /* robot_hurt_data:
     *  armor attacked uint  0, 1,2,3,4
     */
    robot_hurt_data_sub_ = referee_nh.subscribe("robot_hurt_data", 30, &Blackboard::RobotHurtCallback, this);

    /* rfid_info:
     *  rfid_active bool  0,1
     */
    rfid_info_sub_ = referee_nh.subscribe("rfid_info", 30, &Blackboard::RfidInfoCallback, this);

    /* shoot_info:
     *  remain_bullet uint 0 ~ max_bullet
     */
    shoot_info_sub_ = referee_nh.subscribe("shoot_info", 30, &Blackboard::ShootInfoCallback, this);

    /* set_buff_status:
     *  buff_status uint  0,1,2
     */
    game_buff_status_server_ = referee_nh.advertiseService("set_buff_status", &Blackboard::GameBuffStatusCallback, this);

    self_check_client_ = nh.serviceClient<messages::SelfCheck>("self_check");
    chassis_mode_client_ = nh.serviceClient<messages::ChassisMode>("set_chassis_mode");
    gimbal_mode_client_ = nh.serviceClient<messages::GimbalMode>("set_gimbal_mode");
    shoot_control_client_ = nh.serviceClient<messages::ShootModeControl>("shoot_mode_control");
    track_pub_=nh.advertise<geometry_msgs::PoseStamped>("track_pose",100);

    if (!decision_config.simulate()){
      LOG_INFO << "Waiting for Color & Armor detection module...";
      enemy_direction_actionlib_client_.waitForServer();
      LOG_INFO << "Color detection module has been connected!";
      enemy_direction_goal_.command = 1;
      enemy_direction_actionlib_client_.sendGoal(enemy_direction_goal_,
                                                 actionlib::SimpleActionClient<messages::EnemyDirectionAction>::SimpleDoneCallback(),
                                                 actionlib::SimpleActionClient<messages::EnemyDirectionAction>::SimpleActiveCallback(),
                                                 boost::bind(&Blackboard::ColorDetectionFeedbackCallback, this, _1));


      armor_detection_actionlib_client_.waitForServer();
      LOG_INFO << "Armor detection module has been connected!";
      armor_detection_goal_.command = 1;
      armor_detection_actionlib_client_.sendGoal(armor_detection_goal_,
                                                 actionlib::SimpleActionClient<messages::ArmorDetectionAction>::SimpleDoneCallback(),
                                                 actionlib::SimpleActionClient<messages::ArmorDetectionAction>::SimpleActiveCallback(),
                                                 boost::bind(&Blackboard::ArmorDetectionFeedbackCallback, this, _1));
      
      // obstacle_scan_actionlib_client_.waitForServer();
      // LOG_INFO << "Obstacle scan module has been connected!";
      // obstacle_scan_goal_.checkpoint = 1;
      // obstacle_scan_actionlib_client_.sendGoal(obstacle_scan_goal_,
      //                                           actionlib::SimpleActionClient<rmsc_messages::ObstacleScanAction>::SimpleDoneCallback(),
      //                                           actionlib::SimpleActionClient<rmsc_messages::ObstacleScanAction>::SimpleActiveCallback(),
      //                                           boost::bind(&Blackboard::ObstacleScanCallback, this, _1));
      map_mux_actionlib_client_.waitForServer();
      
      // seer_goal_.cmd = 1;
      // seer_actionlib_client_.sendGoal(seer_goal_,
      //                                 actionlib::SimpleActionClient<rmsc_messages::SeerAction>::SimpleDoneCallback(),
      //                                 actionlib::SimpleActionClient<rmsc_messages::SeerAction>::SimpleActiveCallback(),
      //                                 actionlib::SimpleActionClient<rmsc_messages::SeerAction>::SimpleFeedbackCallback());
    }

    if (decision_config.master()) {
      goal_task_sub_ = goal_task_nh.subscribe("/wing/goal_task", 2, &Blackboard::GoalTaskCallBack, this);
      LOG_INFO << "create master receive goal subscribe";
    } else {
      goal_task_sub_ = goal_task_nh.subscribe("/master/goal_task", 2, &Blackboard::GoalTaskCallBack, this);
      LOG_INFO << "create wing receive goal subscribe";
    }

    // Seer Client
    if (!decision_config.master()) { 
      seer_actionlib_client_.waitForServer();
    }
    // Seer Subscribers
    ammo_scan_sub_ = nh.subscribe("ammo_scan", 30, &Blackboard::AmmoScanCallback, this);
    obstacle_scan_sub_ = nh.subscribe("/obstacle_scan", 30, &Blackboard::ObstacleScanCallback, this);

    collected_ammo_pub_ = nh.advertise<std_msgs::Int32>("/collected_ammo",100);
    collected_ammo_sub_ = nh.subscribe("/collected_ammo",30,&Blackboard::CollectedAmmoCallback, this);
  }

  ~Blackboard() = default;

  void StartSelfCheck() {
    messages::SelfCheck self_check_msg;
    self_check_msg.request.self_check = true;
    if(self_check_client_.call(self_check_msg)){
      LOG_INFO << "Started to run SelfCheck!";
    } else{
      LOG_ERROR << "Failed to run SelfCheck!";
    }
      LOG_INFO << "SelfCheck "<<self_check_msg.response.passed ? "Passed":"Not Passed";
  }

  // Game Info
  void GameInfoCallback(const messages::GameInfo::ConstPtr & game_info){
    game_process_ = static_cast<GameProcess>(game_info->game_process);
    remain_hp_ = static_cast<unsigned int>(game_info->remain_hp);
  }

  // Robot Hurt
  void RobotHurtCallback(const messages::RobotHurtData::ConstPtr & robot_hurt_data){
    last_armor_attacked__time_ = ros::Time::now();
    switch(robot_hurt_data->armor_type){
      case 0:
        armor_attacked_ = ArmorAttacked::FRONT;
        break;
      case 1:
        armor_attacked_ = ArmorAttacked::LEFT;
        break;
      case 2:
        armor_attacked_ = ArmorAttacked::BACK;
        break;
      case 3:
        armor_attacked_ = ArmorAttacked::RIGHT;
        break;
      default:
        armor_attacked_ = ArmorAttacked::NONE;
        return;
    }
  }

  //RFID
  void RfidInfoCallback(const messages::RfidInfo::ConstPtr & rfid_info){
    last_rfid_active_time_ = ros::Time::now();
    //if (rfid_info->card_type == 11){
    rfid_active_ = true;
    LOG_INFO<<"rfid receive";
    //} else{
    //  rfid_active_ = false;
    //}
  }

  //Game Buff
  bool GameBuffStatusCallback(messages::GameBuffStatus::Request & req,
                              messages::GameBuffStatus::Response & res){

    switch(req.buff_info){
      case 0:
        buff_status_ = BuffStatus::NONE;
        break;
      case 1:
        buff_status_ = BuffStatus::SELF;
        break;
      case 2:
        buff_status_ = BuffStatus::ENEMY;
        break;
      default:
        LOG_ERROR<<"The game buff status received is not valid!";
        res.received = false;
        return false;
    }
    res.received = true;
    return true;
  }

  //goal task
  void GoalTaskCallBack(const messages::GoalTask::ConstPtr &req) {
    auxiliary_position_ = req->goal;
    last_auxiliary_time_ = ros::Time::now();
    LOG_INFO << "receive goal task";
    auxiliary_ = true;
    arrive_ = req->arrive;
  }

  //Shoot Info
  void ShootInfoCallback(const messages::ShootInfo::ConstPtr & shoot_info){
    remain_bullet_ = shoot_info->remain_bullet;
    sent_bullet_ = shoot_info->sent_bullet;
  }
  bool GetSentBulletStatus() {
    if (sent_bullet_ >= 180) {
      return true;
    } else {
      return false;
    }
  }
  
  bool GetRemainBulletStatus() {
    if (remain_bullet_ < 3) {
      return true;
    } else {
      return false;
    }
  }
  
  // Enemy
  void ArmorDetectionFeedbackCallback(const messages::ArmorDetectionFeedbackConstPtr& feedback){
    if (feedback->detected){
      enemy_detected_ = true;
      LOG_INFO<<"Find Enemy!";

    tf::Stamped<tf::Pose> tf_pose, global_tf_pose;
    geometry_msgs::PoseStamped camera_pose_msg, global_pose_msg;
    camera_pose_msg = feedback->enemy_pos;

    double distance = std::sqrt(camera_pose_msg.pose.position.x*camera_pose_msg.pose.position.x+camera_pose_msg.pose.position.y*camera_pose_msg.pose.position.y);
    double yaw = atan(camera_pose_msg.pose.position.y/camera_pose_msg.pose.position.x);

    //camera_pose_msg.pose.position.z=camera_pose_msg.pose.position.z;
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,
                                                            0,
                                                            yaw);
    camera_pose_msg.pose.orientation.w = quaternion.w();
    camera_pose_msg.pose.orientation.x = quaternion.x();
    camera_pose_msg.pose.orientation.y = quaternion.y();
    camera_pose_msg.pose.orientation.z = quaternion.z();
    poseStampedMsgToTF(camera_pose_msg, tf_pose);

    tf_pose.stamp_ = ros::Time(0);
    try
    {
      tf_ptr_->transformPose("map", tf_pose, global_tf_pose);
      tf::poseStampedTFToMsg(global_tf_pose, global_pose_msg);
      track_pub_.publish(global_pose_msg);

      if(GetDistance(global_pose_msg, enemy_pose_)>0.2 || GetAngle(global_pose_msg, enemy_pose_) > 0.2){
        enemy_pose_ = global_pose_msg;

      }
    }
    catch (tf::TransformException &ex) {
      LOG_INFO<<"tf error";
    }
    } else{
      enemy_detected_ = false;
    }



  }

  void ColorDetectionFeedbackCallback(const messages::EnemyDirectionFeedbackConstPtr& feedback) {
    LOG_INFO<<"Find Enemy back!";
    last_color_detected_time_ = ros::Time::now();
    switch (feedback->direction) {
      case 0:
        color_detected_ = ColorDetected ::NONE;
        break;
      case 1:
        color_detected_ = ColorDetected ::BACK;
        break;
      case 2:
        color_detected_ = ColorDetected ::LEFT;
        break;
      case 3:
        color_detected_ = ColorDetected ::RIGHT;
        break;
      case 4:
        color_detected_ = ColorDetected ::FRONT;
        break;
      default:
        color_detected_ = ColorDetected ::NONE;
        break;
    }
  }

  void EnemyCallback(const geometry_msgs::PoseStamped::ConstPtr& enemy_pose){
    enemy_detected_ = !enemy_detected_;
    enemy_pose_ = *enemy_pose;
  }

  GameProcess GetGameProcess() const{
    LOG_INFO<<__FUNCTION__<<": "<<(int)game_process_;
    return game_process_;
  }

  unsigned int GetHP() const{
    return remain_hp_;
  }

  bool GetRfidActive() const{
    if(ros::Time::now()-last_rfid_active_time_>ros::Duration(0.5)){
      LOG_INFO<<"rfid timeout";
      return false;
    } else{
      LOG_INFO<<__FUNCTION__<<": "<<(int)rfid_active_;
      return rfid_active_;
    }
  }

  geometry_msgs::PoseStamped GetAuxiliaryPosition() const {
    return auxiliary_position_;
  }

  bool GetAuxiliaryState() {
    if(ros::Time::now()-last_auxiliary_time_>ros::Duration(0.1)) {
      LOG_INFO << "this goal task is out of time";
      auxiliary_ = false;
      return false;
    } else {
      return auxiliary_;
    }

  }

  bool GetArrive() {
    return arrive_;
  }

  BuffStatus GetBuffStatus() const {
    LOG_INFO<<__FUNCTION__<<": "<<(int)buff_status_;
    return buff_status_;
  }

  ArmorAttacked GetArmorAttacked() const{
    if(ros::Time::now()-last_armor_attacked__time_>ros::Duration(0.1)){
      LOG_INFO<<__FUNCTION__<<": NONE";
      return ArmorAttacked::NONE;
    } else{
      LOG_INFO<<__FUNCTION__<<": "<<(int)armor_attacked_;
      return armor_attacked_;
    }
  }
  ColorDetected GetColordetected() const{
    if(ros::Time::now()-last_color_detected_time_>ros::Duration(0.2)){
      LOG_INFO<<__FUNCTION__<<": NONE";
      return ColorDetected::NONE;
    } else {
      LOG_INFO<<__FUNCTION__<<": "<<(int)color_detected_;
      return color_detected_;
    }
  }

  double HurtedPerSecond() {
    if (ros::Time::now()-last_get_hp_time_ > ros::Duration(0.5)) {
      auto reduce_hp = last_hp_ - remain_hp_;
      dmp_ = reduce_hp / (ros::Time::now()-last_get_hp_time_).toSec();
      last_hp_ = remain_hp_;
      last_get_hp_time_ = ros::Time::now();
      return dmp_;
    } else {
      return dmp_;
    }
  }

  geometry_msgs::PoseStamped GetEnemy() const {
    return enemy_pose_;
  }

  bool GetEnemyDetected() const{
    LOG_INFO<<__FUNCTION__<<": "<<enemy_detected_;
    return enemy_detected_;
  }

  ChassisMode GetChassisMode() const {
    return chassis_mode_;
  }
  bool SetChassisMode(const ChassisMode &chassis_mode) {
    if(chassis_mode == chassis_mode_){
      return true;
    }
    messages::ChassisMode chassis_mode_msg;
    chassis_mode_msg.request.chassis_mode = static_cast<uint8_t >(chassis_mode);
    if(chassis_mode_client_.call(chassis_mode_msg) && chassis_mode_msg.response.received){
      chassis_mode_ = chassis_mode;
      LOG_INFO<<"Set gimbal mode to"<< static_cast<int>(chassis_mode);
      return true;
    } else{
      LOG_ERROR<<"Set chassis mode failed!";
      return false;
    }
  }

  GimbalMode GetGimbalMode() const {
    return gimbal_mode_;
  }
  bool SetGimbalMode(const GimbalMode &gimbal_mode) {
    if(gimbal_mode == gimbal_mode_){
      return true;
    }
    messages::GimbalMode gimbal_mode_msg;
    gimbal_mode_msg.request.gimbal_mode = static_cast<uint8_t >(gimbal_mode);
    if(gimbal_mode_client_.call(gimbal_mode_msg) && gimbal_mode_msg.response.received){
      gimbal_mode_ = gimbal_mode;
      LOG_INFO<<"Set gimbal mode to"<< static_cast<int>(gimbal_mode);
      return true;
    } else{
      LOG_ERROR<<"Set gimbal mode failed!";
      return false;
    }
  }

  bool GetShootControl() const {
    return shoot_control_;
  }
  bool SetShootControl(const bool &shoot_control) {
    if(shoot_control == shoot_control_){
      return true;
    }
    messages::ShootModeControl shoot_control_msg;
    shoot_control_msg.request.c_shoot_cmd = shoot_control;
    shoot_control_msg.request.fric_wheel_run = true;
    if(shoot_control_client_.call(shoot_control_msg) && shoot_control_msg.response.received){
      shoot_control_ = shoot_control;
      return true;
    } else{
      LOG_ERROR<<"Set shoot control failed!";
      return false;
    }
  }
  double GetDistance(const geometry_msgs::PoseStamped &pose1,
                     const geometry_msgs::PoseStamped &pose2) {
    const geometry_msgs::Point point1 = pose1.pose.position;
    const geometry_msgs::Point point2 = pose2.pose.position;
    const double dx = point1.x - point2.x;
    const double dy = point1.y - point2.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double GetAngle(const geometry_msgs::PoseStamped &pose1,
                  const geometry_msgs::PoseStamped &pose2) {
    const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
    const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
    tf::Quaternion rot1, rot2;
    tf::quaternionMsgToTF(quaternion1, rot1);
    tf::quaternionMsgToTF(quaternion2, rot2);
    return rot1.angleShortestPath(rot2);
  }
  void ResetAllStatus () {
    remain_hp_ = 2000;
    last_hp_ = 2000;
    armor_attacked_ = ArmorAttacked::NONE;
    color_detected_ = ColorDetected::NONE;
    rfid_active_ = false;
    //auxiliary_ = false;
    buff_status_ = BuffStatus::NONE;
    remain_bullet_ = 200;
    last_get_hp_time_ = ros::Time::now();
    dmp_ = 0;
    remain_bullet_ = 200;
    sent_bullet_ = 0;
    //arrive_ = false;
  }
  void SetArrive(bool arrive) {
    arrive_ = arrive;
  }


  /// Collect Ammo Functions ----------------------------------------------------------------------------------------------
  int GetAmmoIndex(const int priority_thres) {
    int min_cnt = priority_thres + 1;
    int min_index = -1;
    for (int i = 0; i < 30; i++){
      if (ammobox_list_[i] < min_cnt && ammobox_list_[i] > 0){
        min_cnt = ammobox_list_[i];
        min_index = i + 1;
      }
    }
    return min_index;
  }
  
  void SetAmmoCollected(unsigned int index) {
    LOG_WARNING << "Ammobox " << index << ": SUCCESS";
    ammobox_collected_cnt++;
    ammobox_list_[index-1] = -1;
    std_msgs::Int32 msg;
    msg.data = index;
    collected_ammo_pub_.publish(msg);
  }

  void SetAmmoNotCollected(unsigned int index) {
    LOG_WARNING << "Ammobox :" << index << ": FAILED";
    ammobox_list_[index-1] = ammobox_list_[index-1] + 1;
  }

  unsigned int GetAmmoCount() {
    return ammobox_collected_cnt;
  }

  void CollectedAmmoCallback(const std_msgs::Int32ConstPtr &msg) {
    LOG_WARNING << "Ammobox " << msg->data << "is collected by someelse";
    ammobox_list_[msg->data-1] = -1;
  }

  void LoadInitialAmmoList(const rrts::decision::DecisionConfig &decision_config) {
    for (int i = 0; i < 30; i++) {
      ammobox_list_[i]= decision_config.initial_ammo_list().state(i);
    }
  }

  void AmmoScanCallback(const std_msgs::Int32MultiArrayConstPtr &msg) {
    GameProcess game_process = GetGameProcess();
    if (ammo_scan_get_ || game_process != rrts::decision::GameProcess::COUNTDOWN) {
      return;
    }
    else {
      LOG_WARNING << "RCV Ammo Scan Data...";
      int cnt = 1;
      for (std::vector<int32_t>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it) {
        ammobox_list_[*it - 1] = cnt;
        cnt += 1; // decrease priority
      }
      ammo_scan_get_ = true;
      DisplayAmmoList();
    }
  }

  void DisplayAmmoList() {
    LOG_WARNING << "Ammo List:" << std::endl << ammobox_list_[0] << ammobox_list_[1] << ammobox_list_[2] << ammobox_list_[3]
            << ammobox_list_[4] << ammobox_list_[5] << " | " << ammobox_list_[6] << ammobox_list_[7]
            << ammobox_list_[8] << "," << ammobox_list_[9] << ammobox_list_[10] << ammobox_list_[11] << "," 
            << ammobox_list_[12] << ammobox_list_[13] << ammobox_list_[14] << std::endl
            << ammobox_list_[15] << ammobox_list_[16] << ammobox_list_[17] << ammobox_list_[18]
            << ammobox_list_[19] << ammobox_list_[20] << " | " << ammobox_list_[21] << ammobox_list_[22] 
            << ammobox_list_[23] << "," << ammobox_list_[24] << ammobox_list_[25] << ammobox_list_[26] << ","
            << ammobox_list_[27] << ammobox_list_[28] << ammobox_list_[29];
  }

  /// Random Obstacle Functions ----------------------------------------------------------------------------------------------

  void ObstacleScanCallback(const std_msgs::Int32ConstPtr &msg)
  {
    GameProcess game_process = GetGameProcess();
    if (obstacle_scan_get_ || game_process != rrts::decision::GameProcess::COUNTDOWN) {
      return;
    }
    else {
      LOG_WARNING << "RCV Obstacle Scan Data...";
      map_index_ = msg->data;
      LOG_WARNING << "Map Index:" << map_index_;
      rmsc_messages::MapMuxGoal goal;
      goal.map_index = map_index_;
      map_mux_actionlib_client_.sendGoal(goal);
      obstacle_scan_get_ = true;
    }
  }

  const int GetMapIndex() {
    return map_index_;
  }

  // void DisplayObstacleList() {
  //   LOG_WARNING << "Obstacle List:" << random_obstacle_list_[0] << random_obstacle_list_[1] << random_obstacle_list_[2]
  //                                   << random_obstacle_list_[3] << random_obstacle_list_[4] << random_obstacle_list_[5];
  // }

  /// Utility Functions ----------------------------------------------------------------------------------------------
  bool GetAGBIssued() {
    return AGB_issued;
  }

  void SetAGBIssued() {
    AGB_issued = true;
  }

  void PlaySound(const std::string &filename){
    std::string path = ros::package::getPath("roborts");
    LOG_INFO << "PLAY:" << path+filename;
    sound_client_.playWave(path+filename);
  }

  /// ----------------------------------------------------------------------------------------------

 private:
  //! tf
  std::shared_ptr<tf::TransformListener> tf_ptr_;

  //! Enenmy detection
  ros::Subscriber enemy_sub_;

  //! Referee system subscriber
  ros::Subscriber game_info_sub_;

  ros::Subscriber robot_hurt_data_sub_;

  ros::Subscriber rfid_info_sub_;

  ros::Subscriber shoot_info_sub_;

  ros::Publisher track_pub_;

  ros::ServiceServer game_buff_status_server_;

  ros::Subscriber goal_task_sub_;

  ros::ServiceClient self_check_client_;

  ros::ServiceClient chassis_mode_client_;

  ros::ServiceClient gimbal_mode_client_;

  ros::ServiceClient shoot_control_client_;

  // Subscriber
  ros::Subscriber ammo_scan_sub_;
  ros::Subscriber obstacle_scan_sub_;
  ros::Publisher collected_ammo_pub_;
  ros::Subscriber collected_ammo_sub_;

  //! Action client
  actionlib::SimpleActionClient<messages::LocalizationAction> localization_actionlib_client_;
  actionlib::SimpleActionClient<messages::ArmorDetectionAction> armor_detection_actionlib_client_;
  actionlib::SimpleActionClient<messages::EnemyDirectionAction> enemy_direction_actionlib_client_;
  // actionlib::SimpleActionClient<rmsc_messages::ObstacleScanAction> obstacle_scan_actionlib_client_;
  actionlib::SimpleActionClient<rmsc_messages::MapMuxAction> map_mux_actionlib_client_;
  actionlib::SimpleActionClient<rmsc_messages::SeerAction> seer_actionlib_client_;


  //! Action goal
  messages::LocalizationGoal localization_goal_;
  messages::ArmorDetectionGoal armor_detection_goal_;
  messages::EnemyDirectionGoal enemy_direction_goal_;
  rmsc_messages::SeerGoal seer_goal_;
  // rmsc_messages::ObstacleScanGoal obstacle_scan_goal_;

  //! Referee system info
  GameProcess game_process_;
  unsigned int remain_hp_;
  unsigned int last_hp_;
  ros::Time last_get_hp_time_;
  double dmp_;

  ArmorAttacked armor_attacked_;
  ros::Time last_armor_attacked__time_;

  ColorDetected color_detected_;
  ros::Time last_color_detected_time_;

  bool rfid_active_;
  ros::Time last_rfid_active_time_;

  // auxiliary info
  geometry_msgs::PoseStamped auxiliary_position_;
  bool auxiliary_;
  bool arrive_;
  ros::Time last_auxiliary_time_;

  BuffStatus buff_status_;

  unsigned int remain_bullet_;
  unsigned int sent_bullet_;

  GimbalMode gimbal_mode_;
  ChassisMode chassis_mode_;

  bool shoot_control_;
  //! Enemy info
  geometry_msgs::PoseStamped enemy_pose_;
  bool enemy_detected_;

  // Debug Info
  sound_play::SoundClient sound_client_;

  // Collect ammo variables ----------------------------------
  unsigned int ammobox_collected_cnt;
  bool ammo_scan_get_ = false;
  int ammobox_list_[30] = 
  {
    // Domestic Ammo
    -1,0,0,0,0,
    -1,0,-1,0,0,
    0,0,0,0,0,
    // Enemy Ammo
    -1,0,0,0,0,
    -1,0,-1,0,0,
    0,0,0,0,0
  };

  bool obstacle_scan_get_ = false;
  int map_index_ = 0;
  
  bool AGB_issued = false;

};
} //namespace decision
} //namespace rrts
#endif //MODULE_DECISION_BEHAVIOR_TREE_BLACKBOARD_H
