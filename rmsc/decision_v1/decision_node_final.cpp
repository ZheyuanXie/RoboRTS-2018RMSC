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

#include "rmsc/decision_v1/behavior_tree.h"
#include "rmsc/decision_v1/action_behavior.h"

#include "common/log.h"

int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::WARNING;
  FLAGS_colorlogtostderr = true;
  FLAGS_v = 3;
  google::InstallFailureSignalHandler();

  LOG_INFO << "Decision Node Starting...";
  ros::init(argc, argv, "decision_node");

  rrts::decision::DecisionConfig robot_config;
  rrts::common::ReadProtoFromTextFile("/rmsc/decision_v1/config/decision.prototxt", &robot_config);
  LOG_WARNING << "Use Referee:" << robot_config.use_referee();
  LOG_WARNING << "Minimum Ammo:" << robot_config.minimum_ammo();
  LOG_WARNING << "Initializing Blackboard...";
  auto blackboard_ptr_ = std::make_shared<rrts::decision::Blackboard>("/rmsc/decision_v1/config/decision.prototxt");
  LOG_WARNING << "Initializing Goal Factory...";
  auto goal_factory_ = std::make_shared<rrts::decision::GoalFactory>(blackboard_ptr_,"/rmsc/decision_v1/config/decision.prototxt");
  LOG_WARNING << "Building BT...";

  //leaves
  auto wait_action_ = std::make_shared<rrts::decision::WaitAction>(blackboard_ptr_, goal_factory_);
  auto whirl_action_ = std::make_shared<rrts::decision::WhirlAction>(blackboard_ptr_, goal_factory_);
  auto gain_buff_action_ = std::make_shared<rrts::decision::GainBuffAction>(blackboard_ptr_, goal_factory_);
  auto get_ammo_action_ = std::make_shared<rrts::decision::GetAmmoAction>(blackboard_ptr_, goal_factory_);
  auto turn_to_hurt_action_ = std::make_shared<rrts::decision::TurnToWoundedArmorAction>(blackboard_ptr_,
                                                                                         goal_factory_);
  auto color_detected_action_ = std::make_shared<rrts::decision::TurnToDetectedDirection>(blackboard_ptr_, goal_factory_);
  auto chase_action_ = std::make_shared<rrts::decision::ChaseAction>(blackboard_ptr_, goal_factory_);
  auto shoot_action_ = std::make_shared<rrts::decision::ShootAction>(blackboard_ptr_, goal_factory_);
  auto patrol_action_ = std::make_shared<rrts::decision::PatrolAction>(blackboard_ptr_, goal_factory_);
  auto search_action_ = std::make_shared<rrts::decision::SearchAction>(blackboard_ptr_, goal_factory_);
  auto agb_action_ = std::make_shared<rrts::decision::AGBAction>(blackboard_ptr_, goal_factory_);

  //tree
  auto agb_not_issued_condition_ = std::make_shared<rrts::decision::PreconditionNode>("rfid agb_not_issued_condition", blackboard_ptr_,
                                                                            agb_action_,
                                                                            [&]() {
                                                                              if (blackboard_ptr_->GetAGBIssued())
                                                                              {
                                                                                return false;
                                                                              }
                                                                              else
                                                                              {
                                                                                return true;
                                                                              }
                                                                            },
                                                                            rrts::decision::AbortType::LOW_PRIORITY);
                                                                            
  auto plan_buff_selector_ = std::make_shared<rrts::decision::SelectorNode>("plan buff selector", blackboard_ptr_);
  plan_buff_selector_->AddChildren(agb_not_issued_condition_);
  plan_buff_selector_->AddChildren(gain_buff_action_);

  auto rfid_condition_ = std::make_shared<rrts::decision::PreconditionNode>("rfid condition", blackboard_ptr_,
                                                                            plan_buff_selector_,
                                                                            [&]() {
                                                                              if (blackboard_ptr_->GetRfidActive())
                                                                              {
                                                                                return false;
                                                                              }
                                                                              else
                                                                              {
                                                                                return true;
                                                                              }
                                                                            },
                                                                            rrts::decision::AbortType::BOTH);

  auto search_buff_selector_ = std::make_shared<rrts::decision::SelectorNode>("gain buff selector", blackboard_ptr_);
  search_buff_selector_->AddChildren(rfid_condition_);

  auto enemy_obtain_buff_condition_ = std::make_shared<rrts::decision::PreconditionNode>("enemy obtain buff", blackboard_ptr_,
                                                                                         whirl_action_,
                                                                                         [&]() {
                                                                                           if (blackboard_ptr_->GetBuffStatus() == rrts::decision::BuffStatus::ENEMY)
                                                                                           {
                                                                                             return true;
                                                                                           }
                                                                                           else
                                                                                           {
                                                                                             return false;
                                                                                           }
                                                                                         },
                                                                                         rrts::decision::AbortType::BOTH);

  auto without_buff_selector_ = std::make_shared<rrts::decision::SelectorNode>("without_buff_selector", blackboard_ptr_);
  without_buff_selector_->AddChildren(enemy_obtain_buff_condition_);
  without_buff_selector_->AddChildren(search_buff_selector_);
  
  auto obtain_buff_condition_ = std::make_shared<rrts::decision::PreconditionNode>("obtain buff condition", blackboard_ptr_,
                                                                                   whirl_action_,
                                                                                   [&]() {
                                                                                     if (blackboard_ptr_->GetBuffStatus() == rrts::decision::BuffStatus::SELF)
                                                                                     {
                                                                                       return true;
                                                                                     }
                                                                                     else
                                                                                     {
                                                                                       return false;
                                                                                     }
                                                                                   },
                                                                                   rrts::decision::AbortType::BOTH);

  auto game_start_selector_ = std::make_shared<rrts::decision::SelectorNode>("game_start_selector", blackboard_ptr_);
  game_start_selector_->AddChildren(obtain_buff_condition_);
  game_start_selector_->AddChildren(without_buff_selector_);

  auto game_stop_condition_ = std::make_shared<rrts::decision::PreconditionNode>("game_stop_condition", blackboard_ptr_,
                                                                                 wait_action_,
                                                                                 [&]() {
                                                                                    if (blackboard_ptr_->GetGameProcess() != rrts::decision::GameProcess::FIGHT)
                                                                                      return true;
                                                                                    else
                                                                                      return false;
                                                                                 },
                                                                                 rrts::decision::AbortType::BOTH);

  auto game_status_selector_ = std::make_shared<rrts::decision::SelectorNode>("game_status_selector", blackboard_ptr_);
  game_status_selector_->AddChildren(game_stop_condition_);
  game_status_selector_->AddChildren(game_start_selector_);

  auto wing_bot_condition = std::make_shared<rrts::decision::PreconditionNode>("wing bot condition", blackboard_ptr_,
                                                                               wait_action_,
                                                                               [&]() {
                                                                                 if (robot_config.master())
                                                                                 {
                                                                                   return false;
                                                                                 }
                                                                                 else
                                                                                 {
                                                                                   return true;
                                                                                 }
                                                                               },
                                                                               rrts::decision::AbortType::NONE);

  auto bot_selector_ = std::make_shared<rrts::decision::SelectorNode>("bot selector", blackboard_ptr_);
  bot_selector_->AddChildren(wing_bot_condition);
  bot_selector_->AddChildren(game_status_selector_);

  //root
  rrts::decision::BehaviorTree root(bot_selector_, 25);
  root.Execute();
}
