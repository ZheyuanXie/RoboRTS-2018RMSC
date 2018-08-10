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

#include "modules/decision/RMSC_decision_v0/behavior_tree.h"
#include "modules/decision/RMSC_decision_v0/action_behavior.h"

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
  
  auto blackboard_ptr_ = std::make_shared<rrts::decision::Blackboard>("/modules/decision/RMSC_decision_v0/config/decision.prototxt");
  auto goal_factory_ = std::make_shared<rrts::decision::GoalFactory>(blackboard_ptr_,"/modules/decision/RMSC_decision_v0/config/decision.prototxt");
  rrts::decision::DecisionConfig robot_config;
  rrts::common::ReadProtoFromTextFile("/modules/decision/RMSC_decision_v0/config/decision.prototxt", &robot_config);

  // Debug
  //blackboard_ptr_ -> PlaySound("/sound/createtree.wav");
  //blackboard_ptr_ -> SetAmmoCollected(1);

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

  //tree
  auto detect_enemy_condition_ = std::make_shared<rrts::decision::PreconditionNode>("plan buff detect enemy condition",
                                                                                    blackboard_ptr_,
                                                                                    chase_action_,
                                                                                    [&]() {
                                                                                      if (blackboard_ptr_->GetEnemyDetected())
                                                                                      {
                                                                                        return true;
                                                                                      }
                                                                                      else
                                                                                      {
                                                                                        return false;
                                                                                      }
                                                                                    },
                                                                                    rrts::decision::AbortType::BOTH);

  auto color_detected_condition_ = std::make_shared<rrts::decision::PreconditionNode>("color detected condition",
                                                                                        blackboard_ptr_,
                                                                                        color_detected_action_,
                                                                                        [&]() {
                                                                                          if (blackboard_ptr_->GetColordetected() != rrts::decision::ColorDetected ::NONE && blackboard_ptr_->GetColordetected() != rrts::decision::ColorDetected ::FRONT)
                                                                                          {
                                                                                            return true;
                                                                                          }
                                                                                          else
                                                                                          {
                                                                                            return false;
                                                                                          }
                                                                                        },
                                                                                        rrts::decision::AbortType::LOW_PRIORITY);

  auto under_attack_condition_ = std::make_shared<rrts::decision::PreconditionNode>("under_attack_condition",
                                                                                    blackboard_ptr_,
                                                                                    turn_to_hurt_action_,
                                                                                    [&]() {
                                                                                      if (blackboard_ptr_->GetArmorAttacked() != rrts::decision::ArmorAttacked::NONE && blackboard_ptr_->GetArmorAttacked() != rrts::decision::ArmorAttacked::FRONT)
                                                                                      {
                                                                                        return true;
                                                                                      }
                                                                                      else
                                                                                      {
                                                                                        return false;
                                                                                      }
                                                                                    },
                                                                                    rrts::decision::AbortType::LOW_PRIORITY);

  auto rfid_condition_ = std::make_shared<rrts::decision::PreconditionNode>("rfid_condition", blackboard_ptr_,
                                                                            gain_buff_action_,
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

  auto final_selector_ = std::make_shared<rrts::decision::SelectorNode>("final_selector", blackboard_ptr_);
  final_selector_->AddChildren(detect_enemy_condition_);
  final_selector_->AddChildren(under_attack_condition_);
  final_selector_->AddChildren(color_detected_condition_);
  final_selector_->AddChildren(patrol_action_);

  auto buff_whirl_sequence_ = std::make_shared<rrts::decision::SequenceNode>("buff_whirl_sequence", blackboard_ptr_);
  buff_whirl_sequence_->AddChildren(gain_buff_action_);
  buff_whirl_sequence_->AddChildren(whirl_action_);

  auto gain_buff_selector_ = std::make_shared<rrts::decision::SelectorNode>("gain_buff_selector", blackboard_ptr_);
  //gain_buff_selector_ ->AddChildren(rfid_condition_);
  gain_buff_selector_ ->AddChildren(final_selector_);

  auto engage_condition_ = std::make_shared<rrts::decision::PreconditionNode>("engage_condition", blackboard_ptr_,
                                                                                 final_selector_,
                                                                                 [&]() {
                                                                                   if ((blackboard_ptr_->GetAmmoCount() >= 3) || blackboard_ptr_->NoAmmo())
                                                                                     return true;
                                                                                   else
                                                                                     return false;
                                                                                 },
                                                                                 rrts::decision::AbortType::SELF);

  auto game_start_selector_ = std::make_shared<rrts::decision::SelectorNode>("game_start_selector", blackboard_ptr_);
  game_start_selector_->AddChildren(engage_condition_);
  game_start_selector_->AddChildren(get_ammo_action_);

  auto game_stop_condition_ = std::make_shared<rrts::decision::PreconditionNode>("game_stop_condition", blackboard_ptr_,
                                                                                 wait_action_,
                                                                                 [&]() {
                                                                                   //return false;
                                                                                   if (blackboard_ptr_->GetGameProcess() != rrts::decision::GameProcess::FIGHT)
                                                                                     return true;
                                                                                   else
                                                                                     return false;
                                                                                 },
                                                                                 rrts::decision::AbortType::BOTH);

  auto game_status_selector_ = std::make_shared<rrts::decision::SelectorNode>("game_status_selector", blackboard_ptr_);
  game_status_selector_->AddChildren(game_stop_condition_);
  game_status_selector_->AddChildren(game_start_selector_);

  //root
  rrts::decision::BehaviorTree root(game_status_selector_, 25);
  root.Execute();
}
