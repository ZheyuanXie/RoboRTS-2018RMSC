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


  //tree
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

  auto under_attack_condition_ = std::make_shared<rrts::decision::PreconditionNode>("plan buff under attack condition",
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

  auto final_selector_ = std::make_shared<rrts::decision::SelectorNode>("final_selector", blackboard_ptr_);
  final_selector_->AddChildren(under_attack_condition_);
  final_selector_->AddChildren(color_detected_condition_);
  final_selector_->AddChildren(whirl_action_);

  auto gain_buff_sequence_ = std::make_shared<rrts::decision::SequenceNode>("gain_buff_sequence", blackboard_ptr_);
  gain_buff_sequence_ ->AddChildren(gain_buff_action_);
  gain_buff_sequence_ ->AddChildren(final_selector_);

  auto engage_condition_ = std::make_shared<rrts::decision::PreconditionNode>("engage_condition", blackboard_ptr_,
                                                                                 gain_buff_sequence_,
                                                                                 [&]() {
                                                                                   if (blackboard_ptr_->GetAmmoCount() >= 1)
                                                                                     return true;
                                                                                   else
                                                                                     return false;
                                                                                 },
                                                                                 rrts::decision::AbortType::BOTH);

  auto game_start_selector_ = std::make_shared<rrts::decision::SelectorNode>("game_start_selector", blackboard_ptr_);
  game_start_selector_->AddChildren(engage_condition_);
  game_start_selector_->AddChildren(get_ammo_action_);

  auto game_stop_condition_ = std::make_shared<rrts::decision::PreconditionNode>("game_stop_condition", blackboard_ptr_,
                                                                                 wait_action_,
                                                                                 [&]() {
                                                                                   return false;
                                                                                   if (blackboard_ptr_->GetConditionOverride().game_stop_condition_)
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
