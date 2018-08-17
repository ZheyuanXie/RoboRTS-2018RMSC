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

#ifndef MODULE_DECISION_ICRA_ACTION_BEHAVIOR_H
#define MODULE_DECISION_ICRA_ACTION_BEHAVIOR_H

#include <unistd.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include "messages/GlobalPlannerAction.h"
#include "messages/LocalPlannerAction.h"

#include "common/error_code.h"
#include "common/log.h"

#include "rmsc/decision_v1_final/behavior_tree.h"
#include "rmsc/decision_v1_final/blackboard.h"

#include "rmsc/decision_v1_final/goal_factory.h"

namespace rrts {
namespace decision {
class EscapeAction : public ActionNode {
 public:
  EscapeAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("escape_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~EscapeAction() = default;

 private:
  virtual void OnInitialize() {
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {

    goal_factory_ptr_->EscapeGoal();

    return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        goal_factory_ptr_->CancelWhirl();
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
        break;
      case BehaviorState::SUCCESS:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" SUCCESS!";
        break;
      case BehaviorState::FAILURE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" FAILURE!";
        break;
      default:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" ERROR!";
        return;
    }
  }

  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // class EscapeAction

class WhirlAction : public ActionNode {
 public:
  WhirlAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("whirl_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~WhirlAction() = default;

 private:
  virtual void OnInitialize() {
    if (goal_factory_ptr_->GetActionState() != BehaviorState::RUNNING) 
      blackboard_ptr_->PlaySound("/sound/no_enemy.wav");
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {

    return goal_factory_ptr_->Whirl();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelWhirl();
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
        break;
      case BehaviorState::SUCCESS:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" SUCCESS!";
        break;
      case BehaviorState::FAILURE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" FAILURE!";
        break;
      default:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" ERROR!";
        return;
    }
  }

  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // class WhirlAction

class ChaseAction : public ActionNode {
 public:
  ChaseAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("chase_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~ChaseAction() = default;

 private:
  virtual void OnInitialize() {
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {


    goal_factory_ptr_->ChaseGoal();
    LOG_INFO << "send chase goal";

    return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        goal_factory_ptr_->CancelWhirl();
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
        break;
      case BehaviorState::SUCCESS:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" SUCCESS!";
        break;
      case BehaviorState::FAILURE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" FAILURE!";
        break;
      default:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" ERROR!";
        return;
    }
  }

  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // class ChaseAction

class PatrolAction : public ActionNode {
 public:
  PatrolAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("patrol_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~PatrolAction() = default;

 private:
  virtual void OnInitialize() {
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {

    goal_factory_ptr_->PatrolGoal();

    goal_factory_ptr_->UpdateActionState();

    return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
        break;
      case BehaviorState::SUCCESS:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" SUCCESS!";
        break;
      case BehaviorState::FAILURE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" FAILURE!";
        break;
      default:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" ERROR!";
        return;
    }
  }

  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // class PatrolAction

class SearchAction : public ActionNode {
 public:
  SearchAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
     failure_count_(0) , ActionNode::ActionNode("search_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~SearchAction() = default;

 private:
  virtual void OnInitialize() {
    failure_count_ = 0;
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {

    if (!(goal_factory_ptr_->SearchValid())) {
      return BehaviorState::FAILURE;
    }

    goal_factory_ptr_->SearchGoal();

    goal_factory_ptr_->UpdateActionState();

    if (goal_factory_ptr_->GetActionState()==BehaviorState::FAILURE) {
      failure_count_++;
      if (failure_count_ == 3) {
        return BehaviorState::FAILURE;
      }
      return BehaviorState::RUNNING;
    } else {
      failure_count_ = 0;
    }

   return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        goal_factory_ptr_->CancelSearch();
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
        break;
      case BehaviorState::SUCCESS:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" SUCCESS!";
        break;
      case BehaviorState::FAILURE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" FAILURE!";
        break;
      default:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" ERROR!";
        return;
    }
  }

  GoalFactory::GoalFactoryPtr goal_factory_ptr_;
  unsigned int failure_count_;

}; // class SearchAction

class GainBuffAction : public ActionNode {
 public:
  GainBuffAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("gain_buff_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~GainBuffAction() = default;

 private:
  virtual void OnInitialize() {
    LOG_INFO<<name_<<" "<<__FUNCTION__;
    if (goal_factory_ptr_->GetActionState() != BehaviorState::RUNNING) 
      blackboard_ptr_->PlaySound("/sound/go_to_buff.wav");
  };

  virtual BehaviorState Update() {

    goal_factory_ptr_->BuffGoal();

    goal_factory_ptr_->UpdateActionState();

    return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
        break;
      case BehaviorState::SUCCESS:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" SUCCESS!";
        break;
      case BehaviorState::FAILURE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" FAILURE!";
        break;
      default:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" ERROR!";
        return;
    }
  }

  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // class GainBuffAction

class ShootAction : public ActionNode {
 public:
  ShootAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("shoot_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~ShootAction() = default;

 private:
  virtual void OnInitialize() {
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {

    blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
    blackboard_ptr_->SetChassisMode(ChassisMode::DODGE_MODE);

    goal_factory_ptr_->SendGoalTask(blackboard_ptr_->GetEnemy());

    return BehaviorState::RUNNING;
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        goal_factory_ptr_->CancelWhirl();
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
        break;
      case BehaviorState::SUCCESS:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" SUCCESS!";
        break;
      case BehaviorState::FAILURE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" FAILURE!";
        break;
      default:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" ERROR!";
        return;
    }
  }

  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // class ShootAction

class TurnToWoundedArmorAction : public ActionNode {
 public:
  TurnToWoundedArmorAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("turn_to_wounded_armor_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~TurnToWoundedArmorAction() = default;

 private:
  virtual void OnInitialize() {
    if (goal_factory_ptr_->GetActionState() != BehaviorState::RUNNING) 
      blackboard_ptr_->PlaySound("/sound/turn_to_hurt.wav");
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {

    goal_factory_ptr_->TurnTOWoundedArmor();

    goal_factory_ptr_->UpdateActionState();

    return goal_factory_ptr_->GetActionState();

  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        goal_factory_ptr_->CancelWhirl();
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
        break;
      case BehaviorState::SUCCESS:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" SUCCESS!";
        break;
      case BehaviorState::FAILURE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" FAILURE!";
        break;
      default:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" ERROR!";
        return;
    }
  }

  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // class TurnTOWoundedArmorAction

class TurnToDetectedDirection : public ActionNode {
 public:
  TurnToDetectedDirection(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("turn_to_detected_direction", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~TurnToDetectedDirection() = default;

 private:
  virtual void OnInitialize() {
    if (goal_factory_ptr_->GetActionState() != BehaviorState::RUNNING) 
      blackboard_ptr_->PlaySound("/sound/turn_to_color.wav");
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {

    goal_factory_ptr_->TurnToDetectedDirection();

    goal_factory_ptr_->UpdateActionState();

    return goal_factory_ptr_->GetActionState();

  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        goal_factory_ptr_->CancelWhirl();
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
        break;
      case BehaviorState::SUCCESS:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" SUCCESS!";
        break;
      case BehaviorState::FAILURE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" FAILURE!";
        break;
      default:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" ERROR!";
        return;
    }
  }

  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // class TurnToDetectedDirection

class WaitAction : public ActionNode {
 public:

  WaitAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("wait_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~WaitAction() = default;
 private:
  virtual void OnInitialize() {
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {

    // goal_factory_ptr_->BackBootArea();

    // goal_factory_ptr_->UpdateActionState();

    // if (goal_factory_ptr_->GetActionState() == BehaviorState::SUCCESS) {
    //   blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
    //   blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELAX);
    // }
    // blackboard_ptr_->ResetAllStatus();

//    if (goal_factory_ptr_->GetState() != BehaviorState::SUCCESS) {
//      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
//      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
//    } else if (goal_factory_ptr_->GetState() == BehaviorState::SUCCESS) {
//    }
    return BehaviorState::RUNNING;
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        // goal_factory_ptr_->CancelGoal();
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
        break;
      case BehaviorState::SUCCESS:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" SUCCESS!";
        break;
      case BehaviorState::FAILURE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" FAILURE!";
        break;
      default:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" ERROR!";
        return;
    }
  }

  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // waitAction

class AuxiliaryAction : public ActionNode {
 public:

  AuxiliaryAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("auxiliary_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~AuxiliaryAction() = default;
 private:
  virtual void OnInitialize() {
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {

    goal_factory_ptr_->GoAuxiliaryPosition();

    goal_factory_ptr_->UpdateActionState();

//    if (goal_factory_ptr_->GetActionState() == BehaviorState::SUCCESS) {
//      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
//      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELAX);
//      blackboard_ptr_->ResetAllStatus();
//    }

//    if (goal_factory_ptr_->GetState() != BehaviorState::SUCCESS) {
//      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
//      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
//    } else if (goal_factory_ptr_->GetState() == BehaviorState::SUCCESS) {
//    }
    if (blackboard_ptr_->GetGameProcess() != GameProcess::FIGHT) {
      return BehaviorState::RUNNING;
    }
    return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
        break;
      case BehaviorState::SUCCESS:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" SUCCESS!";
        break;
      case BehaviorState::FAILURE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" FAILURE!";
        break;
      default:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" ERROR!";
        return;
    }
  }

  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // AuxiliaryAction

class GetAmmoAction : public ActionNode {
 public:

  GetAmmoAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("get_ammo_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~GetAmmoAction() = default;
 private:
  virtual void OnInitialize() {
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {

    if (goal_factory_ptr_->GetGetAmmoActionState()!=BehaviorState::RUNNING){
      goal_factory_ptr_->CancelGoal();
      ammobox_index_ = blackboard_ptr_->GetAmmoIndex();
      if (ammobox_index_ == -1){
        return BehaviorState::SUCCESS;
      }
      switch (blackboard_ptr_->GetAmmoCount()) {
        case 0: blackboard_ptr_->PlaySound("/sound/getammo1.wav"); break;
        case 1: blackboard_ptr_->PlaySound("/sound/getammo2.wav"); break;
        case 2: blackboard_ptr_->PlaySound("/sound/getammo3.wav"); break;
        case 3: blackboard_ptr_->PlaySound("/sound/getammo4.wav"); break;
        case 4: blackboard_ptr_->PlaySound("/sound/getammo5.wav"); break;
        case 5: blackboard_ptr_->PlaySound("/sound/getammolast.wav"); break;
      }
      goal_factory_ptr_->SendAmmoGoal(ammobox_index_);
    }

    goal_factory_ptr_->UpdateGetAmmoActionState();
    BehaviorState state = goal_factory_ptr_->GetGetAmmoActionState();
    return state;
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelAmmoGoal();
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
        break;
      case BehaviorState::SUCCESS:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" SUCCESS!";
        if (ammobox_index_ > 0) blackboard_ptr_->SetAmmoCollected(ammobox_index_);
        break;
      case BehaviorState::FAILURE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" FAILURE!";
        if (ammobox_index_ > 0) blackboard_ptr_->SetAmmoNotCollected(ammobox_index_);
        break;
      default:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" ERROR!";
        return;
    }
  }

  int ammobox_index_ = 0;
  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // GetAmmoAction

class AGBAction : public ActionNode {
 public:

  AGBAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("agb_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~AGBAction() = default;
 private:
  virtual void OnInitialize() {
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {

    if (goal_factory_ptr_->GetAggressiveGainBuffState()!=BehaviorState::RUNNING) {
      goal_factory_ptr_->CancelGoal();
      int route_index;
      int map_index = blackboard_ptr_->GetMapIndex();
      switch(map_index) {
        case 0: route_index = 1; break;
        case 1: route_index = 1; break;
        case 2: route_index = 2; break;
        case 3: route_index = 1; break;
        case 4: route_index = 1; break;
        case 5: route_index = 3; break;
        case 6: route_index = 1; break;
        case 7: route_index = 1; break;
        case 8: route_index = 2; break; 
        case 9: route_index = 1; break;
        case 10: route_index = 1; break;
        case 11: route_index = 2; break;
        case 12: route_index = 1; break;
        default: route_index = 3; break;
      }
      goal_factory_ptr_->SendAggressiveGainBuffGoal(route_index);
      blackboard_ptr_->SetAGBIssued();
    }

    goal_factory_ptr_->UpdateAggressiveGainBuffState();
    BehaviorState state = goal_factory_ptr_->GetAggressiveGainBuffState();
    return state;
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelAggressiveGainBuffGoal();
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
        break;
      case BehaviorState::SUCCESS:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" SUCCESS!";
        break;
      case BehaviorState::FAILURE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" FAILURE!";
        break;
      default:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" ERROR!";
        return;
    }
  }

  int ammobox_index_ = 0;
  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // AGBAction

class BaseWaitAction : public ActionNode {
 public:

  BaseWaitAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("base_wait_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~BaseWaitAction() = default;
 private:
  virtual void OnInitialize() {
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {

    goal_factory_ptr_->BackBootArea();

    goal_factory_ptr_->UpdateActionState();

    // if (goal_factory_ptr_->GetActionState() == BehaviorState::SUCCESS) {
    //   blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
    //   blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELAX);
    // }
    // blackboard_ptr_->ResetAllStatus();

//    if (goal_factory_ptr_->GetState() != BehaviorState::SUCCESS) {
//      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
//      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
//    } else if (goal_factory_ptr_->GetState() == BehaviorState::SUCCESS) {
//    }
    return BehaviorState::RUNNING;
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
        break;
      case BehaviorState::SUCCESS:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" SUCCESS!";
        break;
      case BehaviorState::FAILURE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" FAILURE!";
        break;
      default:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" ERROR!";
        return;
    }
  }

  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // waitAction

class GoEnemyBaseAction : public ActionNode {
 public:

  GoEnemyBaseAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("go enemy base action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~GoEnemyBaseAction() = default;
 private:
  virtual void OnInitialize() {
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {

    goal_factory_ptr_->GoEnemyBase();

    goal_factory_ptr_->UpdateActionState();

    // if (goal_factory_ptr_->GetActionState() == BehaviorState::SUCCESS) {
    //   blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
    //   blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELAX);
    // }
    // blackboard_ptr_->ResetAllStatus();

//    if (goal_factory_ptr_->GetState() != BehaviorState::SUCCESS) {
//      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
//      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
//    } else if (goal_factory_ptr_->GetState() == BehaviorState::SUCCESS) {
//    }
    return BehaviorState::RUNNING;
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
        break;
      case BehaviorState::SUCCESS:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" SUCCESS!";
        break;
      case BehaviorState::FAILURE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" FAILURE!";
        break;
      default:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" ERROR!";
        return;
    }
  }

  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // goEnemyBaseAction
}
}

#endif //MODULE_DECISION_ICRA_ACTION_BEHAVIOR_H
