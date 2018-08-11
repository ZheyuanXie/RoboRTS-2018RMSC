#!/usr/bin/env python
import rospy
from threading import Thread

from actionlib import SimpleActionClient, SimpleActionServer
from messages.msg import GlobalPlannerAction, LocalPlannerAction, GlobalPlannerGoal, LocalPlannerGoal
from rmsc_messages.msg import NavToAction
from geometry_msgs.msg import PoseStamped

class NavToStatus:
    IDLE        = 0
    MOVING      = 0

class NavToNode(object):
    def __init__(self):
        # action client
        self._ac_gp = SimpleActionClient("global_planner_node_action", GlobalPlannerAction)
        self._ac_gp.wait_for_server()
        self._ac_lp = SimpleActionClient("local_planner_node_action", LocalPlannerAction)
        self._ac_lp.wait_for_server()

        # action servergoal
        self._as = SimpleActionServer("nav_to_node_action", NavToAction, self.ExecuteCB, auto_start=False)

        # define goals to send
        self.global_planner_goal_ = GlobalPlannerGoal()
        self.local_planner_goal_ = LocalPlannerGoal()
        self.new_goal_ = False
        self.new_path_ = False
        self.global_planner_error_code = -1
        self.local_planner_error_code = -1

        # new thread running the execution loop
        self.thread_ = Thread(target=self.Loop)
        self.thread_.start()
        self._as.start()

    def ExecuteCB(self, goal):
        rospy.loginfo('NAVTO: Goal received')
        self.global_planner_goal_.goal = goal.navgoal
        self.global_planner_error_code = -1
        self.global_planner_error_code = -1
        self.new_goal_ = True
        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                rospy.loginfo('NAVTO: preempt requested')
                self._as.set_preempted()
                self.new_goal_ = False
                self.new_path_ = False
                self._ac_gp.cancel_all_goals()
                self._ac_lp.cancel_all_goals()
                return
            if self.global_planner_error_code == 0:
                self.new_goal_ = False
                self.new_path_ = False
                self._ac_lp.cancel_all_goals()
                self._as.set_succeeded()
                rospy.loginfo('NAVTO: succeeded')
                break
            if self.global_planner_error_code >=1:
                self.new_goal_ = False
                self.new_path_ = False
                self._ac_lp.cancel_all_goals()
                self._as.set_aborted()
                rospy.loginfo('NAVTO: failed')
                break
    
    def Loop(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                self._ac_gp.cancel_all_goals()
                self._ac_lp.cancel_all_goals()
            if self.new_goal_:
                self._ac_gp.send_goal(self.global_planner_goal_,self.GlobalPlannerDoneCB,None,self.GlobalPlannerFeedbackCallback)
                self.new_goal_ = False
            if self.new_path_:
                self._ac_lp.send_goal(self.local_planner_goal_,self.LocalPlannerDoneCallback,None,self.LocalPlanerFeedbackCallback)
                self.new_path_ = False
            try:
                rate.sleep()
            except:
                rospy.loginfo('NAVTO: exiting')
    
    def GlobalPlannerDoneCB(self,state,result):
        rospy.loginfo("GP:%d, %d"%(state,result.error_code))
        self.global_planner_error_code = result.error_code
    
    def GlobalPlannerFeedbackCallback(self,data):
        self.local_planner_goal_.route = data.path
        self.new_path_ = True
    
    def LocalPlannerDoneCallback(self,state,result):
        rospy.loginfo("LP:%d, %d"%(state,result.error_code))
        self.local_planner_error_code = result.error_code
    
    def LocalPlanerFeedbackCallback(self,data):
        pass

if __name__ == "__main__":
    rospy.init_node("nav_to_node")
    nt = NavToNode()
    rospy.spin()