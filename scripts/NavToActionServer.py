#!/usr/bin/env python
import rospy
from threading import Thread

from actionlib import SimpleActionClient, SimpleActionServer
from messages.msg import GlobalPlannerAction, LocalPlannerAction, GlobalPlannerGoal, LocalPlannerGoal
from messages.msg import NavToAction
from geometry_msgs.msg import PoseStamped

class NavToStatus:
    IDLE        = 0
    MOVING      = 0

class NavigationTestNode:
    def __init__(self):
        # action client
        self._ac_gp = SimpleActionClient("global_planner_node_action", GlobalPlannerAction)
        self._ac_gp.wait_for_server()
        self._ac_lp = SimpleActionClient("local_planner_node_action", LocalPlannerAction)
        self._ac_lp.wait_for_server()

        # action servergoal
        self._as = SimpleActionServer("navigation_node_action", NavToAction, self.ExecuteCB, auto_start=False)

        # define goals to send
        self.global_planner_goal_ = GlobalPlannerGoal()
        self.local_planner_goal_ = LocalPlannerGoal()
        self.new_goal_ = False
        self.new_path_ = False
        self.global_planner_done = False
        self.local_planner_done = False

        # new thread running the execution loop
        self.thread_ = Thread(target=self.Loop)
        self.thread_.start()
        self._as.start()

    def ExecuteCB(self, goal):
        print 'NAV GOAL RCV'
        self.global_planner_goal_.goal = goal.navgoal
        self.global_planner_done = False
        self.local_planner_done = False
        self.new_goal_ = True
        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                print 'PREEMPT REQ'
                self._as.set_preempted()
                self.new_goal_ = False
                self.new_path_ = False
                self._ac_gp.cancel_all_goals()
                self._ac_lp.cancel_all_goals()
                return
            if self.global_planner_done:
                self._as.set_succeeded()
                print 'SUCCESS'
                break
    
    def Loop(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.new_goal_:
                self._ac_gp.send_goal(self.global_planner_goal_,self.GlobalPlannerDoneCB,None,self.GlobalPlannerFeedbackCallback)
                self.new_goal_ = False
            if self.new_path_:
                self._ac_lp.send_goal(self.local_planner_goal_,self.LocalPlannerDoneCallback)
                self.new_path_ = False
            try:
                rate.sleep()
            except:
                print "Exiting..."
    
    def GlobalPlannerDoneCB(self,data1,data2):
        #print "GP:",data1,data2,rospy.get_time()
        self.global_planner_done = True
    
    def GlobalPlannerFeedbackCallback(self,data):
        self.local_planner_goal_.route = data.path
        self.new_path_ = True
        print '.',
    
    def LocalPlannerDoneCallback(self,data1,data2):
        #print "LP:",data1,data2,rospy.get_time()
        self.local_planner_done = True

if __name__ == "__main__":
    rospy.init_node("navto_action_server_node")
    nt = NavigationTestNode()
    rospy.spin()