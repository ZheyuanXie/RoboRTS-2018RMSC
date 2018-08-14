#! /usr/bin/env python
import rospy
import tf
import numpy as np
from rmsc_messages.msg import AggressiveGainBuffAction, AggressiveGainBuffInfo
from rmsc_messages.msg import NavToAction, NavToActionGoal, NavToActionResult
from messages.srv import ChassisModeRequest, ChassisMode, ChassisModeResponse
from actionlib import SimpleActionServer
from actionlib_msgs.msg import GoalStatus
from actionlib import SimpleActionClient

class ChassisModeForm:
  CHASSIS_RELAX = 0
  CHASSIS_STOP = 1
  MANUAL_SEPARATE_GIMBAL = 2
  MANUAL_FOLLOW_GIMBAL = 3
  DODGE_MODE = 4
  AUTO_SEPARATE_GIMBAL = 5
  AUTO_FOLLOW_GIMBAL = 6
  AGGRESSIVE_GAIN_BUFF_ONE = 7
  AGGRESSIVE_GAIN_BUFF_TWO = 8

class ChassisArriveState:
    IDEL = 0
    RUNNING = 1
    SUCCEEDED = 2

BuffPath = [
    {'id':1, 'x':1.0, 'y':4.5, 'yaw':1.57},
    {'id':2, 'x':4.1, 'y':2.9, 'yaw':0.00},
]

BUFF_STATE = ['IDEL', 'RUNNING', 'SUCCEEDED']
CHASSIS_RUN_WAIT_TIME = 5
CHASSIS_MODE_WAIT_TIME = 5


class AggressiveGainBuffNode(object):
    def __init__(self, name="aggressive_gain_buff_action"):
        self.action_name = name
        self._as = SimpleActionServer(self.action_name, AggressiveGainBuffAction, execute_cb=self.ExecuteCB, auto_start=False)
        self._as.start()
        
        self.chassis_state_ = 0
        self.sub_odom = rospy.Subscriber("aggressive_buff_info", AggressiveGainBuffInfo, callback=self.OdomCB)
        self.chassis_mode_client = rospy.ServiceProxy("set_chassis_mode", ChassisMode)
        self.chassis_arrive_state = ChassisArriveState()
        
        self.chassis_mode_ = 0
        self._ac_navto = SimpleActionClient("nav_to_node_action", NavToAction)
        rospy.loginfo('GAIN_BUFF: Connecting NavTo action server...')
        ret = self._ac_navto.wait_for_server()
        rospy.loginfo('GAIN_BUFF: NavTo sever connected!') if ret else rospy.logerr('error: NavTo server not started!')
        self.nav_to_error_code = -1
    
    def ExecuteCB(self, goal):      
        print 'GAIN_BUFF:Aggressive gain buff goal recieved!'
        route = goal.route_index
        CHASSIS_MODE = ChassisModeForm()
        self.nav_to_error_code = -1
        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                print 'GAIN_BUFF:PREEMPT REQ'
                self.SetChassisMode(CHASSIS_MODE.AUTO_SEPARATE_GIMBAL)
                self._ac_navto.cancel_all_goals()
                self._as.set_preempted()
                return
            if route == 1 or route == 2:
                print 'GAIN_BUFF:Route %i received' % route
                if route == 1:
                    self.SetChassisMode(CHASSIS_MODE.AGGRESSIVE_GAIN_BUFF_ONE)
                if route == 2:
                    self.SetChassisMode(CHASSIS_MODE.AGGRESSIVE_GAIN_BUFF_TWO)
                print 'GAIN_BUFF:Wait for chassis response...'
                chassis_wait_start_time = rospy.get_time()
                print 'GAIN_BUFF:chassis state is %i' % (self.chassis_state_)
                while self.chassis_state_ == 0 and (rospy.get_time() - chassis_wait_start_time) < CHASSIS_MODE_WAIT_TIME:
                    a=1
                if self.chassis_state_ == 1:
                    print 'GAIN_BUFF:Chassis has responded!'
                else:
                    print 'GAIN_BUFF:Chassis does not respond for the new mode! Return.'
                    self.SetChassisMode(CHASSIS_MODE.AUTO_SEPARATE_GIMBAL)
                    self._as.set_aborted()
                    return
                chassis_run_start_time = rospy.get_time()
                while not (self.chassis_state_ == self.chassis_arrive_state.SUCCEEDED or (rospy.get_time()-chassis_run_start_time > CHASSIS_RUN_WAIT_TIME) ):
                    if self.chassis_state_ == self.chassis_arrive_state.SUCCEEDED:
                        self._as.set_succeeded()
                        self.SetChassisMode(CHASSIS_MODE.AUTO_SEPARATE_GIMBAL)
                        print 'GAIN_BUFF:Chassis Arrived Buff Area!'
                        return
                    #print 'Odom has running for %i seconds' % (rospy.get_time()-chassis_run_start_time)
                print 'GAIN_BUFF:Chassis Aggressive gain buff FAILED, Time out!'
                self._as.set_aborted()
                self.SetChassisMode(CHASSIS_MODE.AUTO_SEPARATE_GIMBAL)
                return
            elif route == 3:
                print 'GAIN_BUFF:Route 3 received'
                self.NavToBuff()
                if self.nav_to_error_code == 0:
                    self.SetChassisMode(CHASSIS_MODE.AUTO_SEPARATE_GIMBAL)
                    self._as.set_succeeded()
                    print 'GAIN_BUFF:Aggressive Gain Buff Route 3 SUCCEED!'
                return
            else:
                self._as.set_aborted()
                print 'No valied route'
                return
    
    # Set chassis mode to use odom navigation
    def SetChassisMode(self, chassis_mode):
        if self.chassis_mode_ == chassis_mode:
            return True
        rospy.wait_for_service("set_chassis_mode", timeout=5)
        print 'Set chassis mode service connected!'
        # chassis_mode_req_ = ChassisModeRequest()
        # chassis_mode_req_.chassis_mode = chassis_mode
        # haha = ChassisArriveState
        chassis_mode_msg = ChassisMode()
        # chassis_mode_msg._request_class.chassis_mode = 7
        call_status = self.chassis_mode_client.call(chassis_mode) 
        chassis_mode_rec_ = ChassisModeResponse()
        chassis_mode_rec_ = chassis_mode_msg._response_class

        if(call_status and chassis_mode_rec_):
            self.chassis_mode_ = chassis_mode
            print 'Set Chassis Mode to %i' % chassis_mode
            return True
        else: 
            print 'Set gimbal mode failed!'
            return False
        
    # return odom arrive result 
    def OdomCB(self, data):
        self.chassis_state_ = data.state
    
    # Call NavTo Action        
    def NavToBuff(self):
        print 'NavTo action is actived!'
        g = NavToActionGoal()
        for path in BuffPath:
            q = tf.transformations.quaternion_from_euler(0,0,path['yaw'])
            g.goal.navgoal.pose.position.x = path['x']
            g.goal.navgoal.pose.position.y = path['y']
            g.goal.navgoal.pose.orientation.z = q[2]
            g.goal.navgoal.pose.orientation.w = q[3]
            self._ac_navto.send_goal(g.goal, done_cb=self.done_cb, feedback_cb=None)
            print 'Waiting for %i point ...' % (path['id'])
            self._ac_navto.wait_for_result(timeout=rospy.Duration(10))
            if self.nav_to_error_code > 0:
                self._ac_navto.cancel_all_goals()
                self._as.set_aborted()
                print 'AGGRESSIVA_GAIN_BUFF: NavTo Failed!'
                return
            print 'The result of %i point in BuffPath is arrived!' % path['id']
    
    # Nav_to done result
    def done_cb(self,terminal_state,result):
        self.nav_to_error_code = result.error_code


if __name__ == "__main__":
    rospy.init_node("aggressive_gain_buff_node")
    agb = AggressiveGainBuffNode()
    rospy.spin()