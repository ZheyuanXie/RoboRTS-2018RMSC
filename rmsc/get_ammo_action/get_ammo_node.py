#! /usr/bin/env python
import rospy
from actionlib import SimpleActionServer, SimpleActionClient
import numpy as np
import tf

from sensor_msgs.msg import LaserScan, PointCloud2, PointCloud, ChannelFloat32
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point32, Vector3, Twist, PointStamped, PoseStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
from messages.msg import LocalPlannerAction, GlobalPlannerAction
from messages.msg import NavToAction, NavToGoal
from messages.msg import LookAndMoveAction, LookAndMoveGoal
from messages.msg import GetAmmoAction, GetAmmoActionResult, GetAmmoActionFeedback
from messages.msg import GripperCmd, GripperInfo
from actionlib_msgs.msg import GoalStatus

from gripper import GripperController

# debug mode
SERVO_ONLY = False
VISUAL_ONLY = False
NO_GRASP = False

# environment
AB_HEIGHT = 0.55
MAP_ORIGIN_OFFSET_X = 0.25
MAP_ORIGIN_OFFSET_Y = 0.25
MAP_WIDTH = 5.5
MAP_LENGTH = 8.5
# servo
SERVO_AOV       = 40
TARGET_OFFSET_X  = 0.5
TARGET_OFFSET_Y = 0.05
KP_VX = 3.0
KP_VY = 4.5
KP_VYAW = 0.3
MAX_LINEAR_VEL  = 0.2
MAX_ANGULAR_VEL = 0.8
# blind
BLIND_APPROACH_VX = -0.1
WITHDRAW_VX = 0.5
# error tolerance
X_ERROR = 0.02
Y_ERROR = 0.03
YAW_ERROR = 0.2

AmmoBoxes = [
    # ground ----------------------------------
    # {'id':1,  'center':(1.60,3.85,AB_HEIGHT),'checkpoint':(2.00,3.60,0),'type':False},
    {'id':2,  'center':(1.60,3.85,AB_HEIGHT),'checkpoint':(1.20,4.30,-1.57),'type':False},
    {'id':3,  'center':(1.60,3.85,AB_HEIGHT),'checkpoint':(2.60,4.20,-1.57),'type':False},
    {'id':4,  'center':(1.60,3.85,AB_HEIGHT),'checkpoint':(0.50,4.65,1.57),'type':False},
    {'id':5,  'center':(1.60,3.85,AB_HEIGHT),'checkpoint':(1.05,4.30,1.57),'type':False},
    # {'id':6,  'center':(1.60,3.85,AB_HEIGHT),'checkpoint':(2.00,3.60,0),'type':False},
    # elevated ----------------------------------
    {'id':7,  'center':(1.50,3.85,AB_HEIGHT),'checkpoint':(2.00,3.60,-1.57),'type':True},
    # {'id':8,  'center':(0.60,2.35,AB_HEIGHT),'checkpoint':(0.88,3.26,1.57),'type':True},
    {'id':9,  'center':(0.60,2.35,AB_HEIGHT),'checkpoint':(0.88,3.26,1.57),'type':True},
    {'id':10, 'center':(2.10,2.20,AB_HEIGHT),'checkpoint':(2.80,2.90,0.00),'type':True},
    {'id':11, 'center':(2.05,1.80,AB_HEIGHT),'checkpoint':(1.50,2.50,3.14),'type':True},
    {'id':12, 'center':(2.05,1.40,AB_HEIGHT),'checkpoint':(1.50,2.00,3.14),'type':True},
    {'id':13, 'center':(3.25,1.60,AB_HEIGHT),'checkpoint':(4.20,1.75,0),'type':True},
    {'id':14, 'center':(3.25,1.00,AB_HEIGHT),'checkpoint':(4.20,1.20,0),'type':True},
    {'id':15, 'center':(3.25,0.40,AB_HEIGHT),'checkpoint':(4.20,0.50,0),'type':True},
    # enemy ground ----------------------------------
    # {'id':16,  'center':(1.60,3.85,AB_HEIGHT),'checkpoint':(2.00,3.60,0),'type':False},
    {'id':17,  'center':(1.60,3.85,AB_HEIGHT),'checkpoint':(MAP_LENGTH - 1.20, MAP_WIDTH - 4.30,-1.57+3.14),'type':False},
    {'id':18,  'center':(1.60,3.85,AB_HEIGHT),'checkpoint':(MAP_LENGTH - 2.60, MAP_WIDTH - 4.20,-1.57+3.14),'type':False},
    {'id':19,  'center':(1.60,3.85,AB_HEIGHT),'checkpoint':(MAP_LENGTH - 0.50, MAP_WIDTH - 4.65,1.57-3.14),'type':False},
    {'id':20,  'center':(1.60,3.85,AB_HEIGHT),'checkpoint':(MAP_LENGTH - 1.05, MAP_WIDTH - 4.30,1.57-3.14),'type':False},
    # {'id':21,  'center':(1.60,3.85,AB_HEIGHT),'checkpoint':(2.00,3.60,0),'type':False},
    # enemy elevated ----------------------------------
    {'id':22,  'center':(1.60,3.85,AB_HEIGHT),'checkpoint':(MAP_LENGTH - 2.00, MAP_WIDTH - 3.60,-1.57+3.14),'type':True},
    # {'id':23,  'center':(0.60,2.35,AB_HEIGHT),'checkpoint':(0.88,3.26,1.57),'type':True},
    {'id':24,  'center':(0.60,2.35,AB_HEIGHT),'checkpoint':(MAP_LENGTH - 0.88, MAP_WIDTH - 3.26,1.57+3.14),'type':True},
    {'id':25, 'center':(2.10,2.20,AB_HEIGHT),'checkpoint':(MAP_LENGTH - 2.80, MAP_WIDTH - 2.90,0.00+3.14),'type':True},
    {'id':26, 'center':(2.05,1.80,AB_HEIGHT),'checkpoint':(MAP_LENGTH - 1.50, MAP_WIDTH - 2.50,0.00),'type':True},
    {'id':27, 'center':(2.05,1.40,AB_HEIGHT),'checkpoint':(MAP_LENGTH - 1.50, MAP_WIDTH - 2.00,0.00),'type':True},
    {'id':28, 'center':(3.25,1.60,AB_HEIGHT),'checkpoint':(MAP_LENGTH - 4.20, MAP_WIDTH - 1.75,0+3.14),'type':True},
    {'id':29, 'center':(3.25,1.00,AB_HEIGHT),'checkpoint':(MAP_LENGTH - 4.20, MAP_WIDTH - 1.20,0+3.14),'type':True},
    {'id':30, 'center':(3.25,0.40,AB_HEIGHT),'checkpoint':(MAP_LENGTH - 4.20, MAP_WIDTH - 0.50,0+3.14),'type':True}
]

class AmmoType:
    GROUND      = 0
    ELEVATED    = 1 

class GetAmmoStatus:
    IDLE        = 0
    MOVETO      = 1
    SERVO       = 2
    BLIND       = 3
    GRASP       = 4
    WITHDRAW    = 5
    LOOKMOVE    = 6

class GetAmmoNode(object):
    _feedback = GetAmmoActionFeedback()
    _result = GetAmmoActionResult()
    def __init__(self, name="get_ammo_node_action"):
        self.action_name = name
        self._as = SimpleActionServer(self.action_name, GetAmmoAction, execute_cb=self.ExecuteCB, auto_start=False)
        self._as.start()

        # initialize gripper
        self.gripper = GripperController()
        self.gripper.SetState(GripperCmd.NORMAL)

        # initialize navto action server
        if not SERVO_ONLY and not VISUAL_ONLY:
            self._ac_navto = SimpleActionClient("nav_to_node_action", NavToAction)
            rospy.loginfo('tring to connect NavTo action server...')
            ret = self._ac_navto.wait_for_server(timeout=rospy.Duration(5.0))
            rospy.loginfo('NavTo sever connected!' if ret else 'error: NavTo server not started!')
        
        if not SERVO_ONLY:
            self._ac_lookmove = SimpleActionClient("look_n_move_node_action", LookAndMoveAction)
            rospy.loginfo('tring to connect LookAndMove action server...')
            ret = self._ac_lookmove.wait_for_server(timeout=rospy.Duration(5.0))
            rospy.loginfo('LookAndMove sever connected!') if ret else rospy.loginfo('error: LookAndMove server not started!')

        self.state = GetAmmoStatus.IDLE
        self.ammotype = AmmoType.ELEVATED
        # navigation phase
        self.navto_start_time = 0
        self.navto_reached = False
        self.navto_failed = False
        # approach phase
        self.servo_cnt = 0
        self.servo_no_target = 0
        self.servo_base_reached = False
        self.servo_top_reached = False
        #---
        self.looknmove_start_time = 0
        self.looknmove_cnt = 0
        self.looknmove_no_target = 0
        self.looknmove_issued = False
        self.looknmove_reached = False
        self.looknmove_failed = False
        # final phase
        self.blind_cnt = 0
        self.touch_cnt = 0
        self.withdraw_cnt = 0

        # process laserscan data
        self.sub_top_lidar = rospy.Subscriber("scan2", LaserScan, self.TopLidarCB)
        self.sub_base_lidar = rospy.Subscriber("scan", LaserScan, self.BaseLidarCB)
        self.sub_visual_pose = rospy.Subscriber("visual_pose", PoseStamped, self.VisualPoseCB)
        self.pub_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.cmd_vel = Twist()
    
    def ExecuteCB(self, goal):
        rate = rospy.Rate(20)
        
        if SERVO_ONLY:
            self.SetStateServo()
        if VISUAL_ONLY:
            self.SetStateLookMove()
        else:
            self.state = GetAmmoStatus.MOVETO
            id_found = False
            box_info = {}
            for box in AmmoBoxes:
                if box['id'] == goal.ammobox_index:
                    box_info = box
                    id_found = True
            if id_found == False:
                rospy.loginfo('ID Not Found, Aborted')
                self._as.set_aborted()
                return
            checkpoint = box_info.get('checkpoint')
            self.ammotype = box_info.get('type')
            if checkpoint is None:
                rospy.loginfo('No CheckPoint Info, Aborted')
                self._as.set_aborted()
                return
            g = NavToGoal()
            g.navgoal.pose.position.x = checkpoint[0]
            g.navgoal.pose.position.y = checkpoint[1]
            quat = tf.transformations.quaternion_from_euler(0.,0.,checkpoint[2])
            g.navgoal.pose.orientation.z = quat[2]
            g.navgoal.pose.orientation.w = quat[3]
            self.navto_reached = False
            self.navto_failed = False
            self.navto_start_time = rospy.get_time()
            self._ac_navto.send_goal(g,done_cb=self.NavToDoneCB)
        
        self.gripper.SetState(GripperCmd.NORMAL)
        while not rospy.is_shutdown():
            # Check Preempt Request
            if self._as.is_preempt_requested():
                rospy.loginfo('preempt requested')
                self._ac_navto.cancel_all_goals()
                self._as.set_preempted()
                self.state = GetAmmoStatus.IDLE
                break
                
            # Finite State Machine
            if self.state == GetAmmoStatus.MOVETO:
                if self.navto_reached or (rospy.get_time() - self.navto_start_time > 15):
                    self._ac_navto.cancel_all_goals()
                    if self.ammotype == AmmoType.ELEVATED:
                        self.SetStateServo()
                    else:
                        self.SetStateLookMove()
                elif self.navto_failed:
                    self._as.set_aborted()
                    self.state = GetAmmoStatus.IDLE
                    break


            elif self.state == GetAmmoStatus.SERVO:
                self.servo_cnt += 1
                self.pub_cmd_vel.publish(self.cmd_vel)
                if self.servo_no_target > 10:
                    self._as.set_aborted()
                    self.state = GetAmmoStatus.IDLE
                    break
                if self.servo_base_reached and self.servo_top_reached:
                    if NO_GRASP:
                        self._as.set_succeeded()
                        self.state = GetAmmoStatus.IDLE
                        break
                    self.gripper.SetState(GripperCmd.GRIP_HIGH)
                    self.blind_cnt = 0
                    self.state = GetAmmoStatus.BLIND
                # SERVO TIME-OUT
                if self.servo_cnt > 80:
                    self._as.set_aborted()
                    self.state = GetAmmoStatus.IDLE
                    break
            
            elif self.state == GetAmmoStatus.LOOKMOVE:
                if rospy.get_time() - self.looknmove_start_time > 4 and not self.looknmove_issued:
                    self._as.set_aborted()
                    self.state = GetAmmoStatus.IDLE
                    break
                if self.looknmove_cnt == 1:
                    self.gripper.SetPosition(100,200)
                self.looknmove_cnt += 1
                if self.looknmove_no_target > 15:
                    self._as.set_aborted()
                    self.state = GetAmmoStatus.IDLE
                    break
                if self.looknmove_reached:
                    if NO_GRASP:
                        self._as.set_succeeded()
                        self.state = GetAmmoStatus.IDLE
                        break
                    self.blind_cnt = 0
                    self.gripper.SetState(GripperCmd.GRIP_LOW)
                    self.state = GetAmmoStatus.BLIND
                self.blind_cnt = 0
                # TIME-OUT
                if self.looknmove_failed:# or self.visual_cnt > 120:
                    self._as.set_aborted()
                    self.state = GetAmmoStatus.IDLE
                    break

            elif self.state == GetAmmoStatus.BLIND:
                self.blind_cnt += 1
                if self.gripper.feedback == GripperInfo.TOUCHED: 
                    self.touch_cnt = 0
                    self.state = GetAmmoStatus.GRASP
                # BLIND APPROACH TIME-OUT
                if self.blind_cnt > 80:
                    self.SendCmdVel(WITHDRAW_VX,0.,0.)
                    rospy.logwarn('BLIND_WITH')
                else:
                    self.SendCmdVel(BLIND_APPROACH_VX,0.,0.)
                    rospy.logwarn('BLIND_APP')
                if self.blind_cnt > 100:
                    rospy.logwarn('BLIND_EXIT')
                    self._as.set_aborted()
                    self.state = GetAmmoStatus.IDLE
                    break
                    
            elif self.state == GetAmmoStatus.GRASP:
                self.touch_cnt += 1
                if self.touch_cnt > 10:
                    self.SendCmdVel(0.,0.,0.)
                else:
                    self.SendCmdVel(WITHDRAW_VX,0.,0.)
                if self.gripper.feedback == GripperInfo.DONE:
                    self.withdraw_cnt = 0
                    self.state = GetAmmoStatus.WITHDRAW

            elif self.state == GetAmmoStatus.WITHDRAW:
                self.SendCmdVel(WITHDRAW_VX,0.,0.)
                self.withdraw_cnt += 1
                if self.withdraw_cnt > 15:
                    self._as.set_succeeded()
                    self.state = GetAmmoStatus.IDLE
                    break
            rate.sleep()
        self.gripper.SetState(GripperCmd.NORMAL)
        rospy.sleep(.1)
        self.SendCmdVel(0.,0.,0.)
        rospy.sleep(.1)
        self.SendCmdVel(0.,0.,0.)
    
    def NavToDoneCB(self,terminal_state,result):
        if terminal_state == GoalStatus.SUCCEEDED:
            self.navto_reached = True
            self._ac_navto.cancel_all_goals()
            rospy.loginfo('Navto reached')
        else:
            self.navto_failed = True
            rospy.loginfo('Navto failed')
    
    def TopLidarCB(self,data):
        if self.state == GetAmmoStatus.SERVO:
            theta = np.deg2rad(np.linspace(-SERVO_AOV/2,SERVO_AOV/2,num=SERVO_AOV+1))
            dist = np.array(data.ranges[179-SERVO_AOV/2:179+SERVO_AOV/2+1])
            range_cut_index = list(np.where(np.abs(dist - 0.5)>0.2)[0])
            theta_cut = np.delete(theta, range_cut_index)
            dist_cut = np.delete(dist, range_cut_index)
            if theta_cut.size == 0:
                self.servo_no_target += 1
                rospy.loginfo('TOP LIDAR: NO TARTGET')
                return
            y = dist_cut * np.sin(theta_cut)
            mean_y = np.average(y)
            self.cmd_vel.linear.y = (TARGET_OFFSET_Y - mean_y) * KP_VY if self.cmd_vel.angular.z < 0.3 else 0
            self.servo_top_reached = np.abs(TARGET_OFFSET_Y - mean_y) < X_ERROR
            
    def BaseLidarCB(self,data):
        if self.state == GetAmmoStatus.SERVO:
            theta = np.deg2rad(np.linspace(-SERVO_AOV/2,SERVO_AOV/2,num=SERVO_AOV+1))
            dist = np.array(data.ranges[180-SERVO_AOV/2:180+SERVO_AOV/2+1])
            range_cut_index = list(np.where(np.abs(dist)>0.8)[0])
            theta_cut = np.delete(theta, range_cut_index)
            dist_cut = np.delete(dist, range_cut_index)
            if theta_cut.size == 0:
                rospy.loginfo('BASE LIDAR: NO TARTGET')
                return
            x = dist_cut * np.cos(theta_cut)
            y = dist_cut * np.sin(theta_cut)
            mean_x = np.average(x)
            mean_angle = np.arctan(1/np.polyfit(x, y, 1)[0])
            self.cmd_vel.linear.x = (TARGET_OFFSET_X - mean_x) * KP_VX
            self.cmd_vel.angular.z = (0 - mean_angle) * KP_VYAW
            self.servo_base_reached = np.abs(TARGET_OFFSET_X - mean_x) < Y_ERROR and np.abs(0-mean_angle) < YAW_ERROR
            #print mean_x,mean_angle
    
    def VisualPoseCB(self,data):
        if self.state == GetAmmoStatus.LOOKMOVE and self.looknmove_issued == False and self.looknmove_cnt > 20:
            if data.pose.position.x == 0:
                self.looknmove_no_target += 1
                rospy.loginfo('CAMERA: NO TARGET')
            else:
                err_y = (data.pose.position.z - 300) / 1000
                err_x = data.pose.position.x / 1000
                goal = LookAndMoveGoal()
                goal.relative_pose.header.frame_id = "base_link"
                goal.relative_pose.pose.position.x = -err_y
                goal.relative_pose.pose.position.y = err_x
                self._ac_lookmove.cancel_all_goals()
                print goal
                self._ac_lookmove.send_goal(goal, done_cb=self.LookAndMoveDoneCB)
                self.looknmove_issued = True
    
    def LookAndMoveDoneCB(self,terminal_state,result):
        print terminal_state, result
        if terminal_state == GoalStatus.SUCCEEDED:
            self.looknmove_reached = True
            self._ac_lookmove.cancel_all_goals()
            rospy.loginfo('LooknMove reached')
        else:
            self.looknmove_failed = True
            rospy.loginfo('LooknMove failed')

    def SendCmdVel(self, vx, vy, vyaw):
        self.cmd_vel.linear.x = np.clip(vx, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        self.cmd_vel.linear.y = np.clip(vy, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        self.cmd_vel.angular.z = np.clip(vyaw, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
        self.pub_cmd_vel.publish(self.cmd_vel)
        # print self.cmd_vel
    
    def SetStateLookMove(self):
        self.looknmove_start_time = rospy.get_time()
        self.looknmove_cnt = 0
        self.looknmove_no_target = 0
        self.looknmove_issued = False
        self.looknmove_reached = False
        self.looknmove_failed = False
        self.state = GetAmmoStatus.LOOKMOVE

    def SetStateServo(self):
        self.servo_cnt = 0
        self.servo_no_target = 0
        self.servo_base_reached = False
        self.servo_top_reached = False
        self.state = GetAmmoStatus.SERVO



if __name__ == "__main__":
    rospy.init_node("get_ammo_node")
    ga = GetAmmoNode()
    rospy.spin()
