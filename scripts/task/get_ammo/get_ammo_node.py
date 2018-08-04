#! /usr/bin/env python
import rospy
from actionlib import SimpleActionServer, SimpleActionClient
import numpy as np
import tf

from sensor_msgs.msg import LaserScan, PointCloud2, PointCloud, ChannelFloat32
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point32, Vector3, Twist, PointStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
from messages.msg import LocalPlannerAction, GlobalPlannerAction
from messages.msg import NavToAction, NavToActionGoal
from messages.msg import GetAmmoAction, GetAmmoActionResult, GetAmmoActionFeedback
from actionlib_msgs.msg import GoalStatus

from GripperControl import GripperController, GripperState, GripperFeedback

# debug mode
SERVO_ONLY = False
NO_GRASP = True

# environment
AB_HEIGHT = 0.55
MAP_ORIGIN_OFFSET_X = 0.25
MAP_ORIGIN_OFFSET_Y = 0.25
# servo
SERVO_AOV       = 40
TARGET_OFFSET_X  = 0.5
TARGET_OFFSET_Y = 0.05
KP_VX = 3.5
KP_VY = 4
KP_VYAW = 1
MAX_LINEAR_VEL  = 0.2
MAX_ANGULAR_VEL = 0.8
# blind
BLIND_APPROACH_VX = -0.1
WITHDRAW_VX = 0.2

AmmoBoxes = [
    {'id':7,  'center':(1.60,3.85,AB_HEIGHT),'checkpoint':(1.70,3.30,-1.57)},
    {'id':8,  'center':(0.25,2.35,AB_HEIGHT),'checkpoint':(0.84,3.06,1.57)},
    {'id':9,  'center':(0.60,2.35,AB_HEIGHT),'checkpoint':(0.40,3.00,0.00)},
    {'id':10, 'center':(1.95,2.60,AB_HEIGHT),'checkpoint':(2.80,2.90,0.00)},
    {'id':11, 'center':(1.95,2.10,AB_HEIGHT),'checkpoint':(2.80,2.50,0.00)},
    {'id':12, 'center':(1.95,1.60,AB_HEIGHT),'checkpoint':(2.80,2.00,0.00)},
    {'id':13, 'center':(3.25,1.60,AB_HEIGHT),'checkpoint':(2.80,2.00,3.14)},
    {'id':14, 'center':(3.25,0.90,AB_HEIGHT),'checkpoint':(2.80,1.20,3.14)},
    {'id':15, 'center':(3.25,0.20,AB_HEIGHT),'checkpoint':(2.80,0.50,3.14)}
]

class GetAmmoStatus:
    IDLE        = 0
    MOVETO      = 1
    SERVO       = 2
    BLIND       = 3
    GRASP       = 4
    WITHDRAW    = 5

class GetAmmo(object):
    _feedback = GetAmmoActionFeedback()
    _result = GetAmmoActionResult()
    def __init__(self, name):
        self.action_name = name
        self._as = SimpleActionServer(self.action_name, GetAmmoAction, execute_cb=self.ExecuteCB, auto_start=False)
        self._as.start()

        # initialize gripper
        self.gripper = GripperController()
        self.gripper.Initialize()

        # initialize navto action server
        if not SERVO_ONLY:
            self._ac_navto = SimpleActionClient("navigation_node_action", NavToAction)
            print 'tring to connect NavTo action server...'
            ret = self._ac_navto.wait_for_server(timeout=rospy.Duration(5.0))
            print 'sever connected!' if ret else 'error: server not started!'

        self.state = GetAmmoStatus.IDLE
        self.navto_reached = False
        self.navto_failed = False
        self.servo_base_reached = False
        self.servo_top_reached = False
        self.withdraw_cnt = 0

        # process laserscan data
        self.sub_top_lidar = rospy.Subscriber("/scan2", LaserScan, self.TopLidarCB)
        self.sub_base_lidar = rospy.Subscriber("/scan", LaserScan, self.BaseLidarCB)
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.cmd_vel = Twist()
    
    def ExecuteCB(self, goal):
        rate = rospy.Rate(20)
        
        if SERVO_ONLY:
            self.servo_base_reached = False
            self.servo_top_reached = False
            self.state = GetAmmoStatus.SERVO
        else:
            self.state = GetAmmoStatus.MOVETO
            id_found = False
            box_info = {}
            for box in AmmoBoxes:
                if box['id'] == goal.ammobox_index:
                    box_info = box
                    id_found = True
            if id_found == False:
                print 'ID Not Found, Aborted'
                self._as.set_aborted()
                return
            checkpoint = box_info.get('checkpoint')
            if checkpoint is None:
                print 'No CheckPoint Info, Aborted'
                self._as.set_aborted()
                return
            g = NavToActionGoal()
            g.goal.navgoal.pose.position.x = checkpoint[0]
            g.goal.navgoal.pose.position.y = checkpoint[1]
            quat = tf.transformations.quaternion_from_euler(0.,0.,checkpoint[2])
            g.goal.navgoal.pose.orientation.z = quat[2]
            g.goal.navgoal.pose.orientation.w = quat[3]
            self.navto_reached = False
            self.navto_failed = False
            self._ac_navto.send_goal(g.goal,done_cb=self.NavToDoneCB)
        
        self.gripper.SendGripperCmd(GripperState.Normal)
        while not rospy.is_shutdown():
            # Check Preempt Request
            if self._as.is_preempt_requested():
                print 'preempt requested'
                self._as.set_preempted()
                self.state = GetAmmoStatus.IDLE
                break
            # Finite State Machine
            if self.state == GetAmmoStatus.MOVETO:
                if self.navto_reached:
                    self.servo_top_reached = False
                    self.servo_bases_reached = False
                    self.state = GetAmmoStatus.SERVO
                elif self.navto_failed:
                    self._as.set_aborted()
                    self.state = GetAmmoStatus.IDLE
                    break

            elif self.state == GetAmmoStatus.SERVO:
                self.pub_cmd_vel.publish(self.cmd_vel)
                if self.servo_base_reached and self.servo_top_reached:
                    if NO_GRASP:
                        self._as.set_succeeded()
                        self.state = GetAmmoStatus.IDLE
                        break
                    self.gripper.SendGripperCmd(GripperState.Get_High)
                    self.state = GetAmmoStatus.BLIND

            elif self.state == GetAmmoStatus.BLIND:
                self.SendCmdVel(BLIND_APPROACH_VX,0.,0.)
                if self.gripper.feedback == GripperFeedback.Touched:
                    # Gripper movement is triggered automatically by the MCU
                    self.state = GetAmmoStatus.GRASP
                    
            elif self.state == GetAmmoStatus.GRASP:
                self.SendCmdVel(0.,0.,0.)
                if self.gripper.feedback == GripperFeedback.Done:
                    self.withdraw_cnt = 0
                    self.state = GetAmmoStatus.WITHDRAW

            elif self.state == GetAmmoStatus.WITHDRAW:
                self.SendCmdVel(WITHDRAW_VX,0.,0.)
                self.withdraw_cnt += 1
                if self.withdraw_cnt > 20:
                    self._as.set_succeeded()
                    self.state = GetAmmoStatus.IDLE
                    break
            rate.sleep()
        self.gripper.SendGripperCmd(GripperState.Normal)
        rospy.sleep(.1)
        self.SendCmdVel(0.,0.,0.)
        rospy.sleep(.1)
        self.SendCmdVel(0.,0.,0.)
    
    def NavToDoneCB(self,terminal_state,result):
        if terminal_state == GoalStatus.SUCCEEDED:
            self.navto_reached = True
            print 'Navto reached'
        else:
            self.navto_failed = True
            print 'Navto failed'
    
    def TopLidarCB(self,data):
        if self.state == GetAmmoStatus.SERVO:
            theta = np.deg2rad(np.linspace(-SERVO_AOV/2,SERVO_AOV/2,num=SERVO_AOV+1))
            dist = np.array(data.ranges[179-SERVO_AOV/2:179+SERVO_AOV/2+1])
            range_cut_index = list(np.where(np.abs(dist - 0.5)>0.2)[0])
            theta_cut = np.delete(theta, range_cut_index)
            dist_cut = np.delete(dist, range_cut_index)
            if theta_cut.size == 0:
                print 'TOP LIDAR: NO TARTGET'
                return
            y = dist_cut * np.sin(theta_cut)
            mean_y = np.average(y)
            self.cmd_vel.linear.y = (TARGET_OFFSET_Y - mean_y) * KP_VY if self.cmd_vel.angular.z < 0.3 else 0
            self.servo_top_reached = np.abs(TARGET_OFFSET_Y - mean_y) < 0.025

    def BaseLidarCB(self,data):
        if self.state == GetAmmoStatus.SERVO:
            theta = np.deg2rad(np.linspace(-SERVO_AOV/2,SERVO_AOV/2,num=SERVO_AOV+1))
            dist = np.array(data.ranges[180-SERVO_AOV/2:180+SERVO_AOV/2+1])
            range_cut_index = list(np.where(np.abs(dist)>0.8)[0])
            theta_cut = np.delete(theta, range_cut_index)
            dist_cut = np.delete(dist, range_cut_index)
            if theta_cut.size == 0:
                print 'BASE LIDAR: NO TARTGET'
                return
            x = dist_cut * np.cos(theta_cut)
            y = dist_cut * np.sin(theta_cut)
            mean_x = np.average(x)
            mean_angle = np.arctan(1/np.polyfit(x, y, 1)[0])
            self.cmd_vel.linear.x = (TARGET_OFFSET_X - mean_x) * KP_VX
            self.cmd_vel.angular.z = (0 - mean_angle) * KP_VYAW
            self.servo_base_reached = np.abs(TARGET_OFFSET_X - mean_x) < 0.025 and np.abs(0-mean_angle) < 0.1
            #print mean_x,mean_angle

    def SendCmdVel(self, vx, vy, vyaw):
        self.cmd_vel.linear.x = np.clip(vx, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        self.cmd_vel.linear.y = np.clip(vy, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        self.cmd_vel.angular.z = np.clip(vyaw, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
        self.pub_cmd_vel.publish(self.cmd_vel)
    


if __name__ == "__main__":
    rospy.init_node("get_ammo_action_server_node")
    ga = GetAmmo("get_ammo")
    rospy.spin()