#! /usr/bin/env python
import rospy
from actionlib import SimpleActionServer
import numpy as np

from sensor_msgs.msg import LaserScan, PointCloud2, PointCloud, ChannelFloat32
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point32, Vector3, Twist, PointStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
from messages.msg import LocalPlannerAction, GlobalPlannerAction
from messages.msg import GetAmmoAction, GetAmmoActionResult, GetAmmoActionFeedback

from GripperControl import GripperController, GripperState, GripperFeedback

# environment
AB_HEIGHT = 0.55
MAP_ORIGIN_OFFSET_X = 0.25
MAP_ORIGIN_OFFSET_Y = 0.25
# servo
SERVO_AOV       = 40
TARGET_OFFSET_X  = 0.5
TARGET_OFFSET_Y = 0.05
KP_VX = 5
KP_VY = 7
KP_VYAW = 1
MAX_LINEAR_VEL  = 0.2
MAX_ANGULAR_VEL = 0.8
# blind
BLIND_APPROACH_VX = -0.1
WITHDRAW_VX = 0.2

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

        # process laserscan data
        self.sub_top_lidar = rospy.Subscriber("/scan2", LaserScan, self.TopLidarCB)
        self.sub_base_lidar = rospy.Subscriber("/scan", LaserScan, self.BaseLidarCB)
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.cmd_vel = Twist()

        self.state = GetAmmoStatus.IDLE
        self.mean_angle = 0.0
        self.base_reached = False
        self.top_reached = False
        self.withdraw_cnt = 0
    
    def ExecuteCB(self, goal):
        rospy.loginfo(goal)
        rate = rospy.Rate(20)
        self.state = GetAmmoStatus.SERVO
        self.gripper.SendGripperCmd(GripperState.Normal)
        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                print 'preempt requested'
                self._as.set_preempted()
                self.state = GetAmmoStatus.IDLE
                self.SendCmdVel(0.,0.,0.)
                self.gripper.SendGripperCmd(GripperState.Normal)
                self.base_reached = False
                self.top_reached = False
                return
            if self.state == GetAmmoStatus.MOVETO:
                pass
            elif self.state == GetAmmoStatus.SERVO:
                self.pub_cmd_vel.publish(self.cmd_vel)
                print self.base_reached, self.top_reached
                if self.base_reached and self.top_reached:
                    self.state = GetAmmoStatus.BLIND
                    self.gripper.SendGripperCmd(GripperState.Get_High)
            elif self.state == GetAmmoStatus.BLIND:
                self.SendCmdVel(BLIND_APPROACH_VX,0.,0.)
                if self.gripper.feedback == GripperFeedback.Touched:
                    self.state = GetAmmoStatus.GRASP
            elif self.state == GetAmmoStatus.GRASP:
                self.SendCmdVel(0.,0.,0.)
                if self.gripper.feedback == GripperFeedback.Done:
                    self.state = GetAmmoStatus.WITHDRAW
                    self.withdraw_cnt = 0
            elif self.state == GetAmmoStatus.WITHDRAW:
                self.SendCmdVel(WITHDRAW_VX,0.,0.)
                self.withdraw_cnt += 1
                if self.withdraw_cnt > 20:
                    self.state = GetAmmoStatus.IDLE
                    self.SendCmdVel(0.,0.,0.)
                    self.base_reached = False
                    self.top_reached = False
                    self._as.set_succeeded()
                    break
            print self.state
            rate.sleep()
    
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
            self.top_reached = np.abs(TARGET_OFFSET_Y - mean_y) < 0.01

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
            self.base_reached = np.abs(TARGET_OFFSET_X - mean_x) < 0.01 and np.abs(0-mean_angle) < 0.1

    def SendCmdVel(self, vx, vy, vyaw):
        self.cmd_vel.linear.x = np.clip(vx, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        self.cmd_vel.linear.y = np.clip(vy, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        self.cmd_vel.angular.z = np.clip(vyaw, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
        self.pub_cmd_vel.publish(self.cmd_vel)
    


if __name__ == "__main__":
    rospy.init_node("get_ammo_action_server_node")
    ga = GetAmmo("get_ammo")
    rospy.spin()
