#!/usr/bin/env python
import rospy
from threading import Thread
import tf
import numpy as np
from actionlib import SimpleActionClient, SimpleActionServer
from messages.msg import LookAndMoveAction
from geometry_msgs.msg import PoseStamped, Twist

MAX_LINEAR_VEL  = 0.5
MAX_ANGULAR_VEL = 1
KP_X    = 2
KP_Y    = 2
KP_YAW  = 3

class LookAndMoveStatus:
    IDLE        = 0
    MOVING      = 0

class LookAndMove(object):
    cmd_vel = Twist()
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel',Twist,queue_size=1)
        
        self.target_pose = PoseStamped()
        self.observe_frame = ''
        self.state = LookAndMoveStatus.IDLE
        self._as = SimpleActionServer("looknmove", LookAndMoveAction, self.ExecuteCB, auto_start=False)
        self._as.start()

    def ExecuteCB(self,goal):
        print 'LOOKnMOVE GOAL RCV'
        ps = goal.relative_pose
        self.observe_frame = ps.header.frame_id if len(ps.header.frame_id)!=0 else 'base_link'
        ps.header.frame_id = self.observe_frame
        ps.header.stamp = rospy.Time(0)
        self.tf_listener.waitForTransform(self.observe_frame, 'odom', rospy.Time(0),rospy.Duration(1.0))
        self.target_pose = self.tf_listener.transformPose('odom',ps)
        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                print 'PREEMPT REQ'
                rospy.sleep(0.1)
                self.SendCmdVel(0.,0.,0.)
                self._as.set_preempted()
                return
            self.tf_listener.waitForTransform('odom','base_link',rospy.Time(0),rospy.Duration(1.0))
            pose_base = self.tf_listener.transformPose('base_link',self.target_pose)
            pose = pose_base.pose.position
            q = pose_base.pose.orientation
            yaw = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])[2]
            if np.abs(pose.x) < 0.02 and np.abs(pose.y) <0.02 and np.abs(yaw)<0.01:
                print 'SUCCESS'
                rospy.sleep(0.1)
                self.SendCmdVel(0.,0.,0.)
                self._as.set_succeeded()
                return
            vx = pose.x * KP_X
            vy = pose.y * KP_Y
            yaw = yaw * KP_YAW
            self.SendCmdVel(vx,vy,yaw)

    def SendCmdVel(self, vx, vy, vyaw):
        self.cmd_vel.linear.x = np.clip(vx, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        self.cmd_vel.linear.y = np.clip(vy, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        self.cmd_vel.angular.z = np.clip(vyaw, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
        self.pub_cmd_vel.publish(self.cmd_vel)
        #print self.cmd_vel

if __name__ == "__main__":
    rospy.init_node("looknmove_action_server_node")
    lam = LookAndMove()
    rospy.spin()