#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Transform, PoseStamped, Pose, Twist
import tf
import numpy as np

MAX_LINEAR_VEL  = 0.5
MAX_ANGULAR_VEL = 0.8

class LookAndMove(object):
    cmd_vel = Twist()
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.destination = PoseStamped()

        self.pub_cmd_vel = rospy.Publisher('cmd_vel',Twist,queue_size=5)

    def SetDestination(self, pose):
        ps = PoseStamped()
        ps.pose = pose
        ps.header.frame_id = 'base_link'
        ps.header.stamp = rospy.Time(0)
        self.tf_listener.waitForTransform('/base_link','/odom',rospy.Time(0),rospy.Duration(1.0))
        pose_odom = self.tf_listener.transformPose('odom',ps)
        self.destination = pose_odom
    
    def GoToDestination(self):
        while not rospy.is_shutdown():
            self.tf_listener.waitForTransform('/odom','/base_link',rospy.Time(0),rospy.Duration(1.0))
            pose_base = self.tf_listener.transformPose('base_link',self.destination)
            vx = pose_base.pose.position.x
            vy = pose_base.pose.position.y
            q = pose_base.pose.orientation
            yaw = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])[2]
            self.SendCmdVel(vx,vy,yaw)
        
    
    def SendCmdVel(self, vx, vy, vyaw):
        self.cmd_vel.linear.x = np.clip(vx, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        self.cmd_vel.linear.y = np.clip(vy, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        self.cmd_vel.angular.z = np.clip(vyaw, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
        self.pub_cmd_vel.publish(self.cmd_vel)

if __name__ == "__main__":
    rospy.init_node("look_and_move_test_node")
    lam = LookAndMove()
    p = Pose()
    p.position.x = 0
    p.position.y = -2
    p.orientation.w = 1
    lam.SetDestination(p)
    lam.GoToDestination()
    rospy.spin()