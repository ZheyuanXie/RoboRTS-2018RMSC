#!/usr/bin/env python
import rospy
import numpy as np
import laser_geometry.laser_geometry as lg
import tf

from sensor_msgs.msg import LaserScan, PointCloud2, PointCloud, ChannelFloat32
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point32, Vector3, Twist
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker

# environment
AB_HEIGHT = 0.55
MAP_ORIGIN_OFFSET_X = 0.25
MAP_ORIGIN_OFFSET_Y = 0.25
# servo
SERVO_AOV       = 40
TARGET_OFFSET_X  = 0.5
TARGET_OFFSET_Y = 0.05
KP_VX = 5
KP_VY = 10
KP_VYAW = 1
MAX_LINEAR_VEL  = 0.3
MAX_ANGULAR_VEL = 0.8

AmmoBoxPossiblePos = [
    # Grounded ammo boxs
    # (0.20,4.80,0), (0.70,4.80,0), 
    # (2.60,4.80,0), (0.20,3.60,0),
    # (0.60,3.10,0), (2.90,0.20,0),
    # Elevated ammo boxs
    {'center':(1.60,3.85,AB_HEIGHT)},
    {'center':(0.25,2.35,AB_HEIGHT)},
    {'center':(0.60,2.35,AB_HEIGHT)},
    {'center':(1.95,2.60,AB_HEIGHT)},
    {'center':(1.95,2.10,AB_HEIGHT)},
    {'center':(1.95,1.60,AB_HEIGHT)},
    {'center':(3.25,1.60,AB_HEIGHT)},
    {'center':(3.25,0.90,AB_HEIGHT)},
    {'center':(3.25,0.20,AB_HEIGHT)},
]

class  AmmoBoxLocation:
    def __init__(self,abp):
        self.x = abp['center'][0] + MAP_ORIGIN_OFFSET_X
        self.y = abp['center'][1] + MAP_ORIGIN_OFFSET_Y
        self.z = abp['center'][2]
        self.vote = 0
    
    def check(self,pc):
        self.vote = 0
        for p in pc.points:
            dist = np.sqrt((p.x - self.x)**2 + (p.y - self.y)**2)
            if dist < 0.25:
                self.vote = 1
                break

class TopLidarServo:
    def __init__(self):
        rospy.init_node("top_lidar_servo")
        self.tfListener = tf.TransformListener()
        self.lp = lg.LaserProjection()
        rospy.sleep(0.1)

        # initialize instance variables
        self.servo_activate = True

        # initialize a list for all possible ammobox sites
        self.ammobox_list = []
        for abp in AmmoBoxPossiblePos:
            self.ammobox_list.append(AmmoBoxLocation(abp))

        # process laserscan data
        self.sub_top_lidar = rospy.Subscriber("/scan2", LaserScan, self.TopLidarCB)
        self.sub_base_lidar = rospy.Subscriber("/scan", LaserScan, self.BaseLidarCB)
        self.pub_pc = rospy.Publisher("/converted_pc", PointCloud, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.cmd_vel = Twist()

        # publish the marker array to visualize in rviz
        self.pub_markers = rospy.Publisher("/ammobox_markers", MarkerArray, queue_size=1)
        self.pub_timer = rospy.Timer(rospy.Duration(0.1), self.TimerCB)


    def TopLidarCB(self,data):
        rospy.sleep(0.005)
        local_pc = self.GetLocalPC(data)

        # global point cloud for ammobox matching
        global_pc = self.tfListener.transformPointCloud('map',local_pc)
        self.pub_pc.publish(global_pc)
        for ab in self.ammobox_list:
            ab.check(global_pc)

        if self.servo_activate:
            theta = np.deg2rad(np.linspace(-SERVO_AOV/2,SERVO_AOV/2,num=SERVO_AOV+1))
            dist = np.array(data.ranges[179-SERVO_AOV/2:179+SERVO_AOV/2+1])
            range_cut_index = list(np.where(np.abs(dist - 0.5)>0.2)[0])
            theta_cut = np.delete(theta, range_cut_index)
            dist_cut = np.delete(dist, range_cut_index)
            y = dist_cut * np.sin(theta_cut)
            mean_y = np.average(y)
            self.cmd_vel.linear.y = (TARGET_OFFSET_Y - mean_y) * KP_VY if self.cmd_vel.angular.z < 0.3 else 0

    def BaseLidarCB(self,data):
        if self.servo_activate:
            theta = np.deg2rad(np.linspace(-SERVO_AOV/2,SERVO_AOV/2,num=SERVO_AOV+1))
            dist = np.array(data.ranges[180-SERVO_AOV/2:180+SERVO_AOV/2+1])
            range_cut_index = list(np.where(np.abs(dist)>0.8)[0])
            theta_cut = np.delete(theta, range_cut_index)
            dist_cut = np.delete(dist, range_cut_index)
            x = dist_cut * np.cos(theta_cut)
            y = dist_cut * np.sin(theta_cut)
            mean_x = np.average(x)
            mean_angle = np.arctan(1/np.polyfit(x, y, 1)[0])
            self.cmd_vel.linear.x = (TARGET_OFFSET_X - mean_x) * KP_VX
            self.cmd_vel.angular.z = (0 - mean_angle) * KP_VYAW
            self.SendCmdVel()

    def GetLocalPC(self,data):
        pc2_msg = self.lp.projectLaser(data)
        point_generator = pc2.read_points(pc2_msg)
        pc_msg = PointCloud()
        pc_msg.header = pc2_msg.header
        pc_msg.channels.append(ChannelFloat32())
        for p in point_generator:
            pc_msg.points.append(Point32(p[0],p[1],p[2]))
            pc_msg.channels[0].values.append(0)
        return pc_msg
    
    def TimerCB(self,event):
        markerArray = MarkerArray()
        ammobox_detected = 0
        for ab in self.ammobox_list:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale = Vector3(0.2,0.2,0.2)
            marker.color = ColorRGBA(1,0,0,1) if ab.vote == 1 else ColorRGBA(1,1,1,0.5)
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = ab.x
            marker.pose.position.y = ab.y
            marker.pose.position.z = ab.z
            markerArray.markers.append(marker)
            if ab.vote == 1: ammobox_detected += 1
        # renumber the markers
        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1
        self.pub_markers.publish(markerArray)
    
    def SendCmdVel(self):
        self.cmd_vel.linear.x = np.clip(self.cmd_vel.linear.x, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        self.cmd_vel.linear.y = np.clip(self.cmd_vel.linear.y, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        self.cmd_vel.angular.z = np.clip(self.cmd_vel.angular.z, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
        self.pub_cmd_vel.publish(self.cmd_vel)
        print self.cmd_vel

if __name__ == "__main__":
    tls = TopLidarServo()
    rospy.spin()