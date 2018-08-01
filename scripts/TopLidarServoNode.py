#!/usr/bin/env python
import rospy
import numpy as np
import laser_geometry.laser_geometry as lg
import tf

from sensor_msgs.msg import LaserScan, PointCloud2, PointCloud, ChannelFloat32
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point32, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker


AB_HEIGHT = 0.55
MAP_ORIGIN_OFFSET_X = 0.25
MAP_ORIGIN_OFFSET_Y = 0.25

AmmoBoxPossiblePos = [
    #(0.20,4.80,0), (0.70,4.80,0), (2.60,4.80,0), (0.20,3.60,0),
    #(0.60,3.10,0), (2.90,0.20,0),
    (1.60,3.85,AB_HEIGHT), (0.20,2.35,AB_HEIGHT), (0.60,2.35,AB_HEIGHT),
    (1.95,2.60,AB_HEIGHT), (1.95,2.10,AB_HEIGHT), (1.95,1.60,AB_HEIGHT),
    (3.25,1.60,AB_HEIGHT), (3.25,0.90,AB_HEIGHT), (3.25,0.20,AB_HEIGHT)
]

class  AmmoBoxLocation:
    def __init__(self,x,y,z):
        self.x = x + MAP_ORIGIN_OFFSET_X
        self.y = y + MAP_ORIGIN_OFFSET_Y
        self.z = z
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

        # initialize a list for all possible ammobox sites
        self.ammobox_list = []
        for pos in AmmoBoxPossiblePos:
            self.ammobox_list.append(AmmoBoxLocation(pos[0],pos[1],pos[2]))

        # process laserscan data
        self.sub_lidar = rospy.Subscriber("/scan2", LaserScan, self.LidarCB)
        self.pub_pc = rospy.Publisher("/converted_pc", PointCloud, queue_size=1)

        # publish the marker array to visualize in rviz
        self.pub_timer = rospy.Timer(rospy.Duration(0.1), self.TimerCB)
        self.pub_markers = rospy.Publisher("/ammo_markers", MarkerArray, queue_size=1)

    def LidarCB(self,data):
        rospy.sleep(0.005)
        local_pc = self.GetLocalPC(data)

        global_pc = self.tfListener.transformPointCloud('map',local_pc)
        self.pub_pc.publish(global_pc)
        for ab in self.ammobox_list:
            ab.check(global_pc)

        # theta = np.deg2rad(np.linspace(0,359,num=360))
        # d = np.array(data.ranges)
        # x = d * np.cos(theta)
        # y = d * np.sin(theta)
    
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

if __name__ == "__main__":
    tls = TopLidarServo()
    rospy.spin()