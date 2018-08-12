#! /usr/bin/env python
import rospy
from actionlib import SimpleActionServer
from rmsc_messages.msg import MapMuxAction
from nav_msgs.msg import OccupancyGrid, MapMetaData

class MapMux(object):
    def __init__(self):
        self.sub_map0 = rospy.Subscriber("/map0/map",OccupancyGrid,self.map0CB)
        self.sub_map0_meta = rospy.Subscriber("/map0/map_metadata",MapMetaData,self.mapmeta0CB)
        self.sub_map4 = rospy.Subscriber("/map4/map",OccupancyGrid,self.map4CB)
        self.sub_map4_meta = rospy.Subscriber("/map4/map_metadata",MapMetaData,self.mapmeta4CB)
        self.sub_map5 = rospy.Subscriber("/map5/map",OccupancyGrid,self.map5CB)
        self.sub_map5_meta = rospy.Subscriber("/map5/map_metadata",MapMetaData,self.mapmeta5CB)
        self.sub_map6 = rospy.Subscriber("/map6/map",OccupancyGrid,self.map6CB)
        self.sub_map6_meta = rospy.Subscriber("/map6/map_metadata",MapMetaData,self.mapmeta6CB)
        self.sub_map34 = rospy.Subscriber("/map34/map",OccupancyGrid,self.map34CB)
        self.sub_map34_meta = rospy.Subscriber("/map34/map_metadata",MapMetaData,self.mapmeta34CB)
        self.sub_map35 = rospy.Subscriber("/map35/map",OccupancyGrid,self.map35CB)
        self.sub_map35_meta = rospy.Subscriber("/map35/map_metadata",MapMetaData,self.mapmeta35CB)
        self.sub_map36 = rospy.Subscriber("/map36/map",OccupancyGrid,self.map36CB)
        self.sub_map36_meta = rospy.Subscriber("/map36/map_metadata",MapMetaData,self.mapmeta36CB)
        self.map0 = OccupancyGrid()
        self.map0_meta = MapMetaData()
        self.map4 = OccupancyGrid()
        self.map4_meta = MapMetaData()
        self.map5 = OccupancyGrid()
        self.map5_meta = MapMetaData()
        self.map6 = OccupancyGrid()
        self.map6_meta = MapMetaData()
        self.map34 = OccupancyGrid()
        self.map34_meta = MapMetaData()
        self.map35 = OccupancyGrid()
        self.map35_meta = MapMetaData()
        self.map36 = OccupancyGrid()
        self.map36_meta = MapMetaData()
        
        self.pub_map = rospy.Publisher("/map",OccupancyGrid,queue_size=1)
        self.pub_map_meta = rospy.Publisher("/map_metadata",MapMetaData,queue_size=1)
        self._as = SimpleActionServer("map_mux_node_action", MapMuxAction, self.ExecuteCB, auto_start=False)
        self._as.start()
    
    def ExecuteCB(self, goal):
        map_index = goal.map_index
        rate = rospy.Rate(50)
        for _ in range(200):
            if map_index == 0:
                self.pub_map.publish(self.map0)
                self.pub_map_meta.publish(self.map0_meta)
            elif map_index == 1:
                self.pub_map.publish(self.map4)
                self.pub_map_meta.publish(self.map4_meta)
            elif map_index == 2:
                self.pub_map.publish(self.map5)
                self.pub_map_meta.publish(self.map5_meta)
            elif map_index == 3:
                self.pub_map.publish(self.map6)
                self.pub_map_meta.publish(self.map6_meta)
            elif map_index == 4:
                self.pub_map.publish(self.map34)
                self.pub_map_meta.publish(self.map34_meta)
            elif map_index == 5:
                self.pub_map.publish(self.map35)
                self.pub_map_meta.publish(self.map35_meta)
            elif map_index == 6:
                self.pub_map.publish(self.map36)
                self.pub_map_meta.publish(self.map36_meta)
            rate.sleep()
        self._as.set_succeeded()
    
    def map0CB(self,data):
        self.map0 = data
    
    def map4CB(self,data):
        self.map4 = data
    
    def map5CB(self,data):
        self.map5 = data
    
    def map6CB(self,data):
        self.map6 = data
    
    def map34CB(self,data):
        self.map34 = data
    
    def map35CB(self,data):
        self.map35 = data
    
    def map36CB(self,data):
        self.map36 = data
    
    def mapmeta0CB(self,data):
        self.map0_meta = data

    def mapmeta4CB(self,data):
        self.map4_meta = data
    
    def mapmeta5CB(self,data):
        self.map5_meta = data
    
    def mapmeta6CB(self,data):
        self.map6_meta = data
    
    def mapmeta34CB(self,data):
        self.map34_meta = data
    
    def mapmeta35CB(self,data):
        self.map35_meta = data
    
    def mapmeta36CB(self,data):
        self.map36_meta = data

if __name__ == "__main__":
    rospy.init_node("map_mux_node")
    mm = MapMux()
    rospy.spin()