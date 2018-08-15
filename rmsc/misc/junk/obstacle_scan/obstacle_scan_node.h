#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Int32MultiArray.h>

#include <iostream>
#include <fstream>
#include <streambuf>
#include "rmsc/obstacle_scan/proto/arguments.pb.h"
#include <google/protobuf/text_format.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <vector>
#include <iostream>
#include <string>

#include <actionlib/server/simple_action_server.h>
#include "rmsc_messages/ObstacleScanAction.h"

using namespace cv;
using namespace obstacle_scan;

class ObstacleScan 
{
     public:
        ObstacleScan();
        ~ObstacleScan();

        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        bool isPointInBox(const Point2f &point, const Point2f &centerOfBox, const char direction);
        float calcDistance(const Point2f &p1, const Point2f &p2);
        void ActionCB(const rmsc_messages::ObstacleScanGoal::ConstPtr &data);
        bool logicCheck();

       
      protected:
        Arguments args;
        bool loadArguments();
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<rmsc_messages::ObstacleScanAction> as_;

        laser_geometry::LaserProjection projector_;
        
        
        //ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
        bool flag;
        
        float xx,yy,zz;
        cv::Point2f obst_location[35][8];

        char obst_direction[35] = {'n', 'y', 'x', 'y', 'x', 'x', 'x'};

        int obst_exist[35];
};
