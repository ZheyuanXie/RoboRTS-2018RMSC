#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>

#include <fstream>
#include <streambuf>
#include "arguments.pb.h"
#include <google/protobuf/text_format.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <vector>
#include <iostream>
#include <string>

#include <actionlib/server/simple_action_server.h>
#include "rmsc_messages/SeerAction.h"

using namespace cv;
using namespace seer;
using namespace std;

class Seer 
{
     public:
        Seer();
        ~Seer();

        void highLidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void lowLidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        float calcDistance(const Point2f &p1, const Point2f &p2);
        void ActionCB(const rmsc_messages::SeerGoal::ConstPtr &data);
        void arrange();
       
      private:
        Arguments args;
        bool loadArguments();
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<rmsc_messages::SeerAction> as_;
        laser_geometry::LaserProjection projector_;
        
        ros::Subscriber high_scan_sub_;
        ros::Subscriber low_scan_sub_;
        ros::Publisher obstacle_pub_;
        ros::Publisher master_pub_;
        ros::Publisher wing_pub_;
        ros::Publisher ammobox_pub_;
        
        cv::Point2f obst_location[30];
        cv::Point2f ammobox_location[30];
       
        int which_obst;
        int obst_exist[30];
        bool ammobox_known, obst_known;
        int ammobox_exist[35] = {0};

        int trans[35] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15, 14, 13, 12, 11, 10, 7};
        bool running_;
        bool outputOnce;
};
