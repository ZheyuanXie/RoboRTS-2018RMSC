#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <cv_bridge/cv_bridge.h>

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
/*
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "MagTrackAction.h"
*/
#include <fstream>
#include <streambuf>
#include "arguments.pb.h"
#include <google/protobuf/text_format.h>

using namespace std;
using namespace cv;
using namespace mag_track;

class MagTrack
{
public:
  MagTrack();
  ~MagTrack();

  void receiveCallback(const sensor_msgs::ImageConstPtr& msg);
  void Preprocessing(const Mat src, cv::Mat &dst);
  void EdgeDetection(Mat &edgeImg, cv::Mat &dst);
  bool FindContour(Mat &edgeImg, vector<Point2f> &contour);
  bool CalcTransform(const vector<Point2f> contour, cv::Vec3d &magRvec, cv::Vec3d &magTvec);
  bool Publish(const bool detected, const cv::Vec3d magRvec, const cv::Vec3d magTvec);
  void DrawRotatedRect(cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness);
  //void ActionCB(const mag_track::MagTrackGoal::ConstPtr &data);

private:
  bool loadArguments();
  void Init();

  ros::NodeHandle nh_; 
  ros::Publisher resultsPub_;
  image_transport::Subscriber sub_;
  image_transport::ImageTransport it_;
  //actionlib::SimpleActionServer<mag_track::MagTrackAction> as_;
  
  string sub_topic;
  Arguments args;
  cv::Mat dist_coeffs;
  cv::Mat camera_matrix;

  bool enable_debug_;

  int rect_area_threshold;
  double area_ratio_threshold;
  
  cv::Mat srcImg;
  cv::Mat visImg;
  cv::Mat grayImg;
  std::vector<cv::Point3f> modelPoints;
  
  int params[10];  

  bool running_;

};


