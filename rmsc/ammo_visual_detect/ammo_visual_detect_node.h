#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>

#include <string>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// #include <actionlib/server/simple_action_server.h>
// #include <actionlib/client/simple_action_client.h>
// #include <actionlib/client/terminal_state.h>
// #include "MagTrackAction.h"
// #include "LookAndMoveAction.h"

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

  bool loadArguments();
  void drawLines(cv::Mat &img, vector<Point> contour, cv::Scalar color, int width);
  void receiveCallback(const sensor_msgs::ImageConstPtr& msg);
  void redFilter(const Mat src, cv::Mat &dst);
  void Preprocessing(const Mat src, cv::Mat &dst);
  void EdgeDetection(Mat &edgeImg, cv::Mat &dst);
  bool FindContour(Mat &edgeImg, vector<Point2f> &contour);
  bool CalcTransform(const vector<Point2f> contour, cv::Vec3d &magRvec, cv::Vec3d &magTvec);
  bool Publish(const bool detected, const cv::Vec3d magRvec, const cv::Vec3d magTvec);
  void DrawRotatedRect(cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness);
  
  // void ActionCB(const mag_track::MagTrackGoal::ConstPtr &data);

private:
  void Init();
  ros::NodeHandle nh_; 
  ros::Publisher resultsPub_;
  ros::Publisher detectedPub_;
  image_transport::Subscriber sub_;
  image_transport::ImageTransport it_;
  // actionlib::SimpleActionServer<mag_track::MagTrackAction> as_;
  // actionlib::SimpleActionClient<mag_track::LookAndMoveAction> ac_lnm;
  
  
  Arguments args;
  cv::Mat dist_coeffs;
  cv::Mat camera_matrix;
  string sub_topic, detected_topic, results_topic;
  bool enable_debug_;
  
  
  cv::Mat srcImg;
  cv::Mat visImg;
  cv::Mat grayImg;
  std::vector<cv::Point3f> modelPoints;
  
  int params[10];  
  std::vector<cv::RotatedRect> rects;

  bool running_;

};


