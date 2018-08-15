#include "mag_track_node.h"
#include <ros/package.h>

using namespace std;
using namespace cv;
using namespace mag_track;

bool comp (const cv::RotatedRect &a, const cv::RotatedRect &b) 
{
    return (a.size.area() > b.size.area()); 
}

MagTrack::MagTrack():it_(nh_)//,
                    // as_(nh_, "mag_track_action", boost::bind(&MagTrack::ActionCB, this, _1), false)
{
    loadArguments();
    Init();
    sub_ = it_.subscribe(sub_topic, 1, &MagTrack::receiveCallback, this);
    resultsPub_ = nh_.advertise<geometry_msgs::PoseStamped>("visual_pose", 1);
   
    //as_.start();

    //std::cerr << "waiting for goal!" << endl;
    running_ = true;
    
}

/*
void MagTrack::ActionCB(const MagTrackGoal::ConstPtr &data)
{
    switch (data->command) {
      case 1:
        running_ = true;
        as_.setSucceeded();
        std::cerr << "launched! waiting for images!" << endl;
        break;
      case 2:
        running_ = false;
        as_.setSucceeded();
        std::cerr << "stoped! waiting for goal!" << endl;
        break;
      default:
        break;
    }
}
*/

void MagTrack::Init()
{
    dist_coeffs = (cv::Mat_<double>(4,1) << 
        args.camera().camera_distortion().data(0), 
        args.camera().camera_distortion().data(1), 
        args.camera().camera_distortion().data(2), 
        args.camera().camera_distortion().data(3)); 
    camera_matrix = (cv::Mat_<double>(3,3) << 
        args.camera().camera_matrix().data(0), 
        args.camera().camera_matrix().data(1), 
        args.camera().camera_matrix().data(2), 
        args.camera().camera_matrix().data(3),
        args.camera().camera_matrix().data(4), 
        args.camera().camera_matrix().data(5),
        args.camera().camera_matrix().data(6), 
        args.camera().camera_matrix().data(7),
        args.camera().camera_matrix().data(8)); 

    sub_topic = args.subscribe_topic();
     
    modelPoints.push_back(cv::Point3f(-0.5 * args.box_width(), 0.5 * args.box_height(), 96.3));           
    modelPoints.push_back(cv::Point3f(0.5 * args.box_width(), 0.5 * args.box_height(), 96.3));       
    modelPoints.push_back(cv::Point3f(0.5 * args.box_width(), -0.5 * args.box_height(), 96.3));     
    modelPoints.push_back(cv::Point3f(-0.5 * args.box_width(), -0.5 * args.box_height(), 96.3)); 
    
    cout << "I am " << args.iam() << " infantry." << endl;

    if(args.iam() == "RED")
    {
        params[0] = args.thresholds_red().data(0);
        params[1] = args.thresholds_red().data(1);
        params[2] = args.thresholds_red().data(2);
        params[3] = args.thresholds_red().data(3);
        params[4] = args.thresholds_red().data(4);
        params[5] = args.thresholds_red().data(5);
    }else
    {
        params[0] = args.thresholds_blue().data(0);
        params[1] = args.thresholds_blue().data(1);
        params[2] = args.thresholds_blue().data(2);
        params[3] = args.thresholds_blue().data(3);
        params[4] = args.thresholds_blue().data(4);
        params[5] = args.thresholds_blue().data(5);
    }

    enable_debug_ = args.enable_debug();
    rect_area_threshold = args.rect_area_threshold();
    area_ratio_threshold = args.area_ratio_threshold();
}


MagTrack::~MagTrack()
{
}

bool MagTrack::loadArguments() 
{
      //string path = ros::package::getPath("ammo_visual_detect") + "/proto/arguments.prototxt";
      string path = ros::package::getPath("roborts") + "/rmsc/ammo_visual_detect/proto/arguments.prototxt";
      std::ifstream f1(path.c_str());
      std::string input((std::istreambuf_iterator<char>(f1)), std::istreambuf_iterator<char>());
      bool success = google::protobuf::TextFormat::ParseFromString(input, &args);
      if (success) 
      { 
          cout << "data prepared." << endl;
          return success;
      }else
      {
          cout << "data loading failed." << endl;
          return success;
      }
}

void MagTrack::DrawRotatedRect(cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness) 
{
    cv::Point2f vertex[4];
    rect.points(vertex);
    for (int i = 0; i < 4; i++)
       cv::line(img, vertex[i], vertex[(i + 1) % 4], color, thickness);
}


void MagTrack::Preprocessing(const Mat src, Mat &dst)
{
    cv::Mat yuvImg, thresImg;
    cv::cvtColor(srcImg, yuvImg, cv::COLOR_BGR2YUV); 

    cv::Mat thres_lower(cv::Scalar(params[0], params[2], params[4]));
    cv::Mat thres_higher(cv::Scalar(params[1], params[3], params[5]));
    cv::inRange(yuvImg, thres_lower, thres_higher, thresImg);

    if(enable_debug_)
    {
      cv::imshow("thresImg", thresImg);
    }
    
    cv::Mat element = getStructuringElement(cv::MORPH_RECT, 
                                            Size(7,7), 
                                            Point(4, 4));
    cv::Mat erosionImg, dilationImg;
    cv::dilate(thresImg, dilationImg, element);
    cv::erode(dilationImg, erosionImg, element);
   
    if(enable_debug_)
    {
      cv::imshow("erosionImg", erosionImg);
    }
    
    dst = erosionImg.clone();
}


void MagTrack::EdgeDetection(Mat &processedImg, cv::Mat &dst)
{
    cv::Mat dilationImg;
    cv::Mat smallElement = getStructuringElement(cv::MORPH_RECT, Size(3, 3), Point(1, 1));
    cv::dilate(processedImg, dilationImg, smallElement);
    dst = dilationImg - processedImg;

    if(enable_debug_)
    {
      cv::imshow("edge", dst);
    }
}

bool MagTrack::FindContour(Mat &edgeImg, vector<Point2f> &contour)
{
    std::vector<std::vector<cv::Point> > contours;
    std::vector<Vec4i> hierarchy;
    cv::findContours(edgeImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE) ;
    std::vector<cv::RotatedRect> rect_candidates;

    for (unsigned int i = 0; i < contours.size(); ++i) 
    {
        cv::RotatedRect rect = cv::minAreaRect(contours[i]);   
        //if (rect.size.area() > 2000){
        //cout << "ratio of area : " << contourArea(contours[i]) / rect.size.area() << endl;
        //cout << "rect area : " << rect.size.area() << endl;  
        //cout << "rect.angle : " << rect.angle << endl;  
        //cout << "ratio2 : " << rect.size.height / rect.size.width << endl;
        //cout << "---" << endl;}
        if (rect.size.area() > rect_area_threshold         &&
            (contourArea(contours[i]) / rect.size.area()) < area_ratio_threshold &&
            rect.angle < -35                          &&
            rect.angle > -55                          &&
            rect.size.height / rect.size.width < 2.15 &&
            rect.size.height / rect.size.width > 1.85        
           )
        { 
           rect_candidates.push_back(rect);
        }
    }

    if (rect_candidates.size()  == 0)
        return false;

    std::sort (rect_candidates.begin(), rect_candidates.end(), comp);

    cv::RotatedRect biggest_rect = rect_candidates[0];
    float width = std::min(biggest_rect.size.width, biggest_rect.size.height);
    float height = std::max(biggest_rect.size.width, biggest_rect.size.height);
    float angle = 45 - biggest_rect.angle; 
    float diameter = height;
    float radius = 0.5 * diameter;
           
    Point2f vertices[4];
    biggest_rect.points(vertices);        
    cv::Point2f anchor( 0.5*(vertices[0].x + vertices[1].x), 
                        0.5*(vertices[0].y + vertices[1].y));
    cv::RotatedRect square = RotatedRect(anchor, Size2f(diameter, diameter), -angle);
          
    if(enable_debug_)
    {
        DrawRotatedRect(visImg, biggest_rect, cv::Scalar(100, 100, 100), 1);
        DrawRotatedRect(visImg, square, cv::Scalar(0, 255, 0), 2);
    }
           
    square.points(vertices);
    contour.push_back(cv::Point2f(vertices[0].x,vertices[0].y));
    contour.push_back(cv::Point2f(vertices[1].x,vertices[1].y));
    contour.push_back(cv::Point2f(vertices[2].x,vertices[2].y));
    contour.push_back(cv::Point2f(vertices[3].x,vertices[3].y));
    return true;
}

bool MagTrack::CalcTransform(const vector<Point2f> contour, cv::Vec3d &magRvec, cv::Vec3d &magTvec)
{
    if (contour.size()==4)
    {   
        cv::solvePnP(modelPoints, contour, camera_matrix, dist_coeffs, magRvec, magTvec);
        return true;
    }else
    {
        return false;
    }
}

bool MagTrack::Publish(const bool detected, const cv::Vec3d magRvec, const cv::Vec3d magTvec)
{
    geometry_msgs::PoseStamped results_msg;
	if (detected)
	{
        if(enable_debug_)
        {
		    cout << "Rotation Vector : " << endl << magRvec / 3.14 * 180  << endl << endl;
		    cout << "Position Vector : " << endl << magTvec << endl << endl;
        }
        results_msg.header.frame_id = "uvc_camera";
        results_msg.header.stamp = ros::Time::now();
		results_msg.pose.position.x = magTvec[0];
        results_msg.pose.position.y = magTvec[1];
        results_msg.pose.position.z = magTvec[2];
        results_msg.pose.orientation.x = magRvec[0];
        results_msg.pose.orientation.y = magRvec[1];
        results_msg.pose.orientation.z = magRvec[2];
        results_msg.pose.orientation.w = 1.0;
        resultsPub_.publish(results_msg);
	}
	else
	{
        if(enable_debug_)
        {
		    cout << "Not Detected " << endl << magRvec << endl << endl;
        }
        results_msg.header.frame_id = "uvc_camera";
        results_msg.header.stamp = ros::Time::now();
		results_msg.pose.position.x = 0;
        results_msg.pose.position.y = 0;
        results_msg.pose.position.z = 0;
        results_msg.pose.orientation.x = 0;
        results_msg.pose.orientation.y = 0;
        results_msg.pose.orientation.z = 0;
        results_msg.pose.orientation.w = -1.0;
        resultsPub_.publish(results_msg);
	}    
}

void MagTrack::receiveCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if(!running_) 
    {
	    return;
	}
    cv::Mat originImg;
    originImg = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    cv::undistort(originImg, srcImg, camera_matrix, dist_coeffs);

    if(enable_debug_)
    {
       visImg = srcImg.clone();
       cv::imshow("thresImg", srcImg);
       cv::createTrackbar("chn1_low", "thresImg", &params[0], 255);
       cv::createTrackbar("chn1_high", "thresImg", &params[1], 255);
       cv::createTrackbar("chn2_low", "thresImg", &params[2], 255);
       cv::createTrackbar("chn2_high", "thresImg", &params[3], 255);
       cv::createTrackbar("chn3_low", "thresImg", &params[4], 255);
       cv::createTrackbar("chn3_high", "thresImg", &params[5], 255);
    }
    
    cv::Mat processedImg;
    Preprocessing(srcImg, processedImg);
    
    cv::Mat edgeImg;
    EdgeDetection(processedImg, edgeImg);
    
    vector<Point2f> contour;
    cv::Vec3d magRvec, magTvec;
    FindContour(edgeImg, contour);
    if(CalcTransform(contour, magRvec, magTvec))
        Publish(true, magRvec, magTvec);
    else
        Publish(false, magRvec, magTvec);

    if(enable_debug_)
    {
       cv::imshow("visImg", visImg);
       cv::waitKey(5);
       std::cout << "----------" << endl;
    }
   
}
  
int main(int argc, char **argv)
{
  cout << argv[0];
  ros::init(argc, argv, "mag_track_node");
  MagTrack magTrack;
  ros::spin();

  return 0;
}


