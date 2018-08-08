#include "mag_track_node.h"

#define SHOWALL 1
#define SHOW 1

using namespace std;
using namespace cv;
using namespace mag_track;
 
MagTrack::MagTrack():it_(nh_)
{
    loadArguments();
    Init();

    resultsPub_ = nh_.advertise<geometry_msgs::TransformStamped>(results_topic, 1);
    detectedPub_ = nh_.advertise<std_msgs::String>(detected_topic, 1);
    sub_ = it_.subscribe(sub_topic, 1, &MagTrack::receiveCallback, this);

    results_msg.child_frame_id = "";
    results_msg.transform.translation.x = 0;
    results_msg.transform.translation.y = 0;
    results_msg.transform.translation.z = 0;
    results_msg.transform.rotation.x = 0;
    results_msg.transform.rotation.y = 0;
    results_msg.transform.rotation.z = 0;
    results_msg.transform.rotation.w = 0;
}

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

     templ = cv::imread(args.templ1_directory(), 0);
     equalizeHist(templ, templ);
     
     sub_topic = args.subscribe_topic();
     detected_topic = args.detected_topic();
     results_topic = args.results_topic();
     
     modelPoints.push_back(cv::Point3f(-0.5 * args.box_width(), 0.5 * args.box_height(), 96.3));           
     modelPoints.push_back(cv::Point3f(0.5 * args.box_width(), 0.5 * args.box_height(), 96.3));       
     modelPoints.push_back(cv::Point3f(0.5 * args.box_width(), -0.5 * args.box_height(), 96.3));     
     modelPoints.push_back(cv::Point3f(-0.5 * args.box_width(), -0.5 * args.box_height(), 96.3)); 
     
    params[0] = 15;
    params[1] = 111;
    params[2] = 102;
    params[3] = 135;
    params[4] = 141;
    params[5] = 165;
}


MagTrack::~MagTrack()
{
}

bool MagTrack::loadArguments() 
{
      std::ifstream f1("src/proto/arguments.prototxt");
      std::string input((std::istreambuf_iterator<char>(f1)), std::istreambuf_iterator<char>());
      bool success = google::protobuf::TextFormat::ParseFromString(input, &args);
      if (success) 
        cout << "data prepared.";
      return success;
}

void MagTrack::drawLines(cv::Mat &img, vector<Point> contour, cv::Scalar color, int width)
{
     for (int i=0; i < contour.size(); i++)
     {
        cv::line(img, contour[i], contour[(i + 1) % contour.size()], color, width, 8);
     }
}

void MagTrack::DrawRotatedRect(cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness) 
{
    cv::Point2f vertex[4];
    rect.points(vertex);
    for (int i = 0; i < 4; i++)
       cv::line(img, vertex[i], vertex[(i + 1) % 4], color, thickness);
}

 

void MagTrack::verticalFilter(Mat src, Mat &dst, int thres)
{
      Mat_<unsigned char>::iterator begin,end, dstBegin;
      dst = src.clone();
      begin = src.begin<unsigned char>();
      end = src.end<unsigned char>();
      dstBegin = dst.begin<unsigned char>();

      for(int i = 3; i < src.rows - 3; i++ )
         for(int j = 1; j < src. cols - 1; j++)
         {
               int upAve, downAve, middleAve;
               upAve = (src.at<uchar>(i - 3,j - 1) + src.at<uchar>(i - 3,j) +src.at<uchar>(i - 3,j + 1) )/3;
               middleAve =  (src.at<uchar>(i,j - 1) + src.at<uchar>(i, j) +src.at<uchar>(i,j + 1) )/3;
               downAve  = (src.at<uchar>(i + 3,j - 1) + src.at<uchar>(i + 3,j) +src.at<uchar>(i + 3,j + 1) )/3;
               if ((abs(upAve - downAve) < 8) &&(abs(upAve - downAve) !=0) && (middleAve < (abs(upAve + downAve) / 2)))
               {
                     dst.at<uchar>(i,j) = (uchar)((upAve + downAve) /2);
               } 

         }
}

void MagTrack::Preprocessing(const Mat src, Mat &dst)
{
    cv::Mat hsvImg, img_red, img_threshold_red1, img_threshold_red2;
    cv::cvtColor(srcImg, hsvImg, cv::COLOR_BGR2YUV); 
    vector<Mat> hsvChannels;
    cv::split(hsvImg, hsvChannels);

    cv::Mat t1, t2, t3, t4;
    cv::inRange(hsvChannels[0], 5, 10, t1);
    cv::inRange(hsvChannels[0], 150, 180, t2);
    t2 = t1 | t2;
    cv::inRange(hsvChannels[1], 43, 255, t3);
    cv::inRange(hsvChannels[2], 46, 255, t4);
#if SHOWALL
    cv::imshow("h_origin", hsvChannels[0]);
    cv::imshow("h", hsvChannels[0]);
    cv::imshow("s", hsvChannels[1]);
    cv::imshow("v", hsvChannels[2]);
#endif
    cv::Mat red1_low(cv::Scalar(params[0], params[2], params[4]));
    cv::Mat red1_higher(cv::Scalar(params[1], params[3], params[5]));

    for (int j = 0; j < 6; j++)
    {
	    cout << params[j] << "  |  ";
	}
    //cv::Mat red2_low(cv::Scalar(params[0], 100, 145));
    //cv::Mat red2_higher(cv::Scalar(params[0], 135, 175));
    cv::inRange(hsvImg, red1_low, red1_higher, img_threshold_red1);
    //cv::inRange(hsvImg, red2_low, red2_higher, img_threshold_red2);
    img_red = img_threshold_red1 | img_threshold_red2;
#if SHOWALL
    cv::imshow("img_red", img_red);
#endif
    
	cv::Mat element = getStructuringElement(cv::MORPH_RECT, 
                                            Size(7,7), 
                                            Point(4, 4));
    cv::Mat erosionImg, dilationImg;
    cv::dilate(img_red, dilationImg, element);
    cv::erode(dilationImg, erosionImg, element);
#if SHOWALL
    cv::imshow("erosionImg", erosionImg);
#endif
    dst = erosionImg.clone();
    
}


void MagTrack::EdgeDetection(Mat &processedImg, cv::Mat &dst)
{
    cv::Mat dilationImg;
    cv::Mat smallElement = getStructuringElement(cv::MORPH_RECT, Size(3, 3), Point(1, 1));
    cv::dilate(processedImg, dilationImg, smallElement);
    cv::Mat edge = dilationImg - processedImg;
#if SHOWALL
    cv::imshow("edge", edge);
#endif
    dst = edge.clone();
}

bool MagTrack::FindContour(Mat &edgeImg, vector<Point2f> &contour)
{
    //bool detected = false;
    std::vector<std::vector<cv::Point> > contours ;
    std::vector<Vec4i> hierarchy;
    cv::findContours(edgeImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE) ;
    //for( int i=0; i < contours.size(); i ++)
    //   cv::drawContours(visImg, contours, (int)i, Scalar::all(155), 1, 8); 
    
    for (unsigned int i = 0; i < contours.size(); ++i) 
    {
       cv::RotatedRect rect = cv::minAreaRect(contours[i]); 
       cout << "area = " <<  rect.size.area() << endl;      
       if (rect.size.area() > 1000 &&
           rect.angle < -35        &&
           rect.angle > -55        &&
           rect.size.height / rect.size.width < 2.15 &&
           rect.size.height / rect.size.width > 1.85        
          )
       {		   
		   rects.push_back(rect);
		   DrawRotatedRect(visImg, rect, cv::Scalar(80, 80, 80), 1);
		   
		   float width = std::min(rect.size.width, rect.size.height);
           float height = std::max(rect.size.width, rect.size.height);
           float angle = 45 - rect.angle; 
           float diameter = height;
           float radius = 0.5 * diameter;
           
           Point2f vertices[4];
           rect.points(vertices);
           cout << "angle = " << angle << endl;
           cout << "ratio = " << rect.size.height / rect.size.width << endl;
           
           cv::Point2f anchor( 0.5*(vertices[0].x + vertices[1].x), 
                                0.5*(vertices[0].y + vertices[1].y));
           cv::RotatedRect square = RotatedRect(anchor, Size2f(diameter, diameter), -angle);
           DrawRotatedRect(visImg, square, cv::Scalar(0, 255, 0), 2);
           
           square.points(vertices);
           contour.push_back(cv::Point2f(vertices[0].x,vertices[0].y));
           contour.push_back(cv::Point2f(vertices[1].x,vertices[1].y));
           contour.push_back(cv::Point2f(vertices[2].x,vertices[2].y));
           contour.push_back(cv::Point2f(vertices[3].x,vertices[3].y));
           return true;
       }   
    }
    return false;
}

bool MagTrack::CalcTransform(const vector<Point2f> contour, cv::Vec3d &magRvec, cv::Vec3d &magTvec)
{
	cout << contour << modelPoints<< endl;
	 if (contour.size()==4)
        cv::solvePnP(modelPoints, contour, camera_matrix, dist_coeffs, magRvec, magTvec);
     cout << "Results" << endl << 
             magTvec<< endl << endl;
             return true;
}
  
void MagTrack::receiveCallback(const sensor_msgs::ImageConstPtr& msg)
{
    
    bool detected = false;
    cv::Mat originImg;
    originImg = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    cv::undistort(originImg, srcImg, camera_matrix, dist_coeffs);
    //cv::resize(originImg, srcImg, Size(originImg.cols, originImg.rows));
#if SHOWALL
    cv::imshow("img_red", originImg);

    cv::createTrackbar("chn1_low", "img_red", &params[0], 255);
    cv::createTrackbar("chn1_high", "img_red", &params[1], 255);
    cv::createTrackbar("chn2_low", "img_red", &params[2], 255);
    cv::createTrackbar("chn2_high", "img_red", &params[3], 255);
    cv::createTrackbar("chn3_low", "img_red", &params[4], 255);
    cv::createTrackbar("chn3_high", "img_red", &params[5], 255);
#endif
    visImg = srcImg.clone();
    
    cv::Mat processedImg;
    Preprocessing(srcImg, processedImg);
    
    cv::Mat edgeImg;
    EdgeDetection(processedImg, edgeImg);
    
    vector<Point2f> contour;
    cv::Vec3d magRvec, magTvec;
    if(FindContour(edgeImg, contour));
        CalcTransform(contour, magRvec, magTvec);

#if SHOW
   cv::imshow("visImg", visImg);
   cv::waitKey(5);
#endif
   std::cout << "-----------------------------------------" << endl;
}
  
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mag_track");
  MagTrack magTrack;
  ros::spin();

  return 0;
}


