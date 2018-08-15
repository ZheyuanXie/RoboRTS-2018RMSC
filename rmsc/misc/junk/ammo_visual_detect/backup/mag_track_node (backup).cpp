#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TransformStamped.h"
#include "time.h"
#include <iostream>
#include <fstream>
#include <streambuf>
#include <string>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>

#include "arguments.pb.h"
#include <google/protobuf/text_format.h>
//#include <opencv2/aruco/dictionary.hpp>

#define SHOWALL 1
#define SHOW 1

using namespace std;
using namespace cv;
using namespace mag_track;
 
cv::Mat dist_coeffs = (cv::Mat_<double>(4,1) << 0.005612, -0.048042, 0.004065, 0.005149, 0.0); 
cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 527.917811, 0.0, 326.678226, 0.0 ,494.424655, 217.938651, 0, 0, 1);

class MagTrack
{
public:
  MagTrack():it_(nh_)
  {
    //Topic you want to publish
    resultsPub_ = nh_.advertise<geometry_msgs::TransformStamped>("/rmsc4/mag", 1);
    detectedPub_ = nh_.advertise<std_msgs::String>("/rmsc4/detected", 1);
    
    //Topic you want to subscribe
    sub_ = it_.subscribe("/camera_0", 1, &MagTrack::receiveCallback, this);
    
    templ1 = cv::imread("src/mag_track/templ1_82_70.png", 0);
    templ2 = cv::imread("src/mag_track/templ2_82_70.png", 0);
    
    results_msg.child_frame_id = "";
    results_msg.transform.translation.x = 0;
    results_msg.transform.translation.y = 0;
    results_msg.transform.translation.z = 0;
    results_msg.transform.rotation.x = 0;
    results_msg.transform.rotation.y = 0;
    results_msg.transform.rotation.z = 0;
    results_msg.transform.rotation.w = 0;
    loadArguments();
  }

  bool loadArguments() {
  
      std::ifstream t("src/proto/arguments.prototxt");
      std::string input((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
      bool success = google::protobuf::TextFormat::ParseFromString(input, &arguments_info);
      cout << success;
      return success;
  }
  void drawLines(cv::Mat &img, vector<Point> contour, cv::Scalar color, int width)
  {
     for (int i=0; i < contour.size(); i++)
     {
        cv::line(img, contour[i], contour[(i + 1) % contour.size()], color, width, 8);
     }
   }

  void grayStratch(Mat& src, int low, int high)
  {
      Mat_<unsigned char>::iterator begin,end;
 
      begin = src.begin<unsigned char>();
      end = src.end<unsigned char>();

      while(begin != end)
      {
        if ((*begin) < low) 
        {
             (*begin) = 0;
        }
        else
        {
            if ((*begin) > high) 
            {  
                (*begin) = 255;
            }else
            {
                (*begin)= (unsigned char)((*begin) - low) * 255 / ( high - low);
            }
        }
        begin++;
	  }
  }
 
   void verticalFilter(Mat& src, int thres)
   {
      Mat_<unsigned char>::iterator begin,end;
      cv::Mat dst = src.clone();
      begin = src.begin<unsigned char>() + 2*src.cols;
      end = src.end<unsigned char>() - 2*src.cols;

      while(begin != end)
      {
        if ((*(begin - 2 * src.cols) - (*(begin + 2 * src.cols)) < 15) && ((*(begin - 2 * src.cols) -(*begin)) > thres))
        {
             (*begin) = *(begin - 2 * src.cols);
             (*(begin - 1)) = *(begin - 2 * src.cols);
             (*(begin + 1)) = *(begin - 2 * src.cols);
        }
        begin++;
       }
    }

  void receiveCallback(const sensor_msgs::ImageConstPtr& msg)
  {
      
	  bool detected = false;
      cv::Mat srcImg = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
      cv::Mat undistImg = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
   //   cv::undistort(srcImg, undistImg, camera_matrix, dist_coeffs);
  
  /*
  //*****ARUCO*******
  vector< int > markerIds; 
  vector< vector<Point2f> > markerCorners, rejectedCandidates; 
  cv::Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();;
  cv::Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);
  cv::aruco::detectMarkers(undistImg, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
  float smallLength = 38.1;
  float bigLength = 67.4;
  float markerLength = smallLength;
  if (markerIds.size() > 0)
  {
     cv::aruco::drawDetectedMarkers(undistImg, markerCorners, markerIds);
     std::vector< cv::Vec3d > rvecs, tvecs;     
     cv::aruco::estimatePoseSingleMarkers(markerCorners, markerLength, camera_matrix, dist_coeffs, rvecs, tvecs);
     cout <<"rvecs.size()" << rvecs.size() << endl;
     for(int j = 0; j < rvecs.size(); j++)
     {
		 cv::Mat rvec = (cv::Mat_<float>(3,1) << rvecs[j][0], rvecs[j][1], rvecs[j][2]);
		 cv::Mat tvec = (cv::Mat_<float>(3,1) << tvecs[j][0], tvecs[j][1], tvecs[j][2]);
		 cv::Mat rmat, markerRmat, markerTmat;
         //cout << "rvecs_marker" << rvecs[j] << endl << "tvecs_marker" << tvecs[j] << endl ; 
         cv::aruco::drawAxis(undistImg, camera_matrix, dist_coeffs, rvecs[j], tvecs[j], markerLength * 0.5f);
         cv::Rodrigues(rvec, rmat);
         markerRmat = rmat.t(); 
         markerTmat = rmat * (- tvec);
         cout << "via Marker" << rvec << endl << tvec<< endl << endl;
         
         cv::Mat tempRv, tempTv, tempRmat;
         std::vector<cv::Point3d> modelPoints2;
         for (int k=0; k<4 ;k++)
         {
	         circle( undistImg, markerCorners[j][k], (k + 1) , Scalar(200,200,250), -1, 8, 0 );
		 }
         modelPoints2.push_back(cv::Point3d(-markerLength/2, markerLength/2, 0));           
         modelPoints2.push_back(cv::Point3d(markerLength/2, markerLength/2, 0));       
         modelPoints2.push_back(cv::Point3d(markerLength/2, -markerLength/2, 0));     
         modelPoints2.push_back(cv::Point3d(-markerLength/2, -markerLength/2, 0)); 
         cv::solvePnP(modelPoints2, markerCorners[j], camera_matrix, dist_coeffs, tempRv, tempTv);
         cv::Rodrigues(tempRv, tempRmat);
         markerRmat = tempRmat.t(); 
         markerTmat = tempRmat * (- tempTv);
         cout << "via my Marker" << tempRv << endl << tempTv<< endl << endl;
     }
     
  }
  */
  //*********************
          
      cv::Mat grayImg;
      cv::cvtColor(undistImg, grayImg, cv::COLOR_BGR2GRAY); 
      
      vector<Mat> channels;
      cv::split(undistImg, channels);
#if SHOW
      imshow("channels[2]", channels[2]);
#endif     

/*         
      cv::Mat hsvImg;
      cv::cvtColor(undistImg, hsvImg, cv::COLOR_BGR2HSV); 
      vector<Mat> hsvChannels;
      cv::split(hsvImg, hsvChannels);
      
      normalize(hsvChanneltf2_msgs/TFMessages[0], hsvChannels[0], 0, 255, CV_MINMAX);
#if SHOW
      imshow("cropped_h", hsvChannels[0]);
      imshow("cropped_s", hsvChannels[1]);
      imshow("cropped_v", hsvChannels[2]);
#endif
*/

/*         
      cv::Mat ycrcbImg;
      cv::cvtColor(undistImg, ycrcbImg, CV_BGR2YCrCb); 
      vector<Mat> ycrcbChannels;
      cv::split(ycrcbImg, ycrcbChannels);
      
      normalize(ycrcbChannels[0], ycrcbChannels[0], 0, 255, CV_MINMAX);
#if SHOW
      imshow("Y", ycrcbChannels[0]);
      imshow("Cr", ycrcbChannels[1]);
      imshow("Cb", ycrcbChannels[2]);
#endif
*/

/*
      cv::Mat imageLog(grayImg.size(), CV_32FC1);
      for (int i = 0; i < grayImg.rows; i++)
      {
          for (int j = 0; j < grayImg.cols; j++)
          {
              imageLog.at<float>(i, j)= log(1 + 0.02*grayImg.at<uchar>(i, j));
          }
      }
      normalize(imageLog, imageLog, 0, 255, CV_MINMAX);
      convertScaleAbs(imageLog, imageLog);

  #if SHOWALL
      cv::imshow("imageLog", imageLog);
  #endif
    

      
     cv::Mat rangeImg;
     threshold( grayImg, rangeImg, 100, 255, 3);
     threshold( rangeImg, rangeImg, 230, 255, 4);
#if SHOWALL
     cv::imshow("rangeImg", rangeImg);
#endif   
*/

     cv::Mat stretchImg = grayImg.clone();
     grayStratch(stretchImg, 120, 210);
#if SHOWALL
     cv::imshow("stretchImg", stretchImg);
#endif   

     //cv::Mat verticalImg = stretchImg.clone();
     //verticalFilter(verticalImg, 32);
#if SHOWALL
     //cv::imshow("verticalImg", verticalImg);
#endif   
     
      cv::Mat medianImg; 
      cv::medianBlur(stretchImg, medianImg, 7);

  #if SHOWALL
      cv::imshow("medianImg", medianImg);
  #endif

      cv::Mat gaussianImg;
      cv::GaussianBlur(medianImg, gaussianImg, Size(3,5), 0, 13);
      
  #if SHOWALL
      cv::imshow("gaussianImg", gaussianImg);
  #endif
  
/*  
      start = clock();
      **********
     finish = clock();
      double duration = (double)(finish - start) / CLOCKS_PER_SEC;
      printf( "this operation durates %f seconds\n", duration );

*/   
     
      cv::Mat cannyImg;
      int thresh = 10;
      cv::Canny(gaussianImg, cannyImg, thresh, thresh*3.5, 3);
      
  #if SHOWALL
      cv::imshow("cannyImg", cannyImg);
  #endif
  
  
      start = clock();
      cv::Mat newCannyImg = cannyImg.clone();
      std::vector<std::vector<cv::Point2i> > primeContours ;
      vector<Vec4i> tthierarchy;
      cv::findContours(newCannyImg, primeContours, tthierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
      
      for (int i = 0; i < primeContours.size(); i++)
      {
		  
		  //cout << "primeContours[i].size()" << primeContours[i].size() << endl;
          if (primeContours[i].size() < 30)
          {
			int minx = 500;
			int miny = 500;
			int maxx = -1;
			int maxy = -1;
			for (int j = 0; j < primeContours[i].size(); j++)
            {
			    minx = primeContours[i][j].x < minx? primeContours[i][j].x: minx;
			    miny = primeContours[i][j].y < miny? primeContours[i][j].y: miny;
			    maxx = primeContours[i][j].x > maxx? primeContours[i][j].x: maxx;
			    maxy = primeContours[i][j].y > maxy? primeContours[i][j].y: maxy;
		    }
		    if (sqrt((maxx - minx)*(maxx - minx) + (maxy - miny)*(maxy - miny)) >20) continue; 
			//cv::approxPolyDP(primeContours[i], primeCurve, 2, true);
			//if (primeCurve.size() == 2) continue;
            for (int j = 0; j < primeContours[i].size(); j++)
            {
			    newCannyImg.at<uchar>(primeContours[i][j].y, primeContours[i][j].x) = 0;
		    }
	      }
	  }
	  finish = clock();
      double duration = (double)(finish - start) / CLOCKS_PER_SEC;
      printf( "this operation durates %f seconds\n", duration );

  #if SHOWALL
      cv::imshow("newCannyImg", newCannyImg);
  #endif 
       
      cv::Mat element = getStructuringElement(cv::MORPH_RECT, Size(3, 3), Point(1, 1));
      cv::Mat erosionImg, dilationImg;
      cv::dilate(newCannyImg, dilationImg, element);
      cv::erode(dilationImg, erosionImg, element);
   
  #if SHOWALL
      cv::imshow("erosionImg", erosionImg);
  #endif
  
      cv::Mat smallElement = getStructuringElement(cv::MORPH_RECT, Size(3, 3), Point(1, 1));
      cv::dilate(erosionImg, dilationImg, smallElement);
      cv::Mat edge = dilationImg - erosionImg;

  #if SHOWALL
      cv::imshow("edge", edge);
  #endif


      std::vector<std::vector<cv::Point> > contours ;
      vector<Vec4i> hierarchy;
      cv::findContours(edge, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE) ;
  #if SHOWALL
      cv::imshow("grayImg", grayImg);
  #endif
     

      cv::Vec3d accurateRv;   // for output
      cv::Vec3d accurateTv;   // for output
      cv::Mat accurateRmat;   // for output
      cv::Mat magRmat;     
      cv::Mat magTvec;
      
      vector<Point> approxCurve;
      for (int i = 0; i < contours.size(); i++)
      {
      
          cv::approxPolyDP(contours[i], approxCurve, 15, true);
          if (approxCurve.size() !=4) continue;
          if (!cv::isContourConvex(approxCurve)) continue;
          float minDist = 100;
          for (int j = 0; j < 4; j++)
          {
              float d = cv::norm(approxCurve[j]-approxCurve[(j + 1) % 4] );
              if (d < minDist) minDist = d;
          }
          if (minDist < 50) continue;

        double dx1 = approxCurve[1].x - approxCurve[0].x;
        double dy1 = approxCurve[1].y - approxCurve[0].y;
        double dx2 = approxCurve[2].x - approxCurve[0].x;
        double dy2 = approxCurve[2].y - approxCurve[0].y;
        double o = (dx1 * dy2) - (dy1 * dx2);
        if (o < 0.0)
        {  
            std::swap(approxCurve[1], approxCurve[3]);
        }
        drawLines(undistImg, approxCurve, cv::Scalar(220, 220, 220), 2);

        std::vector<cv::Point2d> imagePoints;
        if (approxCurve[0].y < approxCurve[2].y)
        {
            if (approxCurve[0].x < approxCurve[2].x)
            {
                imagePoints.push_back( cv::Point2d(approxCurve[0].x, approxCurve[0].y)); 
                imagePoints.push_back( cv::Point2d(approxCurve[1].x, approxCurve[1].y));  
                imagePoints.push_back( cv::Point2d(approxCurve[2].x, approxCurve[2].y));  
                imagePoints.push_back( cv::Point2d(approxCurve[3].x, approxCurve[3].y));  
            }
            else
            {
                imagePoints.push_back( cv::Point2d(approxCurve[3].x, approxCurve[3].y)); 
                imagePoints.push_back( cv::Point2d(approxCurve[0].x, approxCurve[0].y) );  
                imagePoints.push_back( cv::Point2d(approxCurve[1].x, approxCurve[1].y) );  
                imagePoints.push_back( cv::Point2d(approxCurve[2].x, approxCurve[2].y) );  
            }
        }
        else
        { 
            if (approxCurve[0].x < approxCurve[2].x)
            {
                imagePoints.push_back( cv::Point2d(approxCurve[1].x, approxCurve[1].y)); 
                imagePoints.push_back( cv::Point2d(approxCurve[2].x, approxCurve[2].y) );  
                imagePoints.push_back( cv::Point2d(approxCurve[3].x, approxCurve[3].y) );  
                imagePoints.push_back( cv::Point2d(approxCurve[0].x, approxCurve[0].y) );  
            }
            else
            {
                imagePoints.push_back( cv::Point2d(approxCurve[2].x, approxCurve[2].y)); 
                imagePoints.push_back( cv::Point2d(approxCurve[3].x, approxCurve[3].y) );  
                imagePoints.push_back( cv::Point2d(approxCurve[0].x, approxCurve[0].y) );  
                imagePoints.push_back( cv::Point2d(approxCurve[1].x, approxCurve[1].y) );  
            }
        }
    
    // 3D model points.
        std::vector<cv::Point3d> modelPoints;
        modelPoints.push_back(cv::Point3d(-82.5, 71.5, 96.3));           
        modelPoints.push_back(cv::Point3d(82.5, 71.5, 96.3));       
        modelPoints.push_back(cv::Point3d(82.5, -71.5, 96.3));     
        modelPoints.push_back(cv::Point3d(-82.5, -71.5, 96.3)); 
    
        cv::Vec3d rv;
        cv::Vec3d tv;
        cv::solvePnP(modelPoints, imagePoints, camera_matrix, dist_coeffs, rv, tv);
      
        Point2f pointsRes[4], pointsIn[4];    
        for (int j = 0; j < 4; j++)
        {
             pointsIn[j].x = static_cast<float>(imagePoints[j].x);
             pointsIn[j].y = static_cast<float>(imagePoints[j].y);
	   //      circle( undistImg, imagePoints[j], (j + 1) , Scalar(200,200,250), -1, 8, 0 );
        }
        cv::Size templateSize(82, 70);
        pointsRes[0] = Point2f(0, 0);
        pointsRes[1] = Point2f(static_cast<float>(templateSize.width - 1), 0.f);
        pointsRes[2] = Point2f(static_cast<float>(templateSize.width - 1), static_cast<float>(templateSize.height - 1));
        pointsRes[3] = Point2f(0.f, static_cast<float>(templateSize.height - 1));
        cv::Mat M = getPerspectiveTransform(pointsIn, pointsRes);
        cv::Mat matchingImg;
        cv::warpPerspective(grayImg, matchingImg, M, templateSize, cv::INTER_LINEAR);
#if SHOW
        cv::imshow("matchingImg", matchingImg);
#endif
        /*
        cv::Mat cannyImg2;
        int thresh2 = 25;
        cv::Canny(matchingImg, cannyImg2, thresh2, thresh2*2, 3);
        cv::imshow("test", cannyImg2);
        */
        
        /*
        vector<Vec3f> circles;
        HoughCircles( matchingImg, circles, CV_HOUGH_GRADIENT, 1, matchingImg.rows/8, 40, 70);

        if (circles.size() != 1) continue;
        if (circles[0][0] < 78 || circles[0][0] > 89 || circles[0][1] < 69 || circles[0][1] > 77) 
            continue;
      
#if SHOW
        cv::Point center(cvRound(circles[0][0]), cvRound(circles[0][1]));
        int radius = cvRound(circles[0][2]); 
        //circle( matchingImg, center, 2, Scalar(250), -1, 8, 0 );
        //circle( matchingImg, center, radius, Scalar(250), 2, 8, 0 );
        std::cout << "circle:" << circles[0][0] << " " << circles[0][1] << " " << circles[0][2]<< endl;

        //drawLines(srcImg, approxCurve, cv::Scalar(100, 250, 100), 2);
        drawLines(undistImg, approxCurve, cv::Scalar(100, 250, 100), 2);
#endif  */        
        

        //cout << "my rvec: " << rv << endl << "my tvec:" << tv << endl;
     
       
       int result1_cols = matchingImg.cols - templ1.cols + 1;
       int result1_rows = matchingImg.rows - templ1.rows + 1;
       cv::Mat result1(result1_rows, result1_cols, CV_32FC1, Scalar(0));       
       
       cv::matchTemplate(matchingImg, templ1, result1, CV_TM_CCOEFF);
       //cv::normalize(result1, result1, 0, 2, NORM_MINMAX, -1, Mat());
       
       double minValue1, maxValue1;
       cv::Point minLocation1, maxLocation1;
       cv::minMaxLoc(result1, &minValue1, &maxValue1, &minLocation1, &maxLocation1);
       
       
       int result2_cols = matchingImg.cols - templ2.cols + 1;
       int result2_rows = matchingImg.rows - templ2.rows + 1;
       cv::Mat result2(result2_rows, result2_cols, CV_32FC1);
       
       cv::matchTemplate(matchingImg, templ2, result2, CV_TM_CCOEFF);
       //cv::normalize(result2, result2, 0, 2, NORM_MINMAX, -1, Mat());
       double minValue2, maxValue2;
       cv::Point minLocation2, maxLocation2;
       cv::minMaxLoc(result2, &minValue2, &maxValue2, &minLocation2, &maxLocation2);
       
       double maxValue;
       cv::Point maxLocation;
       if (maxValue2 > maxValue1)
       {
	       maxValue = maxValue2;
	       maxLocation = maxLocation2;
#if SHOW
	       cv::imshow("result", result2);
	       waitKey(5);
	       
#endif
           cout << "Matching: " <<maxValue << endl;
           if (maxValue < 750000) continue;
	    }
       else
       {	
		   maxValue = maxValue1;
	       maxLocation = maxLocation1;
#if SHOW
	       cv::imshow("result", result1);
	       waitKey(5);
#endif
            cout << "Matching: " <<maxValue << endl;
            if (maxValue < 1.1e6) continue;
       }
       drawLines(undistImg, approxCurve, cv::Scalar(100, 250, 100), 2);

        int roiSize = 16;
        std::vector<cv::Point2f> accuratePoints;
        std::vector<cv::Point2f> harrisCorners;
        for (int j=0; j < imagePoints.size(); j++)
        { 
			if ( imagePoints[j].x < 0.5*roiSize || imagePoints[j].x >= grayImg.cols - roiSize||
                 imagePoints[j].y < 0.5*roiSize || imagePoints[j].y >= grayImg.rows - roiSize)
			    continue;
            Rect cornerRegionRect = Rect(imagePoints[j].x - roiSize/2, imagePoints[j].y - roiSize/2, roiSize, roiSize);
            cv::goodFeaturesToTrack(grayImg(cornerRegionRect), harrisCorners, 1, 0.04, 10, cv::Mat(), 5, false, 0.06); 
            if (harrisCorners.size() > 0) 
            {
                harrisCorners[0].x = harrisCorners[0].x + imagePoints[j].x - roiSize/2;
                harrisCorners[0].y = harrisCorners[0].y + imagePoints[j].y - roiSize/2;
#if SHOWALL
                cout << "Harris corner: " << harrisCorners[0].x << "  " << harrisCorners[0].y  << endl;
                cv::circle(grayImg, harrisCorners[0], 1, cv::Scalar(255), 1, 8, 0);
#endif
#if SHOW
                cv::circle(undistImg, harrisCorners[0], 1, cv::Scalar(255), 1, 8, 0);
#endif
                cv::TermCriteria criteria = 
                cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 15, 0.001);
                cv::cornerSubPix(grayImg, harrisCorners, cv::Size(5, 5), cv::Size(-1, -1), criteria);
                cout << "Subpixel corner: " << harrisCorners[0].x << "  " << harrisCorners[0].y  << endl;
                accuratePoints.push_back(cv::Point2f(harrisCorners[0].x, harrisCorners[0].y) );
            }
            else
            {
            cout << "Error by harrisCorners !!" << endl;
            }
        }


        if(accuratePoints.size() == 4)
        {

            detected = true;
            cv::solvePnP(modelPoints, accuratePoints, camera_matrix, dist_coeffs, accurateRv, accurateTv);
            results_msg.child_frame_id = "detected!";
            results_msg.transform.translation.x = accurateTv[0];
            results_msg.transform.translation.y = accurateTv[1];
            results_msg.transform.translation.z = accurateTv[2];
            results_msg.transform.rotation.x = accurateRv[0];
            results_msg.transform.rotation.y = accurateRv[1];
            results_msg.transform.rotation.z = accurateRv[2];
            results_msg.transform.rotation.w = 0;
            cout << "Results" << endl << 
                  accurateRv << endl << "[" <<
                  accurateRv[0]/CV_PI*180 <<" , " << 
                  accurateRv[1]/CV_PI*180 <<" , " << 
                  accurateRv[2]/CV_PI*180 <<" ]"  << endl <<
                  accurateTv<< endl << endl;
            detected_msg.data = "detected!";
            break;
        }
        else
        {
            cout << "Error by Subpixel calculation !!" << endl;
            continue;
        }
      }

#if SHOWALL
      cv::imshow("grayImg", grayImg);
      cv::imshow("srcImg", srcImg);
      cv::imshow("undistImg", undistImg);
      cv::waitKey(5);
#endif
     
         
      if(detected)
      {
		  detectedPub_.publish(detected_msg);
	  }
	  else
      {
            results_msg.child_frame_id = "not detected!";
      }
      resultsPub_.publish(results_msg);
      
      std::cout << "-----------------------------------------" << endl;
  }

private:
  ros::NodeHandle nh_; 
  ros::Publisher resultsPub_;
  ros::Publisher detectedPub_;
  image_transport::Subscriber sub_;
  image_transport::ImageTransport it_;
  
  std_msgs::String detected_msg;
  geometry_msgs::TransformStamped results_msg;
  cv::Mat templ1, templ2;
  clock_t start, finish;
  
  Arguments arguments_info;
  CameraMatrix camera_matrix_info;
  CameraDistortion camera_distortion_info;
  Box box_info;
};

  
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mag_track");
  MagTrack magTrack;
  ros::spin();

  return 0;
}

