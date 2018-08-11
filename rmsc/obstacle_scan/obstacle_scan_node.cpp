#include "obstacle_scan_node.h"
#include <string>
#include <ros/package.h>
#include "common/log.h"

using namespace std;
using namespace cv;
using namespace obstacle_scan;

int scale;
float disThres;
int voteThres;
int x_map_offset;
int y_map_offset;
bool enable_debug;
std::string topic;
int cnt = 1;
int obst_counter = 0;
int checkPoint = 0;

static void OnMouseAction(int event,int x,int y,int flags,void *ustc)
{	
	if(event==CV_EVENT_LBUTTONUP)
	{
		LOG_INFO<< "# obstacle " << cnt << " at checkPoint " << (int) checkPoint << endl 
            << "data: " << (y - y_map_offset)/(1.0*scale)<< endl 
            << "data: " << (x- x_map_offset)/(1.0*scale)
            <<endl;
        cnt ++;
	}
}


ObstacleScan::ObstacleScan():as_(nh_, "obstacle_scan_node_action", boost::bind(&ObstacleScan::ActionCB, this, _1), false)
{
    
    loadArguments();
    scale = args.scale();
    disThres = args.dis_thres();
    voteThres = 7;
    topic = args.pub_topic();
    x_map_offset = args.x_map_offset();
    y_map_offset = args.y_map_offset();
    enable_debug = args.enable_debug();

	scan_sub_ = nh_.subscribe( topic, 1, &ObstacleScan::scanCallback, this);
	flag = false;

    for(int i = 0; i < 35; i++)
        obst_exist[i] = -1;

    LOG_INFO << "I am " << args.iam() << " infantry." << endl;
    
    if (args.iam() == "RED")
    {
        obst_location[4][1] = cv::Point2f(args.red_location().data(0), args.red_location().data(1));
        obst_location[5][1] = cv::Point2f(args.red_location().data(2), args.red_location().data(3));
        obst_location[6][1] = cv::Point2f(args.red_location().data(4), args.red_location().data(5));

        obst_location[1][2] = cv::Point2f(args.red_location().data(6), args.red_location().data(7));
        obst_location[2][2] = cv::Point2f(args.red_location().data(8), args.red_location().data(9));
        obst_location[3][2] = cv::Point2f(args.red_location().data(10), args.red_location().data(11));

        obst_location[1][3] = cv::Point2f(args.red_location().data(12), args.red_location().data(13));
        obst_location[2][3] = cv::Point2f(args.red_location().data(14), args.red_location().data(15));
        obst_location[3][3] = cv::Point2f(args.red_location().data(16), args.red_location().data(17));

        obst_location[1][4] = cv::Point2f(args.red_location().data(18), args.red_location().data(19));
        obst_location[2][4] = cv::Point2f(args.red_location().data(20), args.red_location().data(21));
        obst_location[3][4] = cv::Point2f(args.red_location().data(22), args.red_location().data(23));
    }
    
    if (args.iam() == "BLUE")
    {
        obst_location[4][1] = cv::Point2f(args.blue_location().data(0), args.blue_location().data(1));
        obst_location[5][1] = cv::Point2f(args.blue_location().data(2), args.blue_location().data(3));
        obst_location[6][1] = cv::Point2f(args.blue_location().data(4), args.blue_location().data(5));

        obst_location[1][2] = cv::Point2f(args.blue_location().data(6), args.blue_location().data(7));
        obst_location[2][2] = cv::Point2f(args.blue_location().data(8), args.blue_location().data(9));
        obst_location[3][2] = cv::Point2f(args.blue_location().data(10), args.blue_location().data(11));

        obst_location[1][3] = cv::Point2f(args.blue_location().data(12), args.blue_location().data(13));
        obst_location[2][3] = cv::Point2f(args.blue_location().data(14), args.blue_location().data(15));
        obst_location[3][3] = cv::Point2f(args.blue_location().data(16), args.blue_location().data(17));

        obst_location[1][4] = cv::Point2f(args.blue_location().data(18), args.blue_location().data(19));
        obst_location[2][4] = cv::Point2f(args.blue_location().data(20), args.blue_location().data(21));
        obst_location[3][4] = cv::Point2f(args.blue_location().data(22), args.blue_location().data(23));
    }
    LOG_WARNING << "waiting for goal!" << endl;
    as_.start(); 
}

ObstacleScan::~ObstacleScan()
{    
}

void ObstacleScan::ActionCB(const rmsc_messages::ObstacleScanGoal::ConstPtr &data)
{
    switch (data->checkpoint) {
      case 0:
        checkPoint = data->checkpoint;
        as_.setSucceeded();
        LOG_WARNING << "Stop checking." << endl;
        break;
      case 1:
      case 2:
      case 3:
      case 4:
        checkPoint = data->checkpoint;
        as_.setSucceeded();
        LOG_WARNING << "Checking at check point " << checkPoint << "." << endl;
        break;
      default: 
        LOG_INFO << "The check point is not defined." << endl;
        break;
    }

}


bool ObstacleScan::loadArguments() 
{
      string path = ros::package::getPath("roborts") + "/rmsc/obstacle_scan/proto/arguments.prototxt";
      std::ifstream f1(path.c_str());
      std::string input((std::istreambuf_iterator<char>(f1)), std::istreambuf_iterator<char>());
      bool success = google::protobuf::TextFormat::ParseFromString(input, &args);
      if (success)     
      { 
          LOG_INFO << "data prepared." << endl;
          return success;
      }else
      {
          LOG_INFO << "data loading failed." << endl;
          return success;
      }
}

void ObstacleScan::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    if(checkPoint == 0)  
    { 
        return;
    }
    sensor_msgs::PointCloud2 rosCloud;
    projector_.projectLaser(*scan, rosCloud);

    int size = rosCloud.width;
    int pointStep = rosCloud.point_step;
    
    vector<cv::Point2f> pointList;
    for (int i=0; i<size; i++)
    {
        float tX = 0.0;
        float tY = 0.0;
        int arrayPosX;
        int arrayPosY;
        arrayPosX =i*pointStep + rosCloud.fields[0].offset;
        arrayPosY =i*pointStep + rosCloud.fields[1].offset;
        memcpy(&tX, &rosCloud.data[arrayPosX], sizeof(float));
        memcpy(&tY, &rosCloud.data[arrayPosY], sizeof(float));
        pointList.push_back(Point2f(tX, tY));
    }  
 
    int obst_vote[35] = {0};

    cv::Mat pointsMap = cv::Mat::zeros(1000, 1000, CV_8UC3);
   
    if(enable_debug){
      cv::Rect infantry(x_map_offset-30, y_map_offset-40, 60, 80);
      cv::rectangle(pointsMap, infantry, Scalar(50, 50, 120), 3);
      cv::line(pointsMap, Point(x_map_offset-30, y_map_offset-40), Point(x_map_offset+30, y_map_offset+40), Scalar(50, 50, 120), 3);
      cv::line(pointsMap, Point(x_map_offset+30, y_map_offset-40), Point(x_map_offset-30, y_map_offset+40), Scalar(50, 50, 120), 3);
    }

    vector<int> cl; // = checklist;
    switch(checkPoint){
        case 1: 
             cl.push_back(6);
             cl.push_back(4);
             break;
        case 2:
             cl.push_back(3);
             cl.push_back(2);
             break;
        case 3: 
             cl.push_back(3);
             break;
        case 4:
             cl.push_back(1);
             break;
        default: 
             LOG_INFO << "The check point is not defined." << endl;
             break;
    }
    
    for(int i=0; i<pointList.size(); i++)
    {
        
        Point2i tempPoint(scale*pointList[i].y + x_map_offset, scale*pointList[i].x + y_map_offset);
        if (tempPoint.x < 0 || tempPoint.x > pointsMap.cols ||tempPoint.y < 0 || tempPoint.y > pointsMap.rows)
             continue;
        circle (pointsMap, tempPoint, 1, Scalar(255,255,255));

        for(int j =0; j < cl.size(); j++)
          if (calcDistance(pointList[i], obst_location[cl[j]][checkPoint]) < disThres)
          {
              obst_vote[cl[j]]++;
              circle (pointsMap, tempPoint, 1, Scalar(105,255,105));
          }
    }
    
    for(int j =0; j < cl.size(); j++)
    {
        Point2i tempPoint(scale*obst_location[cl[j]][checkPoint].y + x_map_offset,
                          scale*obst_location[cl[j]][checkPoint].x + y_map_offset);
        if (tempPoint.x < 0 || tempPoint.x > pointsMap.cols ||tempPoint.y < 0 || tempPoint.y > pointsMap.rows)
            continue;
        if (obst_vote[cl[j]] >= voteThres &&
            obst_exist[cl[j]] == -1)
        {
            obst_exist[cl[j]] = 1;
            obst_counter++;
            break;
        }    
        if (obst_vote[cl[j]] < voteThres/2 &&
            obst_exist[cl[j]] == -1)
        {
            obst_exist[cl[j]] = 0;
            obst_counter++;
            break;
        } 
    }

    logicCheck();
    
    rmsc_messages::ObstacleScanFeedback feedback;
    for(int j =0; j < cl.size(); j++)
    {
        Point2i tempPoint(scale*obst_location[cl[j]][checkPoint].y + x_map_offset,
                          scale*obst_location[cl[j]][checkPoint].x + y_map_offset);
        if (tempPoint.x < 0 || tempPoint.x > pointsMap.cols ||tempPoint.y < 0 || tempPoint.y > pointsMap.rows)
            continue;
        if (obst_exist[cl[j]] == 1)
        {
           feedback.found_obst.push_back(cl[j]);
           circle (pointsMap, tempPoint, 10, Scalar(105,255,105), -1);
        }
        else
        {
           circle (pointsMap, tempPoint, 10, Scalar(105,105,105), 2);
        }
        circle (pointsMap, tempPoint, disThres * scale, Scalar(105,155,105), 2); 
    }
       
    for (int j=1; j<=6; j++)
    {
        feedback.obst_status.push_back(obst_exist[j]);
    }

    as_.publishFeedback(feedback);
    if(enable_debug)
    {
       imshow("pointsMap", pointsMap);
       setMouseCallback("pointsMap", OnMouseAction);
       waitKey(10);
    }
}

bool ObstacleScan::isPointInBox(const Point2f &point, const Point2f &centerOfBox, const char direction)
{
    if (direction == 'x')
    {
        if (abs(point.x - centerOfBox.x)<=0.4 && abs(point.y - centerOfBox.y)<=0.15)
           return true;
        else
           return false;
    }
    else  
    {
        if (abs(point.x - centerOfBox.x)<=0.15 && abs(point.y - centerOfBox.y<=0.4))
            return true;
        else 
            return false;

    }
}

bool ObstacleScan::logicCheck()
{
    if (obst_exist[1] == 1)
    {
        if(obst_exist[2] == -1)
           obst_exist[2] = 0;
        if(obst_exist[3] == -1)
           obst_exist[3] = 0;
    }
    if(obst_exist[2] == 1)
    {
        if(obst_exist[1] == -1)
           obst_exist[1] = 0;
        if(obst_exist[3] == -1)
           obst_exist[3] = 0;
    }
    if (obst_exist[3] == 1)
    {
        if(obst_exist[1] == -1)
           obst_exist[1] = 0;
        if(obst_exist[2] == -1)
           obst_exist[2] = 0;
    }

    if (obst_exist[1] == 0 &&
        obst_exist[2] == 0 &&
        obst_exist[3] == -1)
    {
        obst_exist[3] = 1;
    }
    
    if (obst_exist[2] == 0 &&
        obst_exist[3] == 0 &&
        obst_exist[1] == -1)
    {
        obst_exist[1] = 1;
    }
    if (obst_exist[1] == 0 &&
        obst_exist[3] == 0 &&
        obst_exist[2] == -1)
    {
        obst_exist[2] = 1;
    }

    if (obst_exist[4] == 1)
    {
        if(obst_exist[5] == -1)
            obst_exist[5] = 0;
        if(obst_exist[6] == -1)
            obst_exist[6] = 0;
    }
    if (obst_exist[5] == 1)
    {
        if(obst_exist[4] == -1)
            obst_exist[4] = 0;
        if(obst_exist[6] == -1)
        obst_exist[6] = 0;
    }

    if (obst_exist[4] == 0 &&
        obst_exist[5] == 0 &&
        obst_exist[6] == -1)
    {
        obst_exist[6] = 1;
    }
    if (obst_exist[4] == 0 &&
        obst_exist[6] == 0 &&
        obst_exist[5] == -1)
    {
        obst_exist[5] = 1;
    }
    if (obst_exist[5] == 0 &&
        obst_exist[6] == 0 &&
        obst_exist[4] == -1)
    {
        obst_exist[4] = 1;
    }
}

float ObstacleScan::calcDistance(const Point2f &p1, const Point2f &p2)
{
    return sqrt((p1.x - p2.x)*(p1.x - p2.x)+ (p1.y - p2.y)*(p1.y - p2.y));
}

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    FLAGS_v = 3;
    google::InstallFailureSignalHandler();

    ros::init(argc, argv, "obstacle_scan_node");
    ObstacleScan obstacleScan;
    ros::spin();

    return 0;
}
