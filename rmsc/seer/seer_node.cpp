#include "seer_node.h"
#include <ros/package.h>
#include <vector>

using namespace std;
using namespace cv;
using namespace seer;

int scale;
float ammobox_dis_thres;
float obst_dis_thres;
int ammobox_vote_thres;
int obst_vote_thres;
int x_map_offset;
int y_map_offset;
bool enable_debug;
std::string high_lidar_topic;
std::string low_lidar_topic;
std::string pub_topic;

static void OnMouseAction_low(int event,int x,int y,int flags,void *ustc)
{	
	if(event==CV_EVENT_LBUTTONUP)
	{
		cout<< "    # Obstacle " << endl 
            << "    data: " << (y - y_map_offset)/(1.0*scale) << endl 
            << "    data: " << (x - x_map_offset)/(1.0*scale) <<  endl;
	}
}

static void OnMouseAction_high(int event,int x,int y,int flags,void *ustc)
{	
	if(event==CV_EVENT_LBUTTONUP)
	{
		cout<< "    # Ammobox" << endl 
            << "    data: " << (y - y_map_offset)/(-1.0*scale) << endl 
            << "    data: " << (x - x_map_offset)/(-1.0*scale) <<  endl;
	}
}

Seer::Seer():as_(nh_, "seer_action", boost::bind(&Seer::ActionCB, this, _1), false)
{
    loadArguments();

    obstacle_pub_ = nh_.advertise<std_msgs::Int32>("/obstacle_scan", 10);
    master_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/master/ammo_scan", 10);
    wing_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/wing/ammo_scan", 10);
    ammobox_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/seer_debug", 10);

    scale = args.scale();
    ammobox_dis_thres = args.ammobox_dis_thres();
    obst_dis_thres = args.obst_dis_thres();
    ammobox_vote_thres = args.ammobox_vote_thres();
    obst_vote_thres = args.obst_vote_thres();
    high_lidar_topic = args.high_lidar_topic();
    low_lidar_topic = args.low_lidar_topic();
    x_map_offset = args.x_map_offset();
    y_map_offset = args.y_map_offset();
    enable_debug = args.enable_debug();

    running_ = false;
    obst_known = false;
    ammobox_known = false;
    outputOnce = false;

    //if (enable_debug)
    //{
       running_ = true;
    //}
    
    low_scan_sub_ = nh_.subscribe( low_lidar_topic, 1, &Seer::lowLidarCallback, this);
    high_scan_sub_ = nh_.subscribe( high_lidar_topic, 1, &Seer::highLidarCallback, this);

    if (args.iam() == "RED")
    {
        obst_location[3] = cv::Point2f(args.red_obstacle_location().data(0), args.red_obstacle_location().data(1));
        obst_location[4] = cv::Point2f(args.red_obstacle_location().data(2), args.red_obstacle_location().data(3));
        obst_location[6] = cv::Point2f(args.red_obstacle_location().data(4), args.red_obstacle_location().data(5));
        
        ammobox_location[7] = cv::Point2f(args.red_ammobox_location().data(0), args.red_ammobox_location().data(1));
        ammobox_location[8] = cv::Point2f(args.red_ammobox_location().data(2), args.red_ammobox_location().data(3));
        ammobox_location[9] = cv::Point2f(args.red_ammobox_location().data(4), args.red_ammobox_location().data(5));
        ammobox_location[10] = cv::Point2f(args.red_ammobox_location().data(6), args.red_ammobox_location().data(7));
        ammobox_location[11] = cv::Point2f(args.red_ammobox_location().data(8), args.red_ammobox_location().data(9));
        ammobox_location[12] = cv::Point2f(args.red_ammobox_location().data(10), args.red_ammobox_location().data(11));
        ammobox_location[13] = cv::Point2f(args.red_ammobox_location().data(12), args.red_ammobox_location().data(13));
        ammobox_location[14] = cv::Point2f(args.red_ammobox_location().data(14), args.red_ammobox_location().data(15));
        ammobox_location[15] = cv::Point2f(args.red_ammobox_location().data(16), args.red_ammobox_location().data(17));
    }
    
    if (args.iam() == "BLUE")
    {
        obst_location[3] = cv::Point2f(args.blue_obstacle_location().data(0), args.blue_obstacle_location().data(1));
        obst_location[4] = cv::Point2f(args.blue_obstacle_location().data(2), args.blue_obstacle_location().data(3));
        obst_location[6] = cv::Point2f(args.blue_obstacle_location().data(4), args.blue_obstacle_location().data(5));
        
        ammobox_location[7] = cv::Point2f(args.blue_ammobox_location().data(0), args.blue_ammobox_location().data(1));
        ammobox_location[8] = cv::Point2f(args.blue_ammobox_location().data(2), args.blue_ammobox_location().data(3));
        ammobox_location[9] = cv::Point2f(args.blue_ammobox_location().data(4), args.blue_ammobox_location().data(5));
        ammobox_location[10] = cv::Point2f(args.blue_ammobox_location().data(6), args.blue_ammobox_location().data(7));
        ammobox_location[11] = cv::Point2f(args.blue_ammobox_location().data(8), args.blue_ammobox_location().data(9));
        ammobox_location[12] = cv::Point2f(args.blue_ammobox_location().data(10), args.blue_ammobox_location().data(11));
        ammobox_location[13] = cv::Point2f(args.blue_ammobox_location().data(12), args.blue_ammobox_location().data(13));
        ammobox_location[14] = cv::Point2f(args.blue_ammobox_location().data(14), args.blue_ammobox_location().data(15));
        ammobox_location[15] = cv::Point2f(args.blue_ammobox_location().data(16), args.blue_ammobox_location().data(17));
    }
    
    cout << "I am " << args.iam() << " infantry." << endl;

    for(int i = 0; i < 30; i++)
    {
        obst_exist[i] = -1;
        ammobox_exist[i] = 0;
    }

    cout << "waiting for lider data ..." << endl;

    as_.start();
}

Seer::~Seer(){
}

bool Seer::loadArguments() 
{
    string path = ros::package::getPath("roborts") + "/rmsc/seer/proto/arguments.prototxt";
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

void Seer::lowLidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    if(!running_) return;

    sensor_msgs::PointCloud2 rosCloud;
    projector_.projectLaser(*scan, rosCloud);

    int size = rosCloud.width;
    int pointStep = rosCloud.point_step;
    
    vector<cv::Point2f> pointsList;
    for (int i=0; i<size; i++)
    {
        float tX = 0.0;
        float tY = 0.0;
        int arrayPosX;
        int arrayPosY;
        arrayPosX =i * pointStep + rosCloud.fields[0].offset;
        arrayPosY =i * pointStep + rosCloud.fields[1].offset;
        memcpy(&tX, &rosCloud.data[arrayPosX], sizeof(float));
        memcpy(&tY, &rosCloud.data[arrayPosY], sizeof(float));
        pointsList.push_back(Point2f(tX, tY));
    }  

    int obst_vote[10] = {0};
    cv::Mat pointsMap = cv::Mat::zeros(800, 800, CV_8UC3);
   
    if(enable_debug){
      cv::Rect infantry(x_map_offset-30, y_map_offset-40, 60, 80);
      cv::rectangle(pointsMap, infantry, Scalar(50, 50, 120), 3);
      cv::line(pointsMap, Point(x_map_offset-30, y_map_offset-40), Point(x_map_offset+30, y_map_offset+40), Scalar(50, 50, 120), 3);
      cv::line(pointsMap, Point(x_map_offset+30, y_map_offset-40), Point(x_map_offset-30, y_map_offset+40), Scalar(50, 50, 120), 3);
    }

    vector<int> cl; // = checklist;
    cl.push_back(3);
    cl.push_back(4);
    cl.push_back(6);
    
    for(int i=0; i<pointsList.size(); i++)
    {
        Point2i tempPoint(scale * pointsList[i].y + x_map_offset, scale * pointsList[i].x + y_map_offset);
        if (tempPoint.x < 0 || tempPoint.x > pointsMap.cols ||
            tempPoint.y < 0 || tempPoint.y > pointsMap.rows)
              continue;

        circle (pointsMap, tempPoint, 1, Scalar(255,255,255));

        for(int j = 0; j < cl.size(); j++)
          if (calcDistance(pointsList[i], obst_location[cl[j]]) < obst_dis_thres)
          {
              obst_vote[cl[j]]++;
              circle (pointsMap, tempPoint, 1, Scalar(105,255,105));
          }
    }
    
    if (obst_vote[6] >= obst_vote_thres)
    {
        obst_exist[6] = 1;
        obst_exist[4] = 0;
        obst_exist[5] = 0;
        which_obst = 6;

        if (obst_vote[3] >= obst_vote_thres)
        {
            obst_exist[3] = 1;
            obst_exist[1] = 0;
            obst_exist[2] = 0;
        }
        else
        {
            obst_exist[3] = 0;
            obst_exist[1] = -1;
            obst_exist[2] = -1;
        }

        obst_known = true;
    }    

    if (obst_vote[4] >= obst_vote_thres)
    {
        obst_exist[4] = 1;
        obst_exist[5] = 0;
        obst_exist[6] = 0;
        which_obst = 4;

        obst_exist[1] = -1;
        obst_exist[2] = -1;
        obst_exist[3] = -1;
        
        obst_known = true;
    }

    if ((obst_vote[6] < obst_vote_thres) && (obst_vote[4] < obst_vote_thres))
    {
        obst_exist[5] = 1;
        obst_exist[4] = 0;
        obst_exist[6] = 0;
        which_obst = 5;

        if (obst_vote[3] >= obst_vote_thres)
        {
            obst_exist[3] = 1;
            obst_exist[1] = 0;
            obst_exist[2] = 0;
        }
        else
        {
            obst_exist[3] = 0;
            obst_exist[1] = -1;
            obst_exist[2] = -1;
        }
        
        obst_known = true;
    }

    if(enable_debug)
    {
        for(int j =0; j < cl.size(); j++)
        {
            Point2i tempPoint(scale*obst_location[cl[j]].y + x_map_offset,
                              scale*obst_location[cl[j]].x + y_map_offset);
            if (tempPoint.x < 0 || tempPoint.x > pointsMap.cols ||
                tempPoint.y < 0 || tempPoint.y > pointsMap.rows)
                continue;
            if (obst_exist[cl[j]] == 1)
               circle (pointsMap, tempPoint, 10, Scalar(105,255,105), -1);
            else
               circle (pointsMap, tempPoint, 10, Scalar(105,105,105), 2);
            circle (pointsMap, tempPoint, obst_dis_thres * scale, Scalar(105,155,105), 2); 
        }

        imshow("Low Lidar Map", pointsMap);
        setMouseCallback("Low Lidar Map", OnMouseAction_low);
        waitKey(5);
    }

    if (!ammobox_known)
        cout << "ammobox unknown!"<<endl;
    if (ammobox_known && obst_known)
    {
        //cout << "arranging" << endl;
        arrange();
    }
    else
    {
        cout << "obstacle known, but ammobox unknown!" << endl;
    }
}

void Seer::highLidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    sensor_msgs::PointCloud2 rosCloud;
    projector_.projectLaser(*scan, rosCloud);

    int size = rosCloud.width;
    int pointStep = rosCloud.point_step;
    
    vector<cv::Point2f> pointsList;
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
        pointsList.push_back(Point2f(tX, tY));
    }  
 
    int ammobox_vote[35] = {0};

    cv::Mat pointsMap = cv::Mat::zeros(800, 800, CV_8UC3);

    if(enable_debug){
        cv::Rect infantry(x_map_offset-30, y_map_offset-40, 60, 80);
        cv::rectangle(pointsMap, infantry, Scalar(50, 50, 120), 3);
        cv::line(pointsMap, Point(x_map_offset-30, y_map_offset-40), Point(x_map_offset+30, y_map_offset+40), Scalar(50, 50, 120), 3);
        cv::line(pointsMap, Point(x_map_offset+30, y_map_offset-40), Point(x_map_offset-30, y_map_offset+40), Scalar(50, 50, 120), 3);
    }

    for(int i=0; i<pointsList.size(); i++)
    {
        
        Point2i tempPoint(-scale*pointsList[i].y + x_map_offset, 
                          -scale*pointsList[i].x + y_map_offset);
        if (tempPoint.x < 0 || tempPoint.x > pointsMap.cols ||
            tempPoint.y < 0 || tempPoint.y > pointsMap.rows)
            continue;
        circle (pointsMap, tempPoint, 1, Scalar(255,255,255));

        for(int j =7; j <= 15; j++)
        if (calcDistance(pointsList[i], ammobox_location[j]) < ammobox_dis_thres)
        {
            ammobox_vote[j]++;
            circle (pointsMap, tempPoint, 1, Scalar(105,255,105));
        }
    }

    for (int j=7; j<=15; j++)
    {
        Point2i tempPoint(-scale*ammobox_location[j].y + x_map_offset, 
                          -scale*ammobox_location[j].x + y_map_offset);
        if (tempPoint.x < 0 || tempPoint.x > pointsMap.cols ||
            tempPoint.y < 0 || tempPoint.y > pointsMap.rows)
            continue;
        if (ammobox_vote[j] >= ammobox_vote_thres)
        {
            circle (pointsMap, tempPoint, 7, Scalar(105,255,105), -1);
            ammobox_exist[j] = 1;
        }
        else
        {
            circle (pointsMap, tempPoint, 7, Scalar(105,105,105), 2);
            ammobox_exist[j] = 0;
        }
        circle (pointsMap, tempPoint, ammobox_dis_thres * scale, Scalar(105,135,105));
    }
    ammobox_known = true;

    if (!obst_known)
        cout << "obstacle unknown!"<<endl;

    if (ammobox_known && obst_known)
    {
        //cout << "arranging" << endl;
        arrange();
    }
    else
    {
        cout << "ammobox known, but obstacle unknown!" << endl;
    }

    if(enable_debug)
    {
       imshow("High Lidar Map", pointsMap);
       setMouseCallback("High Lidar Map", OnMouseAction_high);
       waitKey(5);
    }

}

void Seer::arrange()
{


    ammobox_exist[2] = 1;
    ammobox_exist[3] = 1;
    ammobox_exist[4] = 1;
    ammobox_exist[5] = 1;

    /*
    cout << "input obstacle number: (4-6)" ;
    cin >> which_obst;
    obst_exist[3] = 1;
    cout << "input ammobox number: (7-15)";
    
    for (int i = 0; i < 3; i++)
    {
        int temp;
        cin >> temp;
        ammobox_exist[temp] = 1;
    }

    for (int i = 7; i <= 15; i++)
    {
        cout << i << "\t";
    }
    cout << endl;
    for (int i = 7; i <= 15; i++)
    {
        cout << ammobox_exist[i] << "\t";
    }
    
    cout << endl;
    
    */
    vector<int> m_cl;
    vector<int> w_cl;

    switch (which_obst)
    {
        case 4:
            for (int i = 0; i < args.obst4m().data_size(); i++)
            {
                m_cl.push_back(args.obst4m().data(i));
            }
            for (int i = 0; i < args.obst4w().data_size(); i++)
            {
                w_cl.push_back(args.obst4w().data(i));
            }    
            break;
        case 5:
            for (int i = 0; i < args.obst5m().data_size(); i++)
            {
                m_cl.push_back(args.obst5m().data(i));
            }
            for (int i = 0; i < args.obst5w().data_size(); i++)
            {
                w_cl.push_back(args.obst5w().data(i));
            }    
            break;
        case 6:
            for (int i = 0; i < args.obst6m().data_size(); i++)
            {
                m_cl.push_back(args.obst6m().data(i));
            }
            for (int i = 0; i < args.obst6w().data_size(); i++)
            {
                w_cl.push_back(args.obst6w().data(i));
            }    
            break;
        default:
            cout << "no obstacle in 4, 5 or 6" << endl;
    }

    for(int i = 0; i < w_cl.size(); i++)
        if ((w_cl[i] == 7)&&(obst_exist[3] == 1))
            w_cl[i] = 22;

    std::vector<int> m_goal, w_goal;
    for (int i = 0; i < m_cl.size(); i++)
    {
        if(ammobox_exist[trans[m_cl[i]]])
        {
            m_goal.push_back(m_cl[i]);
        }
    }
    
    for (int i = 0; i < w_cl.size(); i++)
    {
        if(ammobox_exist[trans[w_cl[i]]])
        {
            w_goal.push_back(w_cl[i]);
        }
    }

    
    std_msgs::Int32 obst_result;
    std_msgs::Int32MultiArray master_result, wing_result, ammobox_result;

    obst_result.data = which_obst - 3;
    if (obst_exist[3] == 1)
       obst_result.data +=3;
    obstacle_pub_.publish(obst_result);

    for (int i = 0; i < m_goal.size(); i++)
    {
        master_result.data.push_back(m_goal[i]);
    }
    for (int i = 0; i < w_goal.size(); i++)
    {
        wing_result.data.push_back(w_goal[i]);
    }
    master_pub_.publish(master_result);
    wing_pub_.publish(wing_result);

    for (int i = 0; i <=15; i++)
    {
        ammobox_result.data.push_back(ammobox_exist[i]);
    }
    ammobox_pub_.publish(ammobox_result);
}


float Seer::calcDistance(const Point2f &p1, const Point2f &p2)
{
    return sqrt((p1.x - p2.x)*(p1.x - p2.x)+ (p1.y - p2.y)*(p1.y - p2.y));
}


void Seer::ActionCB(const rmsc_messages::SeerGoal::ConstPtr &data)
{
    switch (data->cmd) {
      case 1:
        running_ = true;
        as_.setSucceeded();
        std::cerr << "launched! waiting for message!" << endl;
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
int main(int argc, char** argv)
{
    ros::init(argc, argv, "seer_node");
    Seer seer;
    ros::spin();

    return 0;
}
