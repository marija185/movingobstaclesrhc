
#ifndef MOJ_H
#define MOJ_H

#include "Params.h"
#include "Planner.h" //R_point
//#include "DStar.h" //DStarCell
#include <sys/time.h>

#include "ros/ros.h"  //ide u moj.h

//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
//#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "std_msgs/Float64.h"
#include<geometry_msgs/PoseStamped.h>
//#include <boost/bind.hpp>
//#include <boost/thread/mutex.hpp>
#include <tf/transform_listener.h>
//#include <tf/message_filter.h>

#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include <sensor_msgs/Image.h>

#include <visualization_msgs/Marker.h>
#include <movingobstaclesrhc/Moving.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
//#include <slam_msgs/stanje.h>
#include <movingobstaclesrhc/ReqRobStop.h>
//#include <slam_msgs/NewStateReq.h>
//#include <slam_msgs/active_slam.h>
//#include <slam_msgs/NewAugment.h>
using namespace sensor_msgs;
using namespace message_filters;



class VisualizationPublisher
{
protected:
  // Our NodeHandle
  ros::NodeHandle nh_;
  std::string fixed_frame_;


public:
  ros::Publisher marker_pub;
  ros::Publisher shape_pub;
  ros::Publisher efPutanja_pub;
  ros::Publisher DWTrajektorije_pub;
  ros::Publisher DWTrajektorije_pubplus;
  ros::Publisher DWTrajektorije_pubplus1;
  ros::Publisher DWTrajektorije_pubplus2;
  ros::Publisher DWTrajektorije_pubminus;
  ros::Publisher DWTrajektorije_pubminus2;
  ros::Publisher TrajPP_pub;



    visualization_msgs::Marker line_strip;//path
    visualization_msgs::Marker robot_shape;//path
    visualization_msgs::Marker efPut;//effective path
    visualization_msgs::Marker DWTraj;//DW trajectories
    visualization_msgs::Marker DWTrajplus;//DW trajectories
    visualization_msgs::Marker DWTrajplus1;//DW trajectories
    visualization_msgs::Marker DWTrajplus2;//DW trajectories
    visualization_msgs::Marker DWTrajminus2;//DW trajectories
    visualization_msgs::Marker DWTrajminus;//DW trajectories
    visualization_msgs::Marker TrajPP;//moving obstacles

  VisualizationPublisher(ros::NodeHandle n) :
      nh_(n),  fixed_frame_("map") //map kad koristis stage, odom u gazebu simulacija, minefield gazebo kasnija simulacija
  {

#if (ROBOT>=1)
	marker_pub=nh_.advertise<visualization_msgs::Marker>("/visualization_marker",10);
	shape_pub=nh_.advertise<visualization_msgs::Marker>("/shape_marker",10); 
	efPutanja_pub=nh_.advertise<visualization_msgs::Marker>("/ef_putanja",10);
	DWTrajektorije_pub=nh_.advertise<visualization_msgs::Marker>("/DWTrajektorija",10);
	DWTrajektorije_pubplus=nh_.advertise<visualization_msgs::Marker>("/DWTrajplus",10);
	DWTrajektorije_pubplus1=nh_.advertise<visualization_msgs::Marker>("/DWTrajplus1",10);
	DWTrajektorije_pubplus2=nh_.advertise<visualization_msgs::Marker>("/DWTrajplus2",10);
	DWTrajektorije_pubminus=nh_.advertise<visualization_msgs::Marker>("/DWTrajminus",10);
	DWTrajektorije_pubminus2=nh_.advertise<visualization_msgs::Marker>("/DWTrajminus2",10);
	TrajPP_pub=nh_.advertise<visualization_msgs::Marker>("/TrajPP",10);


#else
	marker_pub=nh_.advertise<visualization_msgs::Marker>("/visualization_marker",10);
		shape_pub=nh_.advertise<visualization_msgs::Marker>("/shape_marker",10);
		efPutanja_pub=nh_.advertise<visualization_msgs::Marker>("/ef_putanja",10);
		DWTrajektorije_pub=nh_.advertise<visualization_msgs::Marker>("/DWTrajektorija",10);
		DWTrajektorije_pubplus=nh_.advertise<visualization_msgs::Marker>("/DWTrajplus",10);
		DWTrajektorije_pubplus1=nh_.advertise<visualization_msgs::Marker>("/DWTrajplus1",10);
		DWTrajektorije_pubplus2=nh_.advertise<visualization_msgs::Marker>("/DWTrajplus2",10);
		DWTrajektorije_pubminus=nh_.advertise<visualization_msgs::Marker>("/DWTrajminus",10);
		DWTrajektorije_pubminus2=nh_.advertise<visualization_msgs::Marker>("/DWTrajminus2",10);
		TrajPP_pub=nh_.advertise<visualization_msgs::Marker>("/TrajPP",10);
#endif

      //putanja
    line_strip.header.frame_id = fixed_frame_;
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns =  "movingobstaclesrhc";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w  = 1.0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.05; 
    line_strip.color.r = 0;
    line_strip.color.g = 0.5;
    line_strip.color.b = 0.5;
    line_strip.color.a = 1.0;

    robot_shape.header.frame_id = fixed_frame_;
    robot_shape.header.stamp = ros::Time::now();
    robot_shape.ns =  "movingobstaclesrhc";
    robot_shape.action = visualization_msgs::Marker::ADD;
    robot_shape.pose.orientation.w  = 1.0;
    robot_shape.type = visualization_msgs::Marker::LINE_STRIP;
    robot_shape.scale.x = 0.05; 
    robot_shape.color.r = 0.8;
    robot_shape.color.g = 0.0;
    robot_shape.color.b = 0.5;
    robot_shape.color.a = 1.0;
  
    //efektivna putanja
    efPut.header.frame_id = fixed_frame_;
    efPut.header.stamp = ros::Time::now();
    efPut.ns =  "movingobstaclesrhc";
    efPut.action = visualization_msgs::Marker::ADD;
    efPut.pose.orientation.w  = 1.0;
    efPut.type = visualization_msgs::Marker::LINE_STRIP;
    efPut.scale.x = 0.1; 
    efPut.color.r = 1;
    efPut.color.g = 1;
    efPut.color.b = 0;
    efPut.color.a = 1.0;	
   
    //DW Trajectory
    DWTraj.header.frame_id = fixed_frame_;
    DWTraj.header.stamp = ros::Time::now();
    DWTraj.ns =  "movingobstaclesrhc";
    DWTraj.action = visualization_msgs::Marker::ADD;
    DWTraj.pose.orientation.w  = 1.0;
    DWTraj.type = visualization_msgs::Marker::LINE_LIST;
    DWTraj.scale.x = 0.01; 
    DWTraj.color.a = 1.0;	
    DWTraj.color.r = 0.7;
    DWTraj.color.g = 0.8;
    DWTraj.color.b = 1;

    DWTrajplus.header.frame_id = fixed_frame_;
    DWTrajplus.header.stamp = ros::Time::now();
    DWTrajplus.ns =  "movingobstaclesrhc";
    DWTrajplus.action = visualization_msgs::Marker::ADD;
    DWTrajplus.pose.orientation.w  = 1.0;
    DWTrajplus.type = visualization_msgs::Marker::LINE_LIST;
    DWTrajplus.scale.x = 0.01; 
    DWTrajplus.color.a = 1.0;	
    DWTrajplus.color.r = 0;
    DWTrajplus.color.g = 0;
    DWTrajplus.color.b = 0.8;

    DWTrajplus1.header.frame_id = fixed_frame_;
    DWTrajplus1.header.stamp = ros::Time::now();
    DWTrajplus1.ns =  "movingobstaclesrhc";
    DWTrajplus1.action = visualization_msgs::Marker::ADD;
    DWTrajplus1.pose.orientation.w  = 1.0;
    DWTrajplus1.type = visualization_msgs::Marker::LINE_LIST;
    DWTrajplus1.scale.x = 0.01; 
    DWTrajplus1.color.a = 1.0;	
    DWTrajplus1.color.r = 0;
    DWTrajplus1.color.g = 0.1;
    DWTrajplus1.color.b = 0.3;

    DWTrajplus2.header.frame_id = fixed_frame_;
    DWTrajplus2.header.stamp = ros::Time::now();
    DWTrajplus2.ns =  "movingobstaclesrhc";
    DWTrajplus2.action = visualization_msgs::Marker::ADD;
    DWTrajplus2.pose.orientation.w  = 1.0;
    DWTrajplus2.type = visualization_msgs::Marker::LINE_LIST;
    DWTrajplus2.scale.x = 0.01; 
    DWTrajplus2.color.a = 1.0;	
    DWTrajplus2.color.r = 0.3;
    DWTrajplus2.color.g = 0;
    DWTrajplus2.color.b = 0.3;

    DWTrajminus2.header.frame_id = fixed_frame_;
    DWTrajminus2.header.stamp = ros::Time::now();
    DWTrajminus2.ns =  "movingobstaclesrhc";
    DWTrajminus2.action = visualization_msgs::Marker::ADD;
    DWTrajminus2.pose.orientation.w  = 1.0;
    DWTrajminus2.type = visualization_msgs::Marker::LINE_LIST;
    DWTrajminus2.scale.x = 0.01; 
    DWTrajminus2.color.a = 1.0;	
    DWTrajminus2.color.r = 1;
    DWTrajminus2.color.g = 0.5;
    DWTrajminus2.color.b = 0.8;

    DWTrajminus.header.frame_id = fixed_frame_;
    DWTrajminus.header.stamp = ros::Time::now();
    DWTrajminus.ns =  "movingobstaclesrhc";
    DWTrajminus.action = visualization_msgs::Marker::ADD;
    DWTrajminus.pose.orientation.w  = 1.0;
    DWTrajminus.type = visualization_msgs::Marker::LINE_LIST;
    DWTrajminus.scale.x = 0.01; 
    DWTrajminus.color.a = 1.0;	
    DWTrajminus.color.r = 1;
    DWTrajminus.color.g = 0;
    DWTrajminus.color.b = 0.2;

    //TrajektorijePokretnePrepreke
    TrajPP.header.frame_id = fixed_frame_;
    TrajPP.header.stamp = ros::Time::now();
    TrajPP.ns =  "movingobstaclesrhc";
    TrajPP.action = visualization_msgs::Marker::ADD;
    TrajPP.pose.orientation.w  = 1.0;
    TrajPP.type = visualization_msgs::Marker::LINE_STRIP;
    TrajPP.scale.x = 0.1; 
    TrajPP.color.r = 0.156863;
    TrajPP.color.g = 0.156863;
    TrajPP.color.b = 0.156863;
    TrajPP.color.a = 1.0;
    
  }

  void visualizationduringmotion();


};

class GenericLaserScanFilterNode
{
protected:
  // Our NodeHandle
  ros::NodeHandle nh_;

//read from params
  std::string base_frame_;
  std::string odom_topic_;
  std::string scan_topic_;
  std::string cmd_vel_topic_;
  
  // Components for tf::MessageFilter
  tf::TransformListener tf_listener;
  message_filters::Subscriber<sensor_msgs::LaserScan> sub_laser_front;
  message_filters::Subscriber<sensor_msgs::LaserScan> sub_laser_rear;
  tf::MessageFilter<sensor_msgs::LaserScan>* filter_laser_front;
  tf::MessageFilter<sensor_msgs::LaserScan>* filter_laser_rear;


public:

  tf::StampedTransform transform_laser;

//globalne varijable za callbackove
double v, omega, vy;
double ranges[PLAYER_LASER_MAX_SAMPLES];
R_point glob[PLAYER_LASER_MAX_SAMPLES];
double ranges2[PLAYER_LASER_MAX_SAMPLES];
R_point glob2[PLAYER_LASER_MAX_SAMPLES];
double lasertfx;//laser coordinates (global)
double lasertfy;
double lasertfth;
double lasertfx2;//for the second laser in mm
double lasertfy2;
double lasertfth2;
double sickAngularOffset;
double v_max, dv_max, v_min, vy_max, vx_max, vx_min, vy_min, dvx_max, dvy_max, w_max, w_min, dw_max;
double robottfx, robottfy, robottfth;
int lasercount;
int lasercount2;
ros::Time laser1stamp, laser2stamp;
double angle_min, angle_increment;
double angle_min2, angle_increment2;
double robotX, robotY,robotTH,robotW,robotV;

  // Constructor
  typedef sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, sensor_msgs::LaserScan> MySyncPolicy;
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> odom_sink_sub;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;
  ros::Subscriber odom_sub, pose_sub;
  message_filters::Synchronizer<MySyncPolicy>* sync;
  
  ros::Publisher vel_pub;
  ros::Publisher moving_pub;
  ros::Publisher active_pub;
  ros::Subscriber moving_sub;
  ros::ServiceServer service;
  ros::ServiceClient client,client_augment;
  ros::Subscriber set_goal;


  GenericLaserScanFilterNode(ros::NodeHandle n)
  {
      nh_=(n);
      //defaults
      base_frame_ = "base_link";
      odom_topic_ = "/odom";
      scan_topic_ = "/base_scan";
      cmd_vel_topic_ = "cmd_vel";
      sickAngularOffset = 0;//M_PI/2.;
      vy_max = 0.; //non zero for omnidrive
      vx_max = 1000.; //diff
      vx_min = 0.; //no reverse
      vy_min = 0.;
      dvx_max = 600.; //acc
      dvy_max = 0.; //omnidrive
      w_max = 100.;
      w_min	= -100.;
      dw_max = 100.;

      nh_.getParam("/spoj/base_frame", base_frame_);
      nh_.getParam("/spoj/odom_topic", odom_topic_);
      nh_.getParam("/spoj/scan_topic", scan_topic_);
      nh_.getParam("/spoj/cmd_vel_topic", cmd_vel_topic_);
      nh_.getParam("/spoj/sick_angular_offset", sickAngularOffset);
      //down params not used yet
      nh_.getParam("/spoj/vx_min", vx_min);
      nh_.getParam("/spoj/vx_max", vx_max);
      nh_.getParam("/spoj/vy_min", vy_min);
      nh_.getParam("/spoj/vy_max", vy_max);
      nh_.getParam("/spoj/dvx_max", dvx_max);
      nh_.getParam("/spoj/dvy_max", dvy_max);
      nh_.getParam("/spoj/w_min", w_min);
      nh_.getParam("/spoj/w_max", w_max);
      nh_.getParam("/spoj/dw_max", dw_max);
      //convertion to rad
      w_min = w_min * M_PI/180.;
      w_max = w_max * M_PI/180.;
      dw_max = dw_max * M_PI/180.;
      v_max = std::max(vx_max,vy_max); //total max velocity
      dv_max = std::max(dvx_max,dvy_max); //total max acceleration
      v_min = 0.;
      nh_.getParam("/spoj/v_min", v_min);
      nh_.getParam("/spoj/v_max", v_max);
      nh_.getParam("/spoj/dv_max", dv_max);
      

#if (ROBOT==1)
      sub_laser_front.subscribe(nh_, "/scan", 1);
      sub_laser_rear.subscribe(nh_, "/scan_rear", 1); //for kuka: /scan_front, for pioneer3dx: /scan
#else
      sub_laser_front.subscribe(nh_, scan_topic_, 1); //scan2 za hratc da ne koristi laser
      sub_laser_rear.subscribe(nh_, "/scan_rear", 1);
#endif
	set_goal=nh_.subscribe("/move_base_simple/goal",1,&GenericLaserScanFilterNode::simple_goal,this); //ais_pose
  moving_sub = nh_.subscribe("movingRobot0", 1, &GenericLaserScanFilterNode::movingRobot0Callback, this);
	vel_pub=nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1); //samo cmd_vel /husky/cmd_vel
  moving_pub = nh_.advertise<movingobstaclesrhc::Moving>("movingRobot1",1);
  service = nh_.advertiseService("request_stop", &GenericLaserScanFilterNode::ReqRobotStop,this);
  odom_sub = nh_.subscribe(odom_topic_, 10, &GenericLaserScanFilterNode::odomCallback, this); // /odom /brzine_gazebo /encoder za husky hratc
#if 0  
	odom_sink_sub.subscribe(nh_,"/ais_pose",1); //ais_pose  /robot_pose_ekf/odom /odom_gazebo
//  client = nh_.serviceClient<slam_msgs::NewStateReq>("new_state");
//  client_augment = nh_.serviceClient<slam_msgs::NewAugment>("new_augment");
//  active_pub = nh_.advertise<slam_msgs::active_slam>("active_m",10);
#if (ROBOT==1)
	laser_sub.subscribe(nh_,"/lidar/scan",1);
  odom_sub = nh_.subscribe("/slam/odom", 1, &GenericLaserScanFilterNode::odomCallback, this);
#else
	laser_sub.subscribe(nh_,"/scan",10);// base_scan // /scan za huski gazebo
  pose_sub = nh_.subscribe("/amcl_pose", 10, &GenericLaserScanFilterNode::globalposeCallback, this); //minefield_pose za hratc
#endif
//	sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(8), odom_sink_sub,laser_sub);
//	sync->registerCallback(boost::bind(&GenericLaserScanFilterNode::sink_call, this, _1, _2));
    filter_laser_front = new tf::MessageFilter<sensor_msgs::LaserScan>(sub_laser_front, tf_listener, "minefield", 10);
    filter_laser_front->registerCallback( boost::bind(&GenericLaserScanFilterNode::laserCallback, this, _1));
#else
    filter_laser_front = new tf::MessageFilter<sensor_msgs::LaserScan>(sub_laser_front, tf_listener, "/map", 10);
    filter_laser_front->registerCallback( boost::bind(&GenericLaserScanFilterNode::laserCallback, this, _1));
#if (1)
    filter_laser_rear = new tf::MessageFilter<sensor_msgs::LaserScan>(sub_laser_rear, tf_listener, "/map", 100);
    filter_laser_rear->registerCallback( boost::bind(&GenericLaserScanFilterNode::laserRearCallback, this, _1));
#endif
#endif
  }

  // Callback
  void sink_call(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& odom, const sensor_msgs::LaserScan::ConstPtr& msg);
  void odomCallback(const nav_msgs::OdometryConstPtr& odom);
  void globalposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void movingRobot0Callback(const movingobstaclesrhc::MovingConstPtr& robot0Msg);
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void laserRearCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  bool ReqRobotStop(movingobstaclesrhc::ReqRobStop::Request &req, movingobstaclesrhc::ReqRobStop::Response &res);
  void simple_goal(const geometry_msgs::PoseStamped::ConstPtr& simple_g);



};

struct posevel{

	// Pose
	double x, y, th;

 	// velocities
	double v, w, vy;

};


struct R_polar{
	double  r, th;
};
struct R_vel{
	double  v, w, vy;
};

//laser
typedef struct{
	R_point point[PLAYER_LASER_MAX_SAMPLES];
	R_polar scan[PLAYER_LASER_MAX_SAMPLES];
	int scan_count;      
}moj_Laser;



class moj: public GenericLaserScanFilterNode {

//protected:
//  // Our NodeHandle
//  ros::NodeHandle nh;
  
public:

	moj_Laser LB;              //laser
	moj_Laser mali;
	moj_Laser mali2;
	moj_Laser LB2;
	posevel RB; //odometry
	R_vel setvel;//set velocities
	R_point goal,old_goal;
	char *wld;
	posevel start;
	R_point subgoal;//temp goal if no path
	R_point subgoal2;
	bool subgoal2flag;
	int mySecStart0,myMSecStart0;
	double footprintx[15],footprinty[15];
	int metric;
	int ciklus;
	bool voznja;
    bool novicilj;
    bool zavrsio;
    int slam_loop;
    int snimanje;


//	VisualizationPublisher *visual;
//	ros::NodeHandle nh;
public:
	// Konstruktor
	moj(ros::NodeHandle n):GenericLaserScanFilterNode(n){}
//  moj();
	int main(int argc, char **argv);
	void initializePlanner();
  void executeMotion();
  void gotogoal(R_point newgoal);

	// Destruktor
	~moj();
  
};
#endif
