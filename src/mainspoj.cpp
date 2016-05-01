#include "moj.h"   //DStar.h, Params.h, Planner.h
#include "WorkHorse.h"    //zbog WH

#include "DStar.h"    //zbog DS
#include "GridMap.h"    //zbog GM
#include "DynamicWindow.h" //zbog DW
#include <iostream>
#include <math.h>
//#include "args.h"

#include "boost/filesystem.hpp"   // includes all needed Boost.Filesystem declarations


#if RECTANGULAR
#include <cspacevoronoi.h>
//#else
//#include <vector>
#endif

//extern variables
WorkHorse *WH;
	moj *M;
extern DStar *DS;
extern GridMap *GM;
extern DynamicWindow *DW;
extern Planner *PL;
#if RECTANGULAR
CSpaceVoronoi *cspace;
#endif

double V_MAX, DV_MAX, V_MIN, VY_MAX, VX_MAX, VX_MIN, VY_MIN, DVX_MAX, DVY_MAX, W_MAX, W_MIN, DW_MAX;

void VisualizationPublisher::visualizationduringmotion(){

    	R_point *temp_point;
    	int temp_length;

      line_strip.points.clear();
      robot_shape.points.clear();
      efPut.points.clear();
      DWTrajplus1.points.clear();
      DWTrajplus2.points.clear();
      TrajPP.points.clear();


      int metric=M->metric;
			DWTraj.points.clear();
			DWTrajplus.points.clear();
			DWTrajminus.points.clear();
			DWTrajminus2.points.clear();
			if(PL->GetPathLength()>0){
				temp_length=PL->GetPathLength();
				temp_point=PL->GetPath();
				geometry_msgs::Point p; 	
				for(int pathLength=0; pathLength<temp_length;pathLength++){
					//for(int neke=0;neke<100;neke++) 							
			        	p.x = temp_point[pathLength].x/metric;
    					p.y = temp_point[pathLength].y/metric;
    					p.z = 0;

    					line_strip.points.push_back(p);
				}
			//publish path			
			marker_pub.publish(line_strip);
		
			}
			
			//robot footprint
			geometry_msgs::Point p; 
			for (int ii=0; ii<15; ii++){
				p.x=(M->RB.x+cos(M->RB.th)*M->footprintx[ii]-sin(M->RB.th)*M->footprinty[ii])/metric;
				p.y=(M->RB.y+sin(M->RB.th)*M->footprintx[ii]+cos(M->RB.th)*M->footprinty[ii])/metric;
				p.z=0;
				robot_shape.points.push_back(p);
			}
			shape_pub.publish(robot_shape);

			//efektivna putanja
			for(int ief=0; ief<DW->effective_path_length;ief++)
			{
				geometry_msgs::Point pt;
				pt.x=DW->path_r_reduced[ief].x/metric;
				pt.y=DW->path_r_reduced[ief].y/metric;
				pt.z=0;
				efPut.points.push_back(pt);
			}
			efPutanja_pub.publish(efPut);

			//DW Trajektorija
			geometry_msgs::Point ptdw;

#if OMNIDRIVE
			for(int dwl=0;dwl<VY_DIM+1;dwl++)
#endif
			{
			for(int dwj=0;dwj<V_DIM+1;dwj++)
			{
				for(int dwk=0;dwk<W_DIM+1;dwk++)
				{
					//iscrtaj DWTrajektoriju
#if OMNIDRIVE
					if ((DW->TB.flag[dwj][dwk][dwk] == CLEAR)) //||(DW->TB.flag[dwj][dwk] == HAS_OBSTACLE))
#else
					if ((DW->TB.flag[dwj][dwk] == CLEAR)) //||(DW->TB.flag[dwj][dwk] == HAS_OBSTACLE))
#endif
					{
					for(int dwi=0;dwi<N_KL-1;dwi++)
					{
#if OMNIDRIVE
						ptdw.x=DW->Log.xkl[dwi][dwj][dwk][dwl]/metric;
						ptdw.y=DW->Log.ykl[dwi][dwj][dwk][dwl]/metric;
						ptdw.z=0*DW->Log.Skl[dwi][dwj][dwk][dwl]/metric;
#else						
						ptdw.x=DW->Log.xkl[dwi][dwj][dwk]/metric;
						ptdw.y=DW->Log.ykl[dwi][dwj][dwk]/metric;
						ptdw.z=0*DW->Log.Skl[dwi][dwj][dwk]/metric;
#endif
						DWTraj.points.push_back(ptdw);
					}
#if OMNIDRIVE
					ptdw.x=DW->Log.xkl[N_KL-2][dwj][dwk][dwl]/metric;
					ptdw.y=DW->Log.ykl[N_KL-2][dwj][dwk][dwl]/metric;
					ptdw.z=0*DW->Log.Skl[N_KL-2][dwj][dwk][dwl]/metric;
#else
					ptdw.x=DW->Log.xkl[N_KL-2][dwj][dwk]/metric;
					ptdw.y=DW->Log.ykl[N_KL-2][dwj][dwk]/metric;
					ptdw.z=0*DW->Log.Skl[N_KL-2][dwj][dwk]/metric;
#endif
					DWTraj.points.push_back(ptdw);
#if OMNIDRIVE
					ptdw.x=DW->Log.xkl[N_KL-1][dwj][dwk][dwl]/metric;
					ptdw.y=DW->Log.ykl[N_KL-1][dwj][dwk][dwl]/metric;
					ptdw.z=0*DW->Log.Skl[N_KL-1][dwj][dwk][dwl]/metric;
#else
					ptdw.x=DW->Log.xkl[N_KL-1][dwj][dwk]/metric;
					ptdw.y=DW->Log.ykl[N_KL-1][dwj][dwk]/metric;
					ptdw.z=0*DW->Log.Skl[N_KL-1][dwj][dwk]/metric;
#endif
					DWTraj.points.push_back(ptdw);
					}
#if (KRUZNI_LUK==0)
					//iscrtaj DWTrajektoriju
#if OMNIDRIVE
					if ((DW->TB_plus.flag[dwj][dwk][dwk] == CLEAR)) //||(DW->TB.flag[dwj][dwk] == HAS_OBSTACLE))
#else
					if ((DW->TB_plus.flag[dwj][dwk] == CLEAR)) //||(DW->TB.flag[dwj][dwk] == HAS_OBSTACLE))
#endif
					{
					for(int dwi=0;dwi<N_KL-1;dwi++)
					{
#if OMNIDRIVE
						ptdw.x=DW->Log_plus.xkl[dwi][dwj][dwk][dwl]/metric;
						ptdw.y=DW->Log_plus.ykl[dwi][dwj][dwk][dwl]/metric;
						ptdw.z=0*DW->Log_plus.Skl[dwi][dwj][dwk][dwl]/metric;
#else						
						ptdw.x=DW->Log_plus.xkl[dwi][dwj][dwk]/metric;
						ptdw.y=DW->Log_plus.ykl[dwi][dwj][dwk]/metric;
						ptdw.z=0*DW->Log_plus.Skl[dwi][dwj][dwk]/metric;
#endif
						DWTrajplus.points.push_back(ptdw);
					}
#if OMNIDRIVE
					ptdw.x=DW->Log_plus.xkl[N_KL-2][dwj][dwk][dwl]/metric;
					ptdw.y=DW->Log_plus.ykl[N_KL-2][dwj][dwk][dwl]/metric;
					ptdw.z=0*DW->Log_plus.Skl[N_KL-2][dwj][dwk][dwl]/metric;
#else
					ptdw.x=DW->Log_plus.xkl[N_KL-2][dwj][dwk]/metric;
					ptdw.y=DW->Log_plus.ykl[N_KL-2][dwj][dwk]/metric;
					ptdw.z=0*DW->Log_plus.Skl[N_KL-2][dwj][dwk]/metric;
#endif
					DWTrajplus.points.push_back(ptdw);
#if OMNIDRIVE
					ptdw.x=DW->Log_plus.xkl[N_KL-1][dwj][dwk][dwl]/metric;
					ptdw.y=DW->Log_plus.ykl[N_KL-1][dwj][dwk][dwl]/metric;
					ptdw.z=0*DW->Log_plus.Skl[N_KL-1][dwj][dwk][dwl]/metric;
#else
					ptdw.x=DW->Log_plus.xkl[N_KL-1][dwj][dwk]/metric;
					ptdw.y=DW->Log_plus.ykl[N_KL-1][dwj][dwk]/metric;
					ptdw.z=0*DW->Log_plus.Skl[N_KL-1][dwj][dwk]/metric;
#endif
					DWTrajplus.points.push_back(ptdw);
					}
					//iscrtaj DWTrajektoriju
#if ROT
#if OMNIDRIVE
					if ((DW->TB_plus1.flag[dwj][dwk][dwk] == CLEAR)) //||(DW->TB.flag[dwj][dwk] == HAS_OBSTACLE))
#else
					if ((DW->TB_plus1.flag[dwj][dwk] == CLEAR)) //||(DW->TB.flag[dwj][dwk] == HAS_OBSTACLE))
#endif
					{
					for(int dwi=0;dwi<N_KL-1;dwi++)
					{
#if OMNIDRIVE
						ptdw.x=DW->Log_plus1.xkl[dwi][dwj][dwk][dwl]/metric;
						ptdw.y=DW->Log_plus1.ykl[dwi][dwj][dwk][dwl]/metric;
						ptdw.z=0*DW->Log_plus1.Skl[dwi][dwj][dwk][dwl]/metric;
#else						
						ptdw.x=DW->Log_plus1.xkl[dwi][dwj][dwk]/metric;
						ptdw.y=DW->Log_plus1.ykl[dwi][dwj][dwk]/metric;
						ptdw.z=0*DW->Log_plus1.Skl[dwi][dwj][dwk]/metric;
#endif
						DWTrajplus1.points.push_back(ptdw);
					}
#if OMNIDRIVE
					ptdw.x=DW->Log_plus1.xkl[N_KL-2][dwj][dwk][dwl]/metric;
					ptdw.y=DW->Log_plus1.ykl[N_KL-2][dwj][dwk][dwl]/metric;
					ptdw.z=0*DW->Log_plus1.Skl[N_KL-2][dwj][dwk][dwl]/metric;
#else
					ptdw.x=DW->Log_plus1.xkl[N_KL-2][dwj][dwk]/metric;
					ptdw.y=DW->Log_plus1.ykl[N_KL-2][dwj][dwk]/metric;
					ptdw.z=0*DW->Log_plus1.Skl[N_KL-2][dwj][dwk]/metric;
#endif
					DWTrajplus1.points.push_back(ptdw);
#if OMNIDRIVE
					ptdw.x=DW->Log_plus1.xkl[N_KL-1][dwj][dwk][dwl]/metric;
					ptdw.y=DW->Log_plus1.ykl[N_KL-1][dwj][dwk][dwl]/metric;
					ptdw.z=0*DW->Log_plus1.Skl[N_KL-1][dwj][dwk][dwl]/metric;
#else
					ptdw.x=DW->Log_plus1.xkl[N_KL-1][dwj][dwk]/metric;
					ptdw.y=DW->Log_plus1.ykl[N_KL-1][dwj][dwk]/metric;
					ptdw.z=0*DW->Log_plus1.Skl[N_KL-1][dwj][dwk]/metric;
#endif
					DWTrajplus1.points.push_back(ptdw);
					}
#if OMNIDRIVE
					if ((DW->TB_plus2.flag[dwj][dwk][dwk] == CLEAR)) //||(DW->TB.flag[dwj][dwk] == HAS_OBSTACLE))
#else
					if ((DW->TB_plus2.flag[dwj][dwk] == CLEAR)) //||(DW->TB.flag[dwj][dwk] == HAS_OBSTACLE))
#endif
					{
					for(int dwi=0;dwi<N_KL-1;dwi++)
					{
#if OMNIDRIVE
						ptdw.x=DW->Log_plus2.xkl[dwi][dwj][dwk][dwl]/metric;
						ptdw.y=DW->Log_plus2.ykl[dwi][dwj][dwk][dwl]/metric;
						ptdw.z=0*DW->Log_plus2.Skl[dwi][dwj][dwk][dwl]/metric;
#else						
						ptdw.x=DW->Log_plus2.xkl[dwi][dwj][dwk]/metric;
						ptdw.y=DW->Log_plus2.ykl[dwi][dwj][dwk]/metric;
						ptdw.z=0*DW->Log_plus2.Skl[dwi][dwj][dwk]/metric;
#endif
						DWTrajplus2.points.push_back(ptdw);
					}
#if OMNIDRIVE
					ptdw.x=DW->Log_plus2.xkl[N_KL-2][dwj][dwk][dwl]/metric;
					ptdw.y=DW->Log_plus2.ykl[N_KL-2][dwj][dwk][dwl]/metric;
					ptdw.z=0*DW->Log_plus2.Skl[N_KL-2][dwj][dwk][dwl]/metric;
#else
					ptdw.x=DW->Log_plus2.xkl[N_KL-2][dwj][dwk]/metric;
					ptdw.y=DW->Log_plus2.ykl[N_KL-2][dwj][dwk]/metric;
					ptdw.z=0*DW->Log_plus2.Skl[N_KL-2][dwj][dwk]/metric;
#endif
					DWTrajplus2.points.push_back(ptdw);
#if OMNIDRIVE
					ptdw.x=DW->Log_plus2.xkl[N_KL-1][dwj][dwk][dwl]/metric;
					ptdw.y=DW->Log_plus2.ykl[N_KL-1][dwj][dwk][dwl]/metric;
					ptdw.z=0*DW->Log_plus2.Skl[N_KL-1][dwj][dwk][dwl]/metric;
#else
					ptdw.x=DW->Log_plus2.xkl[N_KL-1][dwj][dwk]/metric;
					ptdw.y=DW->Log_plus2.ykl[N_KL-1][dwj][dwk]/metric;
					ptdw.z=0*DW->Log_plus2.Skl[N_KL-1][dwj][dwk]/metric;
#endif
					DWTrajplus2.points.push_back(ptdw);
					}
#endif
					//iscrtaj DWTrajektoriju
#if OMNIDRIVE
					if ((DW->TB_minus.flag[dwj][dwk][dwk] == CLEAR)) //||(DW->TB.flag[dwj][dwk] == HAS_OBSTACLE))
#else
					if ((DW->TB_minus.flag[dwj][dwk] == CLEAR)) //||(DW->TB.flag[dwj][dwk] == HAS_OBSTACLE))
#endif
					{
					for(int dwi=0;dwi<N_KL-1;dwi++)
					{
#if OMNIDRIVE
						ptdw.x=DW->Log_minus.xkl[dwi][dwj][dwk][dwl]/metric;
						ptdw.y=DW->Log_minus.ykl[dwi][dwj][dwk][dwl]/metric;
						ptdw.z=0*DW->Log_minus.Skl[dwi][dwj][dwk][dwl]/metric;
#else						
						ptdw.x=DW->Log_minus.xkl[dwi][dwj][dwk]/metric;
						ptdw.y=DW->Log_minus.ykl[dwi][dwj][dwk]/metric;
						ptdw.z=0*DW->Log_minus.Skl[dwi][dwj][dwk]/metric;
#endif
						DWTrajminus.points.push_back(ptdw);
					}
#if OMNIDRIVE
					ptdw.x=DW->Log_minus.xkl[N_KL-2][dwj][dwk][dwl]/metric;
					ptdw.y=DW->Log_minus.ykl[N_KL-2][dwj][dwk][dwl]/metric;
					ptdw.z=0*DW->Log_minus.Skl[N_KL-2][dwj][dwk][dwl]/metric;
#else
					ptdw.x=DW->Log_minus.xkl[N_KL-2][dwj][dwk]/metric;
					ptdw.y=DW->Log_minus.ykl[N_KL-2][dwj][dwk]/metric;
					ptdw.z=0*DW->Log_minus.Skl[N_KL-2][dwj][dwk]/metric;
#endif
					DWTrajminus.points.push_back(ptdw);
#if OMNIDRIVE
					ptdw.x=DW->Log_minus.xkl[N_KL-1][dwj][dwk][dwl]/metric;
					ptdw.y=DW->Log_minus.ykl[N_KL-1][dwj][dwk][dwl]/metric;
					ptdw.z=0*DW->Log_minus.Skl[N_KL-1][dwj][dwk][dwl]/metric;
#else
					ptdw.x=DW->Log_minus.xkl[N_KL-1][dwj][dwk]/metric;
					ptdw.y=DW->Log_minus.ykl[N_KL-1][dwj][dwk]/metric;
					ptdw.z=0*DW->Log_minus.Skl[N_KL-1][dwj][dwk]/metric;
#endif
					DWTrajminus.points.push_back(ptdw);
					}
					//iscrtaj DWTrajektoriju
#if OMNIDRIVE
					if ((DW->TB_minus2.flag[dwj][dwk][dwk] == CLEAR)) //||(DW->TB.flag[dwj][dwk] == HAS_OBSTACLE))
#else
					if ((DW->TB_minus2.flag[dwj][dwk] == CLEAR)) //||(DW->TB.flag[dwj][dwk] == HAS_OBSTACLE))
#endif
					{
					for(int dwi=0;dwi<N_KL-1;dwi++)
					{
#if OMNIDRIVE
						ptdw.x=DW->Log_minus2.xkl[dwi][dwj][dwk][dwl]/metric;
						ptdw.y=DW->Log_minus2.ykl[dwi][dwj][dwk][dwl]/metric;
						ptdw.z=0*DW->Log_minus2.Skl[dwi][dwj][dwk][dwl]/metric;
#else						
						ptdw.x=DW->Log_minus2.xkl[dwi][dwj][dwk]/metric;
						ptdw.y=DW->Log_minus2.ykl[dwi][dwj][dwk]/metric;
						ptdw.z=0*DW->Log_minus2.Skl[dwi][dwj][dwk]/metric;
#endif
						DWTrajminus2.points.push_back(ptdw);
					}
#if OMNIDRIVE
					ptdw.x=DW->Log_minus2.xkl[N_KL-2][dwj][dwk][dwl]/metric;
					ptdw.y=DW->Log_minus2.ykl[N_KL-2][dwj][dwk][dwl]/metric;
					ptdw.z=0*DW->Log_minus2.Skl[N_KL-2][dwj][dwk][dwl]/metric;
#else
					ptdw.x=DW->Log_minus2.xkl[N_KL-2][dwj][dwk]/metric;
					ptdw.y=DW->Log_minus2.ykl[N_KL-2][dwj][dwk]/metric;
					ptdw.z=0*DW->Log_minus2.Skl[N_KL-2][dwj][dwk]/metric;
#endif
					DWTrajminus2.points.push_back(ptdw);
#if OMNIDRIVE
					ptdw.x=DW->Log_minus2.xkl[N_KL-1][dwj][dwk][dwl]/metric;
					ptdw.y=DW->Log_minus2.ykl[N_KL-1][dwj][dwk][dwl]/metric;
					ptdw.z=0*DW->Log_minus2.Skl[N_KL-1][dwj][dwk][dwl]/metric;
#else
					ptdw.x=DW->Log_minus2.xkl[N_KL-1][dwj][dwk]/metric;
					ptdw.y=DW->Log_minus2.ykl[N_KL-1][dwj][dwk]/metric;
					ptdw.z=0*DW->Log_minus2.Skl[N_KL-1][dwj][dwk]/metric;
#endif
					DWTrajminus2.points.push_back(ptdw);
					}
#endif
				}
			}
			}

			DWTrajektorije_pubplus.publish(DWTrajplus);
			DWTrajektorije_pubplus1.publish(DWTrajplus1);
			DWTrajektorije_pubplus2.publish(DWTrajplus2);
			DWTrajektorije_pub.publish(DWTraj);
			DWTrajektorije_pubminus.publish(DWTrajminus);
			DWTrajektorije_pubminus2.publish(DWTrajminus2);


			//Trajektorije pokretne prepreke
			for(int ppj=0;ppj<DW->broj_pokretnih_prepreka;ppj++)
			{
				//iscrtaj trajektorije pokretne prepreke
				for(int ppk=0;ppk<=N_KL/DEC_KL_DRAW;ppk++)
				{
					geometry_msgs::Point pt;
					pt.x=DW->MO[ppj].x[ppk*DEC_KL_DRAW]/metric;
					pt.y=DW->MO[ppj].y[ppk*DEC_KL_DRAW]/metric;
					pt.z=0;
					TrajPP.points.push_back(pt);	
				}
				
				TrajPP_pub.publish(TrajPP);
			}  
  }  
  

  void GenericLaserScanFilterNode::sink_call(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& odom, const sensor_msgs::LaserScan::ConstPtr& msg)
  {
	  	  std::cout << "textEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE";

//    tf::StampedTransform transform;
	  tf_listener.waitForTransform("/base_link",msg->header.frame_id, msg->header.stamp, ros::Duration(5));
    tf_listener.lookupTransform("/base_link",msg->header.frame_id, ros::Time(0), transform_laser);
 
    double yaw, tfx, tfy, pitch, roll;
    transform_laser.getBasis().getRPY(roll, pitch, yaw);
	  tfx=transform_laser.getOrigin().x();
	  tfy=transform_laser.getOrigin().y();

    printf("\ntransform base_link -> base_laser_link: x [%f] y [%f] th [%f] [deg]\n",tfx,tfy,yaw*180/M_PI );

    tf::Transform transform_auxiliar;
    transform_auxiliar.setOrigin(tf::Vector3(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z));
    transform_auxiliar.setRotation(tf::Quaternion( odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w));

    tf::Transform transform_baselink=transform_auxiliar*transform_laser;
    transform_baselink.getBasis().getRPY(roll, pitch, yaw);
	  tfx=transform_baselink.getOrigin().x();
	  tfy=transform_baselink.getOrigin().y();

    printf("\ntransform to laser_link: x [%f] y [%f] th [%f] [deg]\n",tfx,tfy,yaw*180/M_PI );

	  lasertfx=tfx;
    lasertfy=tfy;
    lasertfth=yaw;

	  robottfx=odom->pose.pose.position.x;
 	  robottfy=odom->pose.pose.position.y;
 	  robottfth=tf::getYaw(odom->pose.pose.orientation);
	  
    printf("\ncoordinates of base_link: x [%f] y [%f] th [%f] [deg]\n",robottfx,robottfy,robottfth*180/M_PI );
	  

  //save it to global variable along with ranges
  //use it later to paint into map
//	laser1stamp=msg->header.stamp;
	lasercount=msg->ranges.size();
	angle_min=msg->angle_min;
	angle_increment=msg->angle_increment;
	R_point lok;
	for(int i_LS=0;i_LS<lasercount;i_LS++){
                ranges[i_LS]=msg->ranges[i_LS];
                //filtering too small laser readings
		if ((ranges[i_LS]<0.1)|| (i_LS<15) || (i_LS>msg->ranges.size()-15))
			ranges[i_LS]=10.;
			  lok.x=ranges[i_LS]*cos(angle_min+i_LS*angle_increment);
			  lok.y=ranges[i_LS]*sin(angle_min+i_LS*angle_increment);
			  lok.th=angle_min+i_LS*angle_increment;
				glob[i_LS].x=WH->Lokalna_u_globalnu_x(tfx, yaw,lok.x,lok.y);
				glob[i_LS].y=WH->Lokalna_u_globalnu_y(tfy, yaw,lok.x,lok.y);
				glob[i_LS].th=lok.th;
	}

  }

void GenericLaserScanFilterNode::globalposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    	robottfx=msg->pose.pose.position.x;
    	robottfy=msg->pose.pose.position.y;
    	robottfth=tf::getYaw(msg->pose.pose.orientation);


	printf("\nI heard: x [%f], y [%f] and th [%f] deg\n", robottfx, robottfy, robottfth);
}


void GenericLaserScanFilterNode::odomCallback(const nav_msgs::OdometryConstPtr& odom)
{

	v=odom->twist.twist.linear.x;
	vy=odom->twist.twist.linear.y;
	omega=odom->twist.twist.angular.z;

	printf("\nI heard: v [%f], vy [%f] and w [%f]\n", v, vy, omega);
}

void GenericLaserScanFilterNode::movingRobot0Callback(const movingobstaclesrhc::MovingConstPtr& robot0Msg)
{
	robotX = robot0Msg->x;
	robotY = robot0Msg->y;
	robotTH = robot0Msg->th;
	robotW = robot0Msg->w;
	robotV = robot0Msg->v;
	ROS_INFO("\n\n\n\n\n\nPRIMIO SAM OD ROBOTA1 NJEGOVU X: %f \n\n\n\n\n\n",robotX);
}

  void GenericLaserScanFilterNode::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
   tf::StampedTransform transform;
   tf_listener.waitForTransform("map",msg->header.frame_id, msg->header.stamp, ros::Duration(0.05));
   tf_listener.lookupTransform("map", msg->header.frame_id, msg->header.stamp, transform);
 
          double yaw, pitch, roll, tfx, tfy;
          transform.getBasis().getRPY(roll, pitch, yaw);
	  tfx=transform.getOrigin().x();
	  tfy=transform.getOrigin().y();
    printf("transform front: x [%f] y [%f] th [%f] [deg]\n",tfx,tfy,yaw*180/M_PI );
    lasertfx=tfx;
    lasertfy=tfy;
    lasertfth=yaw;
  //now we have the transformation from /map to /laserFront_frame in "transform"
  //save it to global variable along with ranges
  //use it later to paint into map
	laser1stamp=msg->header.stamp;
	lasercount=msg->ranges.size();
	angle_min=msg->angle_min;
	angle_increment=msg->angle_increment;
	R_point lok;
	for(int i_LS=0;i_LS<msg->ranges.size();i_LS++){
#if LASER_INVERTED
        ranges[i_LS]=msg->ranges[lasercount-1-i_LS];
#else
        ranges[i_LS]=msg->ranges[i_LS];
#endif
                //filtering too small laser readings and border readings
			  lok.th=angle_min+i_LS*angle_increment-sickAngularOffset; //sick2 is rotated for pi/2
			  if (((lok.th<-LASER_MIN_MAX_ANGLE*M_PI/180.)&&(1||ranges[i_LS]<0.5)) || ((lok.th>LASER_MIN_MAX_ANGLE*M_PI/180.)&&(1||ranges[i_LS]<0.5)) || (ranges[i_LS]<0.05)) //adapted for sick2 and p3at
			    ranges[i_LS]=10.;
			  lok.x=ranges[i_LS]*cos(angle_min+i_LS*angle_increment);
			  lok.y=ranges[i_LS]*sin(angle_min+i_LS*angle_increment);
				glob[i_LS].x=WH->Lokalna_u_globalnu_x(tfx, yaw,lok.x,lok.y);
				glob[i_LS].y=WH->Lokalna_u_globalnu_y(tfy, yaw,lok.x,lok.y);
				glob[i_LS].th=lok.th; 
	}


  }

  // Callback
  void GenericLaserScanFilterNode::laserRearCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
   tf::StampedTransform transform;
   tf_listener.lookupTransform("/map", msg->header.frame_id, msg->header.stamp, transform);
 
          double yaw, pitch, roll, tfx, tfy;
          transform.getBasis().getRPY(roll, pitch, yaw);
	  tfx=transform.getOrigin().x();
	  tfy=transform.getOrigin().y();
    printf("transform rear: x [%f] y [%f] th [%f] [deg]\n",tfx,tfy,yaw*180/M_PI );
    lasertfx2=tfx;
    lasertfy2=tfy;
    lasertfth2=yaw;
  //now we have the transformation from /map to /laserFront_frame in "transform"
  //save it to global variable along with ranges
  //use it later to paint into map
	laser2stamp=msg->header.stamp;
	lasercount2=msg->ranges.size();
	angle_min2=msg->angle_min;
	angle_increment2=msg->angle_increment;
	R_point lok;
	for(int i_LS=0;i_LS<msg->ranges.size();i_LS++){
                ranges2[i_LS]=msg->ranges[i_LS];
                //filtering too small laser readings
		if ((ranges2[i_LS]<0.1)|| (i_LS<15) || (i_LS>msg->ranges.size()-15))
			ranges2[i_LS]=10.;
			  lok.x=ranges2[i_LS]*cos(angle_min2+i_LS*angle_increment2);
			  lok.y=ranges2[i_LS]*sin(angle_min2+i_LS*angle_increment2);
			  lok.th=angle_min2+i_LS*angle_increment2;
				glob2[i_LS].x=WH->Lokalna_u_globalnu_x(tfx, yaw,lok.x,lok.y);
				glob2[i_LS].y=WH->Lokalna_u_globalnu_y(tfy, yaw,lok.x,lok.y);
				glob2[i_LS].th=lok.th;
	}


  }
bool GenericLaserScanFilterNode::ReqRobotStop(movingobstaclesrhc::ReqRobStop::Request &req, movingobstaclesrhc::ReqRobStop::Response &res){
//	slam_msgs::active_slam active_m;  	
	if (req.stop){
//		 active_m.comm=1000;
  		 M->voznja=false;
  		 WH->processState=HALT;
  		 WH->no_path_counter=0;
  		 M->novicilj=true;
  		 M->slam_loop=1;
  		 M->old_goal=M->goal;
  		 res.agreed=1;
//		 active_m.loop_id=req.loop_id;
  	}else{
//  	  ROS_ERROR("MINA %d koordinate (%f,%f)",req.ismine,req.x,req.y);
  	  if (req.ismine){
  	    GM->mapper_point_temp.x=req.x*M->metric;
  	    GM->mapper_point_temp.y=req.y*M->metric;
  	    GM->mapper_point_temp.th=0;
  	    if (GM->check_point(GM->mapper_point_temp)){
  	    int okolina=2;
				for (int ix=GM->cell_point_temp.x-okolina;ix<=GM->cell_point_temp.x+okolina;ix++){
					for (int jy=GM->cell_point_temp.y-okolina;jy<=GM->cell_point_temp.y+okolina;jy++){
						if((ix>=0)&&(ix<GM->Map_Dim_X)&&(jy>=0)&&(jy<GM->Map_Dim_Y)){
  	          GM->Map[ix][jy].static_cell=true;
      	      GM->Map[ix][jy].occupancy=3;
      	      GM->Map[ix][jy].x=GM->mapper_point_temp.x;
					    GM->Map[ix][jy].y=GM->mapper_point_temp.y;
//      	      ROS_ERROR("UPIS MINE U MAPU (%d,%d)",ix,jy);
  	        }
          }
  	    }
  	    }
  	  }
  		M->slam_loop=0;
  		M->gotogoal(M->old_goal);
  		res.agreed=1;
//		active_m.comm=4000;
		
  	}
//	active_pub.publish(active_m);
  	return true;
  }

void GenericLaserScanFilterNode::simple_goal(const geometry_msgs::PoseStamped::ConstPtr& simple_g){
	R_point tem;
	tem.x = simple_g->pose.position.x*1000;
	tem.y = simple_g->pose.position.y*1000;
	tem.th = tf::getYaw(simple_g->pose.orientation);
	ROS_INFO("RECEIVEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEDDDDDDDDDDD::::: %f %f %f deg",tem.x,tem.y,tem.th*RuS);
	M->gotogoal(tem);
}

void moj::executeMotion(){

	FILE	*logfile, *goalfile, *F;
	double tfx, tfy, yaw;

#if 1 //stari nacin
	  tf::StampedTransform transform;
          double pitch, roll;
	  try{
      if ((WH->processState==RESUMING)){
        printf("\nwaiting for transform\n");
#if (ROBOT>=1)
//        tf_listener.waitForTransform("map", "Pioneer3AT/base_link",ros::Time(0), ros::Duration(1.0));//robot_0/now
//        tf_listener.lookupTransform("map", "Pioneer3AT/base_link", ros::Time(0), transform);//robot_0/
        tf_listener.waitForTransform("map", base_frame_,ros::Time(0), ros::Duration(1.0));//robot_0/now
        tf_listener.lookupTransform("map", base_frame_, ros::Time(0), transform);//robot_0/
#else
        tf_listener.waitForTransform("map", "/robot_0/base_link",ros::Time(0), ros::Duration(1.0));//robot_0/now
        tf_listener.lookupTransform("map", "/robot_0/base_link", ros::Time(0), transform);//robot_0/
#endif
      }else{
#if (ROBOT>=1)
//        tf_listener.lookupTransform("map", "Pioneer3AT/base_link", ros::Time(0), transform);robot_0/
        tf_listener.lookupTransform("map", base_frame_, ros::Time(0), transform);//robot_0/
        if ((transform.stamp_-laser1stamp).toSec()>0.1 && (transform.stamp_-laser2stamp).toSec()>0.1){
          printf("AaaAAAaaaAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA!\n");
//        std::cerr<<(transform.stamp_-laser1stamp).toSec()<<std::endl;
//        std::cerr<<(transform.stamp_-laser2stamp).toSec()<<std::endl;
//        std::cerr<<(transform.stamp_).toSec()<<std::endl;
//        std::cerr<<(laser2stamp).toSec()<<std::endl;
        }
#else
        listener.lookupTransform("map", "/robot_0/base_link", ros::Time(0), transform);//robot_0/
#endif
      }
	  }
	  catch (tf::TransformException ex){
		  ROS_ERROR("%s",ex.what());
	  }
          transform.getBasis().getRPY(roll, pitch, yaw);
	  tfx=transform.getOrigin().x();
	  tfy=transform.getOrigin().y();
	printf("\n map to base_link: x [%f], y [%f] and th [%f] deg\n", tfx, tfy, yaw*180/M_PI);

#else
	  tfx=robottfx;
    tfy=robottfy;
    yaw=robottfth;
#endif


    ciklus++;
	  printf("\n cycle no. %d\n",ciklus);
    
    if (ciklus>10){
#if (IDEAL_MODEL==1)
		if (ciklus==11){//samo prvi ciklus uzimamo ocitanja, ostalo iz DW na kraju petlje
		RB.x=tfx*metric;//1350.;//tfx*metric;//1320.;//6400.;//9340.;//6460.;//6401.;//5499.;//800.;//12650.;//1250.;//1950.;//2250.;//500.;//
		  RB.y=tfy*metric;//2350.;//tfy*metric;//2310.;//3100.;//1504.;//3100.;//1400.;//3801.;//5900.;//1300.;//16400.;//5300.;//1000.;
		  RB.th=yaw;//-M_PI/2.;//yaw;//M_PI*7/8.;//-M_PI/2.-0.008;//M_PI/7.-M_PI/2.-0.008;//M_PI/2.+0.1;//-M_PI/2.0;//1.135;3.7977;//1.57;//3.14;//  //radijani
		  RB.v=v*metric; //mm ovo daje odomcallback
		  RB.vy=vy*metric;
		  RB.w=omega;
		}
#else
		  RB.x=tfx*metric;
		  RB.y=tfy*metric;
		  while (yaw>=2*M_PI) yaw-=2*M_PI;
		  while (yaw<0) yaw+=2*M_PI;
		  RB.th=yaw;  //radijani
		  RB.v=v*metric; //mm ovo daje odomcallback
		  RB.vy=vy*metric;
		  RB.w=omega;
#endif

#if (NO_LASER==1)
		  lasercount=0;
 		  lasercount2=0;
#endif

//laser origins!!!
//first laser
  WH->laserx=lasertfx*metric;//now it is in global k.s.
  WH->lasery=lasertfy*metric;
  WH->laserth=lasertfth;
//second laser
  WH->laser2x=lasertfx2*metric;
  WH->laser2y=lasertfy2*metric;
  WH->laser2th=lasertfth2*metric;
		  int i=0;
		  printf("angle_min=%f angle_increment=%f angle_max=%f, lasertfth=%f, lasercount=%d\n",angle_min, angle_increment, angle_min+(lasercount-1)*angle_increment,lasertfth,lasercount);//prema ispisu sicklms100 daje kut od -135 do 135 uvijek, dakle treba mu dodati ofset istog predznaka!
		  for (int j=0; j<lasercount; j++) 
		  {
        LB.point[j].x=glob[j].x*metric;
        LB.point[j].y=glob[j].y*metric;
			  LB.scan[j].r=ranges[j]*metric;
			  LB.scan[j].th=glob[j].th;
			  if (((j % 1) == 0)&&(LB.scan[j].r<LASER_RANGE_MAX))             //svako mjerenje koje je manje od 3 metra
			  {//noj
				  mali.point[i].x=LB.point[j].x;
				  mali.point[i].y=LB.point[j].y;
				  mali.scan[i].r=LB.scan[j].r;
				  mali.scan[i].th=LB.scan[j].th;
				  i++;
			  } // end if

		  }// end for j ocitanja lasera
		  LB.scan_count=lasercount;
		  mali.scan_count=i;

//second laser
		  i=0;
		  printf("angle_min2=%f angle_increment2=%f angle_max2=%f, lasertfth2=%f\n",angle_min2, angle_increment2, angle_min2+(lasercount-1)*angle_increment2,lasertfth2);
		  for (int j=0; j<lasercount2; j++) 
		  {
        LB2.point[j].x=glob2[j].x*metric;
        LB2.point[j].y=glob2[j].y*metric;
			  LB2.scan[j].r=ranges2[j]*metric;
			  LB2.scan[j].th=glob2[j].th;
			  if (((j % 1) == 0)&&(LB2.scan[j].r<LASER_RANGE_MAX))             //svako mjerenje koje je manje od 3 metra
			  {//noj
				  mali2.point[i].x=LB2.point[j].x;
				  mali2.point[i].y=LB2.point[j].y;
				  mali2.scan[i].r=LB2.scan[j].r;
				  mali2.scan[i].th=LB2.scan[j].th;
				  i++;
			  } // end if

		  }// end for j ocitanja lasera
		  LB2.scan_count=lasercount2;
		  mali2.scan_count=i;
#if 0
		  if(snimanje){
			 slam_msgs::NewAugment servNewAugment;
			 servNewAugment.request.new_state=1;
			 if (client_augment.call(servNewAugment)){
			  	if(servNewAugment.response.sent){
			  		snimanje = 0;
			  		ROS_INFO("Measuring state augmentation successfull!");
			 	}else{
			 		ROS_ERROR("State not augmented");
			  	}
			  }else{
				  ROS_ERROR("Failed to call service NewAugment");
			  }
		  }
#endif
                 if (((ciklus % 10)==0)&&(voznja==false)){
			 int dobar,offset;
			 double x,y,th;
			 th=0;
			 char rdLine[36]="";
			 char *line;
			 char *word;
#if 0
			 if (slam_loop){
				 slam_msgs::active_slam active_m;
				 slam_msgs::NewStateReq servNewState;
				 servNewState.request.new_state=1;
				 if (client.call(servNewState)){
					active_m.comm=1;
				 	if(servNewState.response.sent){
						active_m.sent=1;				 		
						R_point tem;
				 		tem.x=servNewState.response.x;
				 		tem.y=servNewState.response.y;
				 		tem.th=servNewState.response.theta;
						active_m.x=tem.x;
						active_m.y=tem.y;
						active_m.th=tem.th;
						active_m.loop_id = servNewState.response.loop_id;
						printf("Coordinates: %f %f %f\n",tem.x,tem.y,tem.th);
						ROS_INFO("New state coordinates confirmed!!");				 		
						gotogoal(tem);

				 	}else{
				 		ROS_ERROR("Active SLAM failure, please restart!!!!");
				 	}
				 }else{
					active_m.comm = 0;
				 	ROS_ERROR("Failed to call service NewStateReq");
				 }
				active_pub.publish(active_m);			 
			}
#endif			
			 if ( (logfile = fopen("mirko","a")) == NULL ){
				 printf("Error! mirko file couldn't be opened.");
			 }else{

			 
			 if ( (goalfile = fopen("koordinate.txt","r+")) == NULL ){
				 printf("Error! goalfile couldn't be opened.");
			 }else if(slam_loop){
				printf("HOCU CITAT A NE SMIJEM, SRANJE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
			 }else{
// 				 fprintf(logfile,"pokazivac=%ld\n",ftell(goalfile));
				 offset=ftell(goalfile);
				 while (fgets(rdLine,35,goalfile) != NULL)
				 {
// 					 fprintf(logfile,"pokazivac=%ld\n",ftell(goalfile));
					 offset=ftell(goalfile)-offset;
					 line=&rdLine[0];
					 if (line[0] == ';' || line[0] == '#' || line[0] == '\n')
					 {
 						 fprintf(logfile,"komentar ili prazan red\n");
						 offset=ftell(goalfile);
						 continue;
					 }
					 if (1){//s ip
					 word = strtok (line,">");
					 if (word != NULL){
						 fprintf(logfile,"ip adresa %s\n",word);
					 }
					 word = strtok (NULL," ");
					 if (word == NULL){
						 offset=ftell(goalfile);
						 continue;
					 }
					 }else{
						 word = strtok (line," ");
					 }
					 dobar=0;
					 if (word != NULL){
						 x=atof(word);
//						 fprintf(logfile,"x je %s\n",word);
						 word = strtok (NULL," ");
//					 	 fprintf(logfile,"y je %s\n",word);
						 if (word!= NULL){
						 	y=atof(word);
						 	word=strtok(NULL," \n");
						 	if (word != NULL){
							 th=atof(word);
							 dobar=1;
							 fseek(goalfile,-1*offset,SEEK_CUR);
// 							 fprintf(logfile,"th pokazivac=%ld\n",ftell(goalfile));
 							 fputc('#',goalfile);
							 fseek(goalfile,offset-1,SEEK_CUR);
// 							 fprintf(logfile,"th pokazivac=%ld\n",ftell(goalfile));
						 	}else{
						 	 dobar=1;
							 fseek(goalfile,-1*offset,SEEK_CUR);
// 							 fprintf(logfile,"y pokazivac=%ld\n",ftell(goalfile));
 							 fputc('#',goalfile);
							 fseek(goalfile,offset-1,SEEK_CUR);
// 							 fprintf(logfile,"y pokazivac=%ld\n",ftell(goalfile));
						 	}
						 	
						 }
					 }
					 fprintf(logfile,"x=%f, y=%f, th=%f, dobar=%d\n",x,y,th,dobar);
					 offset=ftell(goalfile);
					 if (dobar==1){
						 voznja=true;
						 th=th*SuR;
#if RECTANGULAR
//-------------determing orientation of the goal ---------------------
//real coordinates within a cell
	int orientation;
	while (th>=2*M_PI) th-=2*M_PI;
	while (th<0) th+=2*M_PI;
	orientation=cspace->worldToMapTheta(th);
	GM->mapper_point_temp.x=x;
	GM->mapper_point_temp.y=y;
	if((GM->check_point(GM->mapper_point_temp))) {
	IntPose checkme(GM->cell_point_temp.x, GM->cell_point_temp.y,orientation);
	
	if (cspace->checkCollision(checkme)){
		printf("M: goal orientation collides %f deg\n",th*RuS);
		OrientationIntervals ints;
    		ints= cspace->getAdmissibleOrientationStructure(GM->cell_point_temp.x, GM->cell_point_temp.y);
    		std::cerr<<ints.print()<<std::endl;
       		
	std::cout<<"at "<<GM->cell_point_temp.x<<","<<GM->cell_point_temp.y<<" orientation "<<orientation<<" is contained: "<<(ints.containsOrientation(orientation)?"yes":"no")<<std::endl;
		int tempOrientation=orientation;
	    	int minD=ints.getMaxOrientation()+1;
	    	for (int j=ints.getMinOrientation(); j<=ints.getMaxOrientation(); j++){
	    		if (ints.containsOrientation(j)){
	    			if ((abs(j-orientation)<minD)||(abs(j+ints.getMaxOrientation()+1-orientation)<minD)||(abs(j-ints.getMaxOrientation()-1-orientation)<minD)){
					minD=std::min(abs(j-orientation),std::min((abs(j+ints.getMaxOrientation()+1-orientation)),(abs(j-ints.getMaxOrientation()-1-orientation))));
					tempOrientation=j;
	    			}
	    		}
	    	}
	    	th=cspace->mapToWorldTheta(tempOrientation);
	    	printf("new orientation is %d (%f deg), max is %d\n", tempOrientation, th*RuS, ints.getMaxOrientation());

	}
	}
#endif						 
						 goal.x=x;
						 goal.y=y; 
						 goal.th=th;
						if ( (F = fopen("logger//newgoals","a")) == NULL ){
				 			printf("Error! komb file couldn't be opened.");
				 		}else{
				 			fprintf(F,"3 %f %f %f\n",x,y,th);
				 			fclose(F);
				 		}

						 break;
					 }
				 }

/*				 if(fscanf(goalfile,"%d %d\n",&x,&y)==2){
					printf("x=%d, y=%d\n",x,y);
				 }else{
				 }*/
				 fclose(goalfile);
			 }
			 fclose(logfile);
			 }
		}

		//----------------tu sam---------------------------------
		// 		printf("M> brzine v %f, w %f, x %f, y %f, th %f\n",M->RB.v,M->RB.w,M->RB.x,M->RB.y,M->RB.th);
		//postavljanje startne pozicije
		  if (ciklus==11) //jer se ceka gore da ih prodje 10, bilo je nula 0
		  {
			  printf("M->mali.scan_count=%d\n",mali.scan_count);
			  start.x=RB.x;
			  start.y=RB.y;
			  start.th=RB.th;
			  printf("M->start.x:%.3f\tM->start.y:%.3f\tgoal=(%.3f,%.3f)\n",start.x,start.y,goal.x,goal.y);
		  } // if ciklus = 11
		  if (voznja)
		  {
//			  printf("Usao sam u voznju\n");
			  if (novicilj){
				  novicilj=false;

				  subgoal=goal;
				  if (WH->processState==NO_PATH){
					  subgoal=WH->global_goal_workhorse; //postavljen u planneru na slobodnu
				  }
				  WH->new_global_goal=false;
				  printf("The new goal is (%f,%f,%f) and subgoal is (%f,%f,%f)\n",goal.x,goal.y,goal.th,subgoal.x,subgoal.y,subgoal.th);
				  zavrsio=false;

				 
			  }
        DW->ChangeVariables(robotX,robotY,robotTH,robotW,robotV);
			  //DW->Pokretna_prepreka(robotX,robotY,robotTH,robotW,robotV);
			  WH->process();//pozivanje procesa koji sve racuna
		          
		  }

//		  printf("setvel.v=%f, setvel.w=%f, setvel.vy=%f, x=%f, y=%f, th=%f (%d)\n",setvel.v*metric,setvel.w, setvel.vy*metric, RB.x, RB.y, RB.th, cspace->worldToMapTheta(RB.th));
		  if ((WH->processState==HALT)&&(WH->no_path_counter==0)){   //kad dodje do cilja udje u HALT mod
			  //WH->Logger();
			  voznja=false;//vise nejde
			
			  novicilj=true;
			  printf("Finished! My pose is (%f mm, %f mm, %f deg)\n",RB.x,RB.y,RB.th*RuS);

		  }
		  if (WH->processState==NO_PATH){//dabar
			  novicilj=true; //neka izracuna pomaknuti cilj
		  }

		 movingobstaclesrhc::Moving myMessage;
		 myMessage.x = RB.x;
		 myMessage.y = RB.y;
		 myMessage.th = RB.th;
		 myMessage.v = RB.v;
		 myMessage.w = RB.w;

		moving_pub.publish(myMessage);		

      geometry_msgs::Twist vel;
		  vel.linear.x = setvel.v;
		  vel.angular.z = setvel.w;//0.75;
		  vel.linear.y = setvel.vy;
		  

//testing omnidrive		  
//		  vel.linear.x = 0.;
//		  vel.linear.y = 0.;
//		  vel.angular.z = 0.;
//		  vel_pub.publish(vel);

#if (IDEAL_MODEL==0)
      if (voznja)
		      vel_pub.publish(vel);
#else
		if(DW->poceo){
		  RB.x=DW->SP.setpoint_x;
		  RB.y=DW->SP.setpoint_y;
		  RB.th=DW->SP.setpoint_th;
		  RB.v=DW->SP.setpoint_v;
		  RB.vy=DW->SP.setpoint_vy;
		  RB.w=DW->SP.setpoint_w;
		}
#endif
}

}

void moj::gotogoal(R_point newgoal){
  
  voznja=true;
  WH->processState=HALT;
  WH->no_path_counter=0;
  novicilj=true;

  I_point newgoal_i;
  if(!PL->RealToInt(newgoal, newgoal_i, GM->Map_Dim_X_A,GM->Map_Dim_Y_A, GM->Map_Home.x, GM->Map_Home.y, CELL_DIM)){
        printf("Planner> Goal is out of the map borders!\n");

     }




  goal=newgoal;
  DW->naCilju=false;

  FILE *F;
	if ( (F = fopen("logger//newgoals","a")) == NULL ){
		printf("Error! komb file couldn't be opened.");
	}else{
		fprintf(F,"3 %f %f %f\n",newgoal.x,newgoal.y,newgoal.th);
		fclose(F);
	}

}

//moj::moj(ros::NodeHandle n):GenericLaserScanFilterNode(n)
//moj::moj()
void moj::initializePlanner()
{

  boost::filesystem::create_directory( "logger" );
	goal.x=MAP_GOAL_POSITION_X;
	goal.y=MAP_GOAL_POSITION_Y;
	subgoal=goal;
	start.x=MAP_START_POSITION_X;
	start.y=MAP_START_POSITION_Y;
	start.th=MAP_START_POSITION_TH;
	wld=WORLD;
	LB.scan_count=0;
	LB2.scan_count=0;
	subgoal2flag=false;
	//initialization - not working like this
	    VY_MAX = vy_max;
      VX_MAX = vx_max;
      VX_MIN = vx_min;
      VY_MIN = vy_min;
      DVX_MAX = dvx_max;
      DVY_MAX = dvy_max;
      W_MAX = w_max;
      W_MIN = w_min;
      DW_MAX = dw_max;
      V_MAX = v_max;
      DV_MAX = dv_max;
      V_MIN = v_min;

  WH=new WorkHorse();
  slam_loop=0;
  snimanje=0;
  

//  nh=n;
//  visual=new VisualizationPublisher(nh);
//  VisualizationPublisher visual(nh);
//  GenericLaserScanFilterNode t(nh);

  ciklus=0;
  voznja=false;// ako je true onda ide do jednog cilja pa do drugog
  novicilj=true;
  zavrsio=false;
	metric=1000;//da bi bilo u milimetrima (za planiranje putanje)

  double mapwidth, mapheight;
double mapresolution, maporiginx, maporiginy;

#if LOADMAP
nav_msgs::GetMap map;
ros::service::call("static_map",map);
ROS_INFO("map data size %d",map.response.map.data.size());
printf("writing the map data: data size %d, res=%f, width=%d, height=%d, origin (%f,%f,%f)\n",map.response.map.data.size(), map.response.map.info.resolution, map.response.map.info.width, map.response.map.info.height, map.response.map.info.origin.position.x, map.response.map.info.origin.position.y, map.response.map.info.origin.position.z);
mapwidth=map.response.map.info.width;
mapheight=map.response.map.info.height;
mapresolution=map.response.map.info.resolution;
maporiginx=map.response.map.info.origin.position.x;
maporiginy=map.response.map.info.origin.position.y;

  WH->LoadInit(mapwidth, mapheight, mapresolution, maporiginx, maporiginy);//konstrukcija sveg (navigacijski modul)
  
  
#else

// nacin ucitavanja prazne mape preko parametara
  mapwidth=20.;
  mapheight=20.;
  nh_.getParam("/spoj/map_width", mapwidth);
  nh_.getParam("/spoj/map_height", mapheight);
  nh_.getParam("/spoj/map_originx", maporiginx);
  nh_.getParam("/spoj/map_originy", maporiginy);
    ROS_INFO("map width is %f m", mapwidth);
    ROS_INFO("map height is %f m", mapheight);
//    mapwidth+=2.;
//    mapheight+=2.;
  maporiginx=-mapwidth/2.;
  maporiginy=-mapheight/2.;
    if (maporiginx<-mapwidth+1. || maporiginx>-1.) maporiginx=-mapwidth/2;
    if (maporiginy<-mapheight+1. || maporiginy>-1.) maporiginy=-mapheight/2;
    ROS_INFO("map originx is %f m", maporiginx);
    ROS_INFO("map originy is %f m", maporiginy);
    
  mapresolution=1.;
  
  WH->LoadInit(mapwidth, mapheight, mapresolution, maporiginx, maporiginy);//konstrukcija sveg (navigacijski modul)

//upis prepreka na rubove
  int ti,tj;
	for (int i=0; i<GM->Map_Dim_X; i++){
		for (int j=0; j<2; j++){// j==0, j==1
      tj=j*(GM->Map_Dim_Y-1);
				if ((GM->Map[i][tj].occupancy==0)){
					GM->Map[i][tj].occupancy=GRID_MAP_OCCUPANCY_TRESHOLD + 1;
					GM->Map[i][tj].x=GM->Map_Home.x+i*GM->Map_Cell_Size+0.5*GM->Map_Cell_Size;
					GM->Map[i][tj].y=GM->Map_Home.y+tj*GM->Map_Cell_Size+0.5*GM->Map_Cell_Size;
					GM->Map[i][tj].static_cell=true;
				}
			
    }
  }

	for (int i=0; i<2; i++){//i==0, i==1
    ti=i*(GM->Map_Dim_X-1);
		for (int j=0; j<GM->Map_Dim_Y; j++){// j==0, j==1
				if ((GM->Map[ti][j].occupancy==0)){
					GM->Map[ti][j].occupancy=GRID_MAP_OCCUPANCY_TRESHOLD + 1;
					GM->Map[ti][j].x=GM->Map_Home.x+ti*GM->Map_Cell_Size+0.5*GM->Map_Cell_Size;
					GM->Map[ti][j].y=GM->Map_Home.y+j*GM->Map_Cell_Size+0.5*GM->Map_Cell_Size;
					GM->Map[ti][j].static_cell=true;
				}
			
    }
  }


#endif

FILE	*F;
#if LOADMAP
if ((F = fopen("gridmapaMap.dat","wt")) != NULL)
	{
		for (int j=0; j < mapwidth; j++)
		{
			for (int i=0; i < mapheight; i++)
				fprintf(F,"%d ",map.response.map.data[i*map.response.map.info.width+j]);
			fprintf(F,"\n");
		}
		fclose(F);
	}
	int ii,jj;
	for (int j=0; j < map.response.map.info.width; j++)
		{
			for (int i=0; i < map.response.map.info.height; i++){
				ii=(int)floor(j*map.response.map.info.resolution/GM->Map_Cell_Size*metric);
				jj=(int)floor(i*map.response.map.info.resolution/GM->Map_Cell_Size*metric);
				if ((GM->Map[ii][jj].occupancy==0)&&(map.response.map.data[i*map.response.map.info.width+j]>0)){
					GM->Map[ii][jj].occupancy=GRID_MAP_OCCUPANCY_TRESHOLD + 1;
					GM->Map[ii][jj].x=j*map.response.map.info.resolution*metric+GM->Map_Home.x;       //prema player-stage karti
					GM->Map[ii][jj].y=i*map.response.map.info.resolution*metric+GM->Map_Home.y;
					GM->Map[ii][jj].static_cell=true;
				}
			}
			
			
		}
#endif
//--------------------------------------------------------------------
	//create robot model "wheelchair"
	//note that heigt is given in number of layers
	int length = ROBOT_MASKY*2;//this must be integer?
	int width = ROBOT_MASK*2;
	int boxwidth=1; //3 for the experiment//3 for bigexp
	int boxlength=0;//9-width; //2//18 for the experiments //(9-width) for simulation paper//0 for bigexp
	int boxlength2=3;//10//11 for bigexp
	for (int i=0; i<15; i++){
	  footprintx[i]=-length*CELL_DIM/2.;
	  footprinty[i]=-width*CELL_DIM/2.;
	}
	footprintx[1]=length*CELL_DIM/2.;
	footprintx[2]=footprintx[1];
	footprinty[2]=width*CELL_DIM/2.;
	footprinty[3]=footprinty[2];

#if (RECTANGULAR==0) && 1
////pioneer2dx
//	 for (int i=7; i<15; i++){
//	    if (i<8){
//      footprinty[0]=-2.*CELL_DIM;  footprintx[0]=1.2*CELL_DIM;
//      footprinty[1]=-2.*CELL_DIM;  footprintx[1]=-1.2*CELL_DIM;
//      footprinty[2]=-1.2*CELL_DIM;  footprintx[2]=-2.555*CELL_DIM;
//      footprinty[3]=1.2*CELL_DIM;  footprintx[3]=-2.555*CELL_DIM;
//      footprinty[4]=2.*CELL_DIM;  footprintx[4]=-1.2*CELL_DIM;
//      footprinty[5]=2.*CELL_DIM;  footprintx[5]=1.2*CELL_DIM;
//      footprinty[6]=1.2*CELL_DIM;  footprintx[6]=2.555*CELL_DIM;
//      footprinty[7]=-1.2*CELL_DIM;  footprintx[7]=2.555*CELL_DIM;
//      }else{
//      footprintx[i]=footprintx[0];
//      footprinty[i]=footprinty[0];
//      }
//    }
//circular
//	 for (int i=7; i<15; i++){
//	    if (i<8){
//      footprinty[0]=0;  footprintx[0]=width/2.*CELL_DIM;
//      footprinty[1]=width/2./sqrt(2.)*CELL_DIM;  footprintx[1]=footprinty[1];
//      footprinty[2]=footprintx[0];  footprintx[2]=0;
//      footprinty[3]=footprinty[1];  footprintx[3]=-1.*footprintx[1];
//      footprinty[4]=0;  footprintx[4]=-1.*footprintx[0];
//      footprinty[5]=footprintx[3];  footprintx[5]=footprintx[3];
//      footprinty[6]=footprintx[4];  footprintx[6]=0;
//      footprinty[7]=footprintx[3];  footprintx[7]=footprintx[1];
//      }else{
//      footprintx[i]=footprintx[0];
//      footprinty[i]=footprinty[0];
//      }
//    }
#endif

#if RECTANGULAR

	std::vector<RobotColumn> columns;
	RobotColumn col;
	columns.clear();
	col.lower = 0;
	col.upper = 1;

//pioneer shape
#if 0
  col.vertices.push_back(std::make_pair(-2., 1.2));
  col.vertices.push_back(std::make_pair( -2.0, -1.2));
  col.vertices.push_back(std::make_pair( -1.2,  -2.555));
  col.vertices.push_back(std::make_pair( 1.2,  -2.555));
  col.vertices.push_back(std::make_pair( 2.,  -1.2));
  col.vertices.push_back(std::make_pair( 2.,  1.2));
  col.vertices.push_back(std::make_pair( 1.2,  2.555));
  col.vertices.push_back(std::make_pair( -1.2,  2.555));
	columns.push_back(col);
	 for (int i=0; i<15; i++){
	    if (i<8){
      footprintx[i]=col.vertices[i].first*CELL_DIM;
      footprinty[i]=col.vertices[i].second*CELL_DIM;
      }else{
      footprintx[i]=footprintx[0];
      footprinty[i]=footprinty[0];
      }
    }
#else //rectangular shape
//  col.vertices.push_back(std::make_pair(-(double)length/2.0, -(double)width/2.0));
//  col.vertices.push_back(std::make_pair( (double)length/2.0, -(double)width/2.0));
//  col.vertices.push_back(std::make_pair( (double)length/2.0,  (double)width/2.0));
//  col.vertices.push_back(std::make_pair(-(double)length/2.0,  (double)width/2.0));
//	columns.push_back(col);
//husky s hratca
//  col.vertices.push_back(std::make_pair(-4.5, -2.5));
//  col.vertices.push_back(std::make_pair( 11.5, -2.5));
//  col.vertices.push_back(std::make_pair( 11.5,  2.5));
//  col.vertices.push_back(std::make_pair( -4.5,  2.5));
//  col.vertices.push_back(std::make_pair(-2.5, -2.5));
//  col.vertices.push_back(std::make_pair( 5.5, -2.5));
//  col.vertices.push_back(std::make_pair( 5.5,  2.5));
//  col.vertices.push_back(std::make_pair( -2.5,  2.5));
//	columns.push_back(col);
//nas husky
  col.vertices.push_back(std::make_pair(-5., -3.5));
  col.vertices.push_back(std::make_pair( 5., -3.5));
  col.vertices.push_back(std::make_pair( 5.,  3.5));
  col.vertices.push_back(std::make_pair( -5.,  3.5));
	columns.push_back(col);
	 for (int i=0; i<15; i++){
	    if (i<4){
      footprintx[i]=col.vertices[i].first*CELL_DIM;
      footprinty[i]=col.vertices[i].second*CELL_DIM;
      }else{
      footprintx[i]=footprintx[0];
      footprinty[i]=footprinty[0];
      }
    }

#endif
	//this thing is a multi-part robot
//    RobotColumn col_two;
//    col_two.lower = 0;
//    col_two.upper = 1;
//    //this is for simulation paper
//    col_two.vertices.push_back(std::make_pair( -(double)length/4.0,  -(double)width/2.0));
//    col_two.vertices.push_back(std::make_pair( -(double)length/4.0,  -(double)(width/2.0+boxlength)));
//    col_two.vertices.push_back(std::make_pair( (double)(-length/4.0+boxwidth), -(double)(width/2.0+boxlength)));
//    col_two.vertices.push_back(std::make_pair( (double)(-length/4.0+boxwidth),-(double)width/2.0));
//    columns.push_back(col_two);

//    footprintx[5]=-length*CELL_DIM/4.; footprinty[5]=-width/2.*CELL_DIM;
//    footprintx[6]=-length*CELL_DIM/4.; footprinty[6]=-(width/2.0+boxlength)*CELL_DIM;
//    footprintx[7]=(-length/4.0+boxwidth)*CELL_DIM; footprinty[7]=-(width/2.0+boxlength)*CELL_DIM;
//    footprintx[8]=(-length/4.0+boxwidth)*CELL_DIM; footprinty[8]=-width/2.*CELL_DIM;

//keep this for the experiments from the paper
//    col_two.vertices.push_back(std::make_pair( (double)(length/2.0-3.92),  -(double)width/2.0));
//    col_two.vertices.push_back(std::make_pair( (double)(length/2.0-3.92),  -(double)(width/2.0+boxlength)));
//    col_two.vertices.push_back(std::make_pair( (double)(length/2.0-3.92-boxwidth), -(double)(width/2.0+boxlength)));
//    col_two.vertices.push_back(std::make_pair( (double)(length/2.0-3.92-boxwidth),-(double)width/2.0));
//    columns.push_back(col_two);

//    for (int i=0; i<4; i++){
//      footprintx[5+i]=col_two.vertices[i].first*CELL_DIM;
//      footprinty[5+i]=col_two.vertices[i].second*CELL_DIM;
//    }

//bigexp - changed to T sim
#if 0
    footprintx[9]=footprintx[1];  footprinty[9]=footprinty[1];
    footprintx[10]=footprintx[2]; footprinty[10]=footprinty[2];
    RobotColumn col_three;
    col_three.lower = 0; 
    col_three.upper = 1;
    col_three.vertices.push_back(std::make_pair( (double)(length/2.0),  (double)width/2.0));
    col_three.vertices.push_back(std::make_pair( (double)(length/2.0),  (double)(width/2.0+boxlength2)));
    col_three.vertices.push_back(std::make_pair( (double)(length/2.0-boxwidth), (double)(width/2.0+boxlength2)));
    col_three.vertices.push_back(std::make_pair( (double)(length/2.0-boxwidth), (double)width/2.0));
    columns.push_back(col_three);

    for (int i=0; i<4; i++){
      footprintx[11+i]=col_three.vertices[i].first*CELL_DIM;
      footprinty[11+i]=col_three.vertices[i].second*CELL_DIM;
    }
#endif    


	SimpleMap<int> smap(GM->Map_Dim_X,GM->Map_Dim_Y);
	int sizeX = smap.getMapSizeX();
	int sizeY = smap.getMapSizeY();
	// turn map into a height map
	for (int x=0; x<sizeX; x++) {
		for (int y=0; y<sizeY; y++) {
			if ((GM->Map[x][y].occupancy)>GRID_MAP_OCCUPANCY_TRESHOLD){
				smap.setCell(x,y,0); // totally occupied
			}
			else{
				smap.setCell(x,y,INT_MAX);
			}
		}
	}

	fprintf(stderr, "Map loaded (%d x %d).\n",  smap.getMapSizeX(), smap.getMapSizeY());

	// DYNAMIC CSPACE
	cspace = new CSpaceVoronoi(columns, &smap);
#if DSTAR3DORI
//  cspace->initializeDStar();
#else
//#endif
	cspace->update();
#endif
	if ( (F = fopen("logger//angularresolution.dat","wt")) == NULL ){
				 printf("Error! The file couldn't be opened.");
	}else{
		fprintf(F,"%f\n",cspace->angularResolution);
	}
	fclose(F);
	//some visualization
	cspace->layers[0]->saveCountMap("countmap_0.ppm");
	cspace->layers[3]->saveCountMap("countmap_3.ppm");
	cspace->layers[6]->saveCountMap("countmap_6.ppm");
	cspace->layers[9]->saveCountMap("countmap_9.ppm");
	
#endif
//----------------------------------------------------------------------	
//	PL->reset();//calculate DS costmap from GM binary map
	if ( (F = fopen("logger//robot_shape.dat","wt")) == NULL ){
				 printf("Error! The file couldn't be opened.");
	}else{
		fprintf(F,"%f %f\n",length*CELL_DIM/metric, width*CELL_DIM/metric);
	}
	fclose(F);  
	if ( (F = fopen("logger//robot_footprint.dat","wt")) == NULL ){
				 printf("Error! The file couldn't be opened.");
	}else{
	  for (int i=0; i<15; i++){
		  fprintf(F,"%f %f\n",footprintx[i],footprinty[i]);
		}
	}
	fclose(F);

	if ((F = fopen("gridmapa2.dat","wt")) != NULL)
	{
		for (int i=0; i < GM->Map_Dim_X; i++)
		{
			for (int j=0; j < GM->Map_Dim_Y; j++)
				fprintf(F,"%d ",GM->Map[i][j].occupancy);
			fprintf(F,"\n");
		}
		fclose(F);
	}

}



int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "tmsp1");
  ros::NodeHandle nh;

  M = new moj(nh);
  M->initializePlanner();

  VisualizationPublisher visual(nh);

  ros::Rate rate(10.0);

  while (nh.ok()) {
    ros::spinOnce(); 
    M->executeMotion();

//drawing in rviz	  
		if(M->voznja)
		{
			visual.visualizationduringmotion();	
		}

	  if (M->zavrsio)
		  break;
	  
	  rate.sleep();
  }
  return 0;
}



