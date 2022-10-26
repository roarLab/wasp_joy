#include "ros/ros.h"
#include "wasp_joy/GetPose.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <iostream>
#include <fstream>
#include <vector>
#include<move_base_msgs/MoveBaseAction.h>
#include<actionlib/client/simple_action_client.h>
#include<sstream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
struct goal_msgs
{
	std::vector<float> x;
	std::vector<float> y;
	std::vector<float> theta_z;
	std::vector<float> theta_w;
};

goal_msgs ReadFile(std::string file_location)
	{
		  //std::ofstream myfile(file_location);
		  std::ifstream myfile;
		  myfile.open(file_location);
		  std::string line;
		  //std::vector<goal_msgs> point_vec;
		  goal_msgs data,point;
		  float valx,valy,valtheta_z, valtheta_w;
		  int i=0;
		  while(std::getline(myfile,line))
		  {
		  	std::string token;
		  	std::stringstream ss(line);
		  	//ss>> valx >> valy >> valtheta;
		  	std::getline(ss,token,',');
		  	valx=std::stod(token);
		  	std::getline(ss,token,',');
		  	valy=std::stod(token);
		  	std::getline(ss,token,',');
		  	valtheta_z=std::stod(token);
		  	std::getline(ss,token,',');
		  	valtheta_w=std::stod(token);
		  	point.x.push_back(valx);
		  	point.y.push_back(valy);
		  	point.theta_z.push_back(valtheta_z);
		  	point.theta_w.push_back(valtheta_w);
		  	
		  }
		  myfile.close();
		  for (int i=0; i<point.theta_z.size();i++)
		  {
		  	std::cout << point.theta_z[i];
		  }
		  
		  //point.x={0,1,1};
		  //point.y={0,1,1};
		  //point.theta={0,1,1};
		  return point;		  
	}

class GetWaypoints{
public:

		//double _x     = 0.0;
		//double _y     = 0.0;
		//double _theta = 0.0;
		bool Waypoint(wasp_joy::GetPose::Request &req, wasp_joy::GetPose::Response &res );
		bool ResetWaypoint(wasp_joy::GetPose::Request &req, wasp_joy::GetPose::Response &res );
		bool NaviWaypoint(wasp_joy::GetPose::Request &req, wasp_joy::GetPose::Response &res );
		bool WallFollower(wasp_joy::GetPose::Request &req, wasp_joy::GetPose::Response &res );
		void WriteFile(std::vector<float> V_X, std::vector<float> V_Y, std::vector<float> V_THETA_Z,std::vector<float> V_THETA_W);
		void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
		void AmclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL);

	
	
private:
	double _x;
	double _y;
	double _theta_z;
	double _theta_w;
	double o_x;
	double o_y;
	double o_theta;
	std::vector<float> v_x;
	std::vector<float> v_y;
	std::vector<float> v_theta_z;
	std::vector<float> v_theta_w;


};

bool GetWaypoints::Waypoint(wasp_joy::GetPose::Request &req, wasp_joy::GetPose::Response &res )
	{
		res.x = _x;
		res.y = _y;      
		res.theta_z=_theta_z;
		res.theta_w=_theta_w;
		v_x.push_back(_x);
		v_y.push_back(_y);
		v_theta_z.push_back(_theta_z);
		v_theta_w.push_back(_theta_w);


		ROS_INFO("  waypoint storing: [%ld]", (long int)v_x.size());
		WriteFile(v_x, v_y, v_theta_z, v_theta_w);
		return true;
	}
bool GetWaypoints::ResetWaypoint(wasp_joy::GetPose::Request &req, wasp_joy::GetPose::Response &res )
	{
		res.x = _x;
		res.y = _y;      
		res.theta_z=_theta_z;
		res.theta_w=_theta_w;
		v_x.clear();
		v_y.clear();
		v_theta_z.clear();
		v_theta_w.clear();


		ROS_INFO("  reset waypoint: [%ld]", (long int)v_x.size());
		WriteFile(v_x, v_y, v_theta_z, v_theta_w);
		return true;
	}
bool GetWaypoints::NaviWaypoint(wasp_joy::GetPose::Request &req, wasp_joy::GetPose::Response &res )
	{
		res.x = _x;
		res.y = _y;      
		res.theta_z=_theta_z;
		MoveBaseClient ac("move_base", true);
		goal_msgs point_;
		ROS_INFO("navi_waypoint");
		point_=ReadFile("/home/sutd_wasp/nav_points.csv");
    	while(!ac.waitForServer(ros::Duration(5.0)))
    	{
    	ROS_INFO("Sending goal[%ld]", (long int)point_.x.size());
        ROS_INFO("Waiting for the move_base action server");
        }
	    move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "base_link";
		goal.target_pose.header.stamp = ros::Time::now();
		for( int i = 0; i < point_.x.size(); i++ ) 
		{
		    goal.target_pose.pose.position.x = point_.x[i];
		    goal.target_pose.pose.position.y = point_.y[i];

		    goal.target_pose.pose.orientation.z = point_.theta_z[i];
		    goal.target_pose.pose.orientation.w = point_.theta_w[i];
		  	ROS_INFO("Sending goal[%ld]", (long int)point_.x[i]);
		  	ac.sendGoal(goal);

		  	ac.waitForResult();

	      	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	        	ROS_INFO("Robot moving");
	      	else
	        	ROS_INFO("The robot failed to move for some reason");
       }


		ROS_INFO("  sending back response: [%ld]", (long int)v_x.size());
		WriteFile(v_x, v_y, v_theta_z, v_theta_w);
		return true;
	}
bool GetWaypoints::WallFollower(wasp_joy::GetPose::Request &req, wasp_joy::GetPose::Response &res )
{
		res.x = _x;
		res.y = _y;      
		res.theta_z=_theta_z;
		res.theta_w=_theta_w;


		ROS_INFO("  WallFollower: ");
		return true;
}
void GetWaypoints::WriteFile(std::vector<float> V_X, std::vector<float> V_Y, std::vector<float> V_THETA_Z, std::vector<float> V_THETA_W)
	{
		  std::ofstream myfile("/home/sutd_wasp/nav_points.csv");
		  for(int i=0; i < V_X.size(); ++i)
		  {
		  	myfile << V_X.at(i)<<","<<V_Y.at(i)<<","<<V_THETA_Z.at(i)<<","<<V_THETA_W.at(i)<<"\n";
		  }
		  
		  
	}
void GetWaypoints::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
	{
		o_x=msg->pose.pose.position.x;
		o_y=msg->pose.pose.position.y;
		o_theta=msg->pose.pose.orientation.z;
	}
void GetWaypoints::AmclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
	_x =msgAMCL->pose.pose.position.x;
    _y = msgAMCL->pose.pose.position.y;
    _theta_w = msgAMCL->pose.pose.orientation.w;
    _theta_z = msgAMCL->pose.pose.orientation.z;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_server");
  ros::NodeHandle n;
  GetWaypoints monitor;

  ros::ServiceServer service = n.advertiseService("get_pose", &GetWaypoints::Waypoint, &monitor);
  ros::ServiceServer resetservice = n.advertiseService("reset_pose", &GetWaypoints::ResetWaypoint, &monitor);
  ros::ServiceServer naviservice = n.advertiseService("navi_pose", &GetWaypoints::NaviWaypoint, &monitor);
  ros::ServiceServer wall_followservice = n.advertiseService("wall_follow", &GetWaypoints::WallFollower, &monitor);
  ros::Subscriber sub = n.subscribe("odometry/filtered", 1, &GetWaypoints::OdomCallback, &monitor);
  ros::Subscriber sub_amcl = n.subscribe("amcl_pose",1, &GetWaypoints::AmclCallback, &monitor);
  ros::spin();
  //myfile.close();
  return 0;
}

