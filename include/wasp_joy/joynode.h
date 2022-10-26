#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <wasp_joy/logistics.h>
#include <wasp_joy/wall.h>
#include <cstdlib>
#include "std_msgs/Int16.h"

geometry_msgs::Twist command_velocity, cmd_xfm;
wasp_joy::logistics logistics_command;
wasp_joy::wall wall_command;
std_msgs::Int16 command_inter;
ros::Publisher  teleop_cmd, teleop_cmd2, teleop_cmd3,inter_cmd, xfm;
double start_button;
double joy_check=0;
double previous_start_button=0;
double waypointstart_button;
int flag = 1;
int teleop_state =0;
void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy);
void TeleopVelocity(int flag);
