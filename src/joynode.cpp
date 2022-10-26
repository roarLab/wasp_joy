#include "wasp_joy/joynode.h"
#include <cstdlib>
double button_state=0;
double start = 0;
double stop =0;
double x_trans = 0;
double coefx = 0.0;
double coefy = 0.0;
double coefz = 0.0;

void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  //previous_start_button=0;
  sensor_msgs::Joy newjoy;
  newjoy.buttons =joy->buttons;
  newjoy.axes =joy->axes;
  //ROS_INFO("  newjoy: [%ld]", (long int)newjoy.buttons[9]);
  start_button= joy->buttons[9];
  waypointstart_button= joy->buttons[8];
  button_state = joy->buttons[1];
/* ps4 controller
  command_velocity.linear.x  = 0.25*joy->axes[1] + 0.3*joy->axes[10];
  logistics_command.liftup  = 1*joy->buttons[1];
  command_velocity.linear.y  = 0.4*joy->axes[2];
  logistics_command.liftdown = 1*joy->buttons[3];
  logistics_command.extrude  = 1*joy->buttons[0];
  logistics_command.intrude  = 1*joy->buttons[2];
  command_velocity.angular.z = 0.1*joy->axes[0] + 0.2*joy->axes[9];
*/
  //Linear x and angular z must not exceed 0.45 and 0.25 respectively.
  if (abs(joy->axes[1]) > abs(joy->axes[4])){
  x_trans = joy->axes[1];
  }
  else {
  x_trans = joy->axes[4];
  }
 //This is for faster x-direction movement when y-direction and rotation is 0. 
  if (x_trans != 0){
	if (joy->axes[3] == 0 && joy->axes[0] == 0){
		coefx = 0.5;
		coefy = 0;
		coefz = 0;
	}
	else{
		coefx = 0.2;
		coefy = 0.2;
		coefz = 0.2;
	}
  }
 /* else if (joy-> axes[3] != 0){
	if (x_trans == 0 && joy->axes[0] == 0){
		coefx = 0;
		coefy = 0.5;
		coefz = 0;
	}
	else{
		coefx = 0.2;
		coefy = 0.2;
		coefz = 0.2;
	}
  }
  else if (joy-> axes[0] != 0){
	if (x_trans == 0 && joy->axes[3] == 0){
		coefx = 0;
		coefy = 0;
		coefz = 0.5;
	}
	else{
		coefx = 0.2;
		coefy = 0.2;
		coefz = 0.2;
	}
  }
*/
  command_velocity.linear.x  = joy->buttons[4] *coefx*x_trans;// + 0.3*joy->axes[10];
  command_velocity.linear.y  = joy->buttons[4] *coefy*joy->axes[3];
  command_velocity.angular.z = joy->buttons[4] *coefz*joy->axes[0];// + 0.2*joy->axes[9];
  logistics_command.liftup = 1*joy->buttons[4];
  logistics_command.liftdown = 1*joy->buttons[5];
  logistics_command.lifthome = 1*joy->buttons[3];
  logistics_command.extrude  = 1*joy->buttons[1];
  logistics_command.intrude  = 1*joy->buttons[2];
  command_velocity.angular.y  =1*joy->buttons[0];
  wall_command.init = 1*joy->axes[5];
  wall_command.move  = 1*joy->axes[7];
  
  
  cmd_xfm.linear.x = joy->buttons[4] * joy->buttons[0]; //xfm+ :: Fission Command
  cmd_xfm.linear.y = joy->buttons[4] * joy->buttons[1]; //xfm- :: Fusion Command
  cmd_xfm.linear.z = joy->buttons[4] * joy->buttons[2]; //clr :: Clear Costmap
  cmd_xfm.angular.x = joy->buttons[4] * joy->buttons[3];//call :: 
  cmd_xfm.angular.y = joy->buttons[4] * joy->axes[6];   //f+/- :: function selector
  cmd_xfm.angular.z = joy->buttons[4] * joy->axes[7];   //j+/- :: joint control
  
  
  
  
    if(button_state== 1){
start=1;
}
if (start==1 && stop ==0){
command_inter.data = 1;
stop = 1;
start= 0;
}
if (start==1 && stop ==1){
command_inter.data = 0;
stop = 0;
start= 0;
}
 // TeleopVelocity(2);
/*
switch(teleop_state)
	{
	case 0 :
		//ROS_INFO(" Teleop mode ON");	
		if(start_button!=previous_start_button)
		{
			teleop_state=1;
			previous_start_button= start_button;
			ROS_INFO("  case0");

		}
		break;
	case 1 :
		
		if(start_button!=previous_start_button)
		{
			teleop_state=2;
			previous_start_button= start_button;

		}
			ROS_INFO("  case1");

		//TeleopVelocity(2);
		break;
	
	case 2 :
		
		if(start_button!=previous_start_button)
		{
			teleop_state=3;
			previous_start_button= start_button;
			//ROS_INFO(" going to case 0");

		}
		ROS_INFO("  case2");
		TeleopVelocity(2);
		break;
	
	case 3 :
		
		if(start_button!=previous_start_button)
		{
			teleop_state=0;
			previous_start_button= start_button;
			ROS_INFO("  going to case 0");

		}
		ROS_INFO("  case3");
		//TeleopVelocity(2);
		break;

	}
	*/
	
   
}

void TeleopVelocity(int flag)
{
	ROS_INFO("  flag: [%ld]", (long int)flag);


}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joystick_teleop");
	ros::NodeHandle n;
	ros::Subscriber joy_sub_ = n.subscribe<sensor_msgs::Joy>("joy", 10, &JoyCallback);
	teleop_cmd = n.advertise<geometry_msgs::Twist>("command_velocity", 10);
	teleop_cmd2 = n.advertise<wasp_joy::logistics>("cmd_logi", 10);
        teleop_cmd3 = n.advertise<wasp_joy::wall>("cmd_wall", 10);
        inter_cmd = n.advertise<std_msgs::Int16>("/wm1/command_interconfig", 10);
	xfm = n.advertise<geometry_msgs::Twist>("/wm1/command_reconfig", 10);
		
	ros::Rate loop_rate(15);
	while (ros::ok())
	{
//if (abs(command_velocity.linear.x)>=0.00000001 || abs(command_velocity.linear.y)>=0.00000001 || abs(command_velocity.angular.z)>=0.00000001)
//{
	teleop_cmd.publish(command_velocity); 
	teleop_cmd2.publish(logistics_command);
	teleop_cmd3.publish(wall_command);
	inter_cmd.publish(command_inter); 
	xfm.publish(cmd_xfm);
//} 
	ros::spinOnce();
	loop_rate.sleep();
	}

	ros::spin();
	//srv.request.name = atoll(argv[1]);
	return 0;
}
