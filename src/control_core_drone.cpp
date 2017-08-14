#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <geometry_msgs/Twist.h>
#include <dji_sdk/GlobalPosition.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Char.h>
#include <apron/AutoChargeMsg.h>
#include <apriltags/AutoChargeMsg.h>
#include <rmg2/rmg2_control.h>
#include <rmg2/rmg2_pid.h>
#include <rmg2/rmg2_position.h>

#include "rmg2_pid_controller.h"
#include <dji_sdk/dji_drone.h>

using namespace DJI::onboardSDK;
using namespace std;


//float Odm_correct_x = 0;
//float Odm_correct_y = 0;

std_msgs::Char       c_state_sub;
std_msgs::Char       d_state_pub;

std_msgs::Char       drone_mission_feedback;
rmg2::rmg2_control   drone_control;
rmg2::rmg2_pid       pidX,pidY,pidZ,pidYaw,pidQX,pidQY;
float pid_acc = 0.02;

ros::Subscriber drone_control_sub;
ros::Subscriber car_state_sub;

ros::Publisher  d_mission_feedback_pub;
ros::Publisher  drone_state_pub;

float vx,vy,vz,vyaw;
void car_state_callback(const std_msgs::Char&  msg)
{
	c_state_sub = msg;
	ROS_INFO("Recevice Car requist is %d \n",c_state_sub.data);
}
void drone_control_callback(const rmg2::rmg2_control& msg)
{
	 pidX = msg.xpid;
	 pidY = msg.ypid;
	 pidZ = msg.zpid;
	 pidYaw = msg.yawpid;
	 pidQX  = msg.qxpid;
	 pidQY  = msg.qypid;
	 drone_control.data = msg.data;
	 drone_control.command = msg.command;
	 //drone_mission_feedback.data = 0;
	

}
int main(int argc, char** argv)
{
	int i;
	int j=0;
	int k=0;
	ros::init(argc,argv,"control_core_drone");
	ros::NodeHandle ch;
        
	
	ros::Rate  pid_rate(50);
	ros::Rate  Qpos_rate(20);
	DJIDrone* drone = new DJIDrone(ch);
	drone_control_sub = ch.subscribe("/rmg2/drone_control",1,drone_control_callback);
	car_state_sub = ch.subscribe("/rmg2/car_state",10,car_state_callback);
	d_mission_feedback_pub = ch.advertise<std_msgs::Char>("/rmg2/drone_mission_feedback",10);
	drone_state_pub        = ch.advertise<std_msgs::Char>("/rmg2/drone_state",10);
	ROS_INFO("Control core drone is running");
      while(ros::ok())
      {
	drone_control.command = 0;
	ros::spinOnce();	
	switch(drone_control.command)
	{
		case 1:
			ROS_INFO("Request control !\n");
			drone->request_sdk_permission_control();
			ROS_INFO("Already requested control ! \n");
			ROS_INFO("drone will take off in 3s !!\n");
			sleep(2);
			drone->takeoff();
			sleep(5);
			for(int i =0;i<200;i++)
			{
				if(i<100)
					drone->attitude_control(0x4b,0,0,1,0);
				else
					drone->attitude_control(0x4b,0,0,0,0);
			usleep(20000);
			}
				
			ROS_INFO("The drone is already took off!\n");
			drone_mission_feedback.data = 1;
			d_mission_feedback_pub.publish(drone_mission_feedback);
			break;
		case 2:
			ROS_INFO("Qpos mode\n");
		    while(absd(pidQX.target)>pid_acc || absd(pidQY.target)>pid_acc)
			{
				vx = rmg2_Xpid_control(pidQX);
				vy = rmg2_Ypid_control(pidQY);
				vyaw = rmg2_Yawpid_control(pidYaw);
				drone->attitude_control(0x4b,vx,vy,0,vyaw);
				printf("vx[%f],vy[%f]  \n",vx,vy);
				ros::spinOnce();
				Qpos_rate.sleep();	
			}
		 		drone_mission_feedback.data = 1;
			  for(i=0;i<5;i++)
			{
				d_mission_feedback_pub.publish(drone_mission_feedback);
			        drone->attitude_control(0x4b,0,0,0,0);	
				usleep(200);
			}
				sleep(1);
			break;
		case 3:
			ROS_INFO("Position mode\n");
		    while(absd(pidX.feedback - pidX.target)>pid_acc || absd(pidY.feedback-pidY.target)>pid_acc )
			{
				vx = rmg2_Xpid_control(pidX);
				vy = rmg2_Ypid_control(pidY);
				vz = rmg2_Zpid_control(pidZ);
			//	vyaw = rmg2_Yawpid_control(pidYaw);
				
				//drone->attitude_control(0x4B,vx,vy,0,-vyaw);
				drone->velocity_control(1,vx,vy,vz,0);
				printf("vx[%f],vy[%f],vz[%f]  \n",vx,vy,vz);
				pid_rate.sleep();
				ros::spinOnce();
			}
			        //drone->attitude_control(0x4b,0,0,0,0);
		 		drone_mission_feedback.data = 1;
				std::cout << j <<std::endl;
			  for(i=0;i<3;i++)
			{
				d_mission_feedback_pub.publish(drone_mission_feedback);
				drone->velocity_control(1,0,0,0,0);	
				sleep(1);
				j++;
			}
				sleep(1);
				break;
	   	case 4:
			ROS_INFO("Landpos mode\n");
		    
		    while(absd(pidQX.target)>pid_acc || absd(pidQY.target)>pid_acc)
			{
				vx = rmg2_Xpid_control(pidQX);
				vy = rmg2_Ypid_control(pidQY);

				drone->attitude_control(0x4b,vx,vy,0,0);
				printf("vx[%f],vy[%f]  \n",vx,vy);
				ros::spinOnce();
				Qpos_rate.sleep();	
                 	}
		        	drone_mission_feedback.data = 1;
			  for(i=0;i<5;i++)
			{
				d_mission_feedback_pub.publish(drone_mission_feedback);
		                drone->attitude_control(0x4b,0,0,0,0);
				usleep(200);
			}
				sleep(1);
				break;
	   	case 5:
			ROS_INFO("Landing mode\n");
			drone->landing();
			ROS_INFO("The drone is already landing!\n");
			drone_mission_feedback.data = 1;
			d_mission_feedback_pub.publish(drone_mission_feedback);
		break;	
}
}
}
