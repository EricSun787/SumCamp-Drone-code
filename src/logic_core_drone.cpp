#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <stdlib.h>
#include <dji_sdk/GlobalPosition.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <std_msgs/Char.h>
#include <apron/AutoChargeMsg.h>
#include <apriltags/AutoChargeMsg.h>
#include <rmg2_pid_controller.h>
#include <rmg2/rmg2_control.h>

using namespace std;

ros::Subscriber drone_pos_sub;
ros::Subscriber drone_yaw_sub;
ros::Subscriber drone_UtraH_sub;
ros::Subscriber drone_qpos_sub;
ros::Subscriber drone_landpos_sub;

ros::Subscriber drone_mission_feedback_sub;
ros::Publisher  drone_control_pub;
char                      mission_state = 0;
apriltags::AutoChargeMsg            drone_qpos;
apron::AutoChargeMsg                drone_landpos;
float		                    drone_yaw;
geometry_msgs::Vector3Stamped       drone_pos;
float		                    drone_UtraH;
float                               drone_realH;

rmg2::rmg2_control  control_command;

void drone_qpos_callback(const apriltags::AutoChargeMsg& qpos)
{
	drone_qpos = qpos;
	
//	ROS_INFO("Qpos is X:%f  Y:%f  Dis:%f  Yaw:%f  id:%d  \n",drone_qpos.X_head,drone_qpos.Y_right,drone_qpos.distance,drone_qpos.yaw,drone_qpos.id);
}
void drone_pos_callback(const  geometry_msgs::Vector3Stamped& dpos)
{
	drone_pos = dpos;
	drone_realH = -(drone_pos.vector.z);
//	ROS_INFO("Dpos is X:%f  Y:%f  Z:%f  \n",drone_pos.vector.x,drone_pos.vector.y,drone_realH);
}


void drone_yaw_callback(const geometry_msgs::TransformStamped& dyaw)
{
	drone_yaw = dyaw.transform.rotation.z;
	ROS_INFO("Drone yaw is %f \n",drone_yaw);
}
void drone_UtraH_callback(const dji_sdk::GlobalPosition& dUH)
{
	drone_UtraH = dUH.height;
//	ROS_INFO("UtraH is : %f \n",drone_UtraH);	
}
void drone_landpos_callback(const apron::AutoChargeMsg& landpos)
{
	drone_landpos = landpos;
//	ROS_INFO("Landpos is X: %f  Y: %f  id: %d  \n",drone_landpos.X,drone_landpos.Y,drone_landpos.id);
	
}
void drone_mission_feedback_callback(const std_msgs::Char&         msg)
{
      mission_state = msg.data;
//	ROS_INFO("mission_state is %d  \n ",mission_state);
}
static void Display_Main_Menu(void)
{
	printf("\r\n");
	printf("----------------< Control Menu >--------------------\n");
	printf("|  [1]----Take off        \n");
	printf("|  [2]----Find Qpos       \n");
	printf("|  [3]----Position hold   \n");
	printf("|  [4]----Find Landpos    \n");
	printf("|  [5]----Landing         \n");
	printf("|  Input num to start mission : \n");
}
int main(int argc,char** argv)
{
	int   temp32;
	int   i=0;
	int   j=0;	
        int   mission_code;
	float   Heigth = 3.0;

	control_command.xpid.limit  = 0.8;
	control_command.xpid.Kp     = 1.0;
	control_command.xpid.Ki	    = 0.1;
	control_command.xpid.Kd     = 0.1;
	
	control_command.ypid.limit  = 0.8;
	control_command.ypid.Kp     = 1.0;
	control_command.ypid.Ki	    = 0.1;
	control_command.ypid.Kd     = 0.1;
	
	control_command.zpid.target = Heigth;
	control_command.zpid.limit  = 0.8;
	control_command.zpid.Kp     = 1.0;
	control_command.zpid.Ki	    = 0.1;
	control_command.zpid.Kd     = 0.1;
	
        control_command.yawpid.target  = 0.0;
	control_command.yawpid.limit = 9.0;
	control_command.yawpid.Kp    = 10.0;
	control_command.yawpid.Ki    = 2;
	control_command.yawpid.Kd    = 2;
	
	control_command.qxpid.feedback = 0;
	control_command.qypid.feedback = 0;

	control_command.qxpid.Kp       = 0.8;
	control_command.qypid.Kp       = 0.8;
	control_command.qxpid.Ki       = 0.06;
	control_command.qypid.Ki       = 0.06;
	control_command.qxpid.Kd       = 0.06;
	control_command.qypid.Kd       = 0.06;
	
	control_command.qxpid.limit    = 0.6;
	control_command.qypid.limit    = 0.6;

	
	
		
	ros::init(argc,argv, "logic_core_drone");
	
	ros::NodeHandle ln;
	ros::Rate pid_rate(50);
	
	drone_pos_sub   = ln.subscribe("/guidance/motion_data",100,drone_pos_callback);
	drone_yaw_sub   = ln.subscribe("/guidance/imu",100,drone_yaw_callback);
	drone_UtraH_sub = ln.subscribe("/dji_sdk/global_position",20,drone_UtraH_callback);
	drone_qpos_sub  = ln.subscribe("/chargedata_aircraft",10,drone_qpos_callback);
	drone_landpos_sub = ln.subscribe("/chargedata_apron",10,drone_landpos_callback); 
	drone_mission_feedback_sub   = ln.subscribe("/rmg2/drone_mission_feedback",10,drone_mission_feedback_callback);
	drone_control_pub = ln.advertise<rmg2::rmg2_control>("/rmg2/drone_control",1);
	while(ros::ok())
	{
	  Display_Main_Menu();
	//  ros::spinOnce();
	  std::cout << "Enter mission code : ";
	  std::cin >> temp32;
	  if(temp32 > 0 && temp32 < 99)
		{
		   mission_code = temp32;	
		}	
		else 
			{
				printf("Out of Range input! \n");
				Display_Main_Menu();
				continue;
			}
		switch(mission_code)
			{
				case 1:
					control_command.command = 1;
					drone_control_pub.publish(control_command);
					mission_state = 0;
					while(!mission_state)
						ros::spinOnce();
					ROS_INFO("Task 1 is done!\n");
				break;
				case 2:
					mission_state = 0;
					control_command.command = 2;
					while(!mission_state)
						{
						    control_command.qxpid.target = drone_qpos.X_head;
						    control_command.qypid.target = drone_qpos.Y_right;	
						    control_command.qxpid.header.frame_id = drone_qpos.id;
						    control_command.yawpid.feedback = 0;
						    control_command.yawpid.target = drone_qpos.yaw;
						    drone_control_pub.publish(control_command);
						    ros::spinOnce();
						}
					ROS_INFO("Task 2 is done!\n");
					mission_code = 0;
	
					break;
				case 3:	
					mission_state = 0;
					control_command.command = 3;
					printf("Please input X target : ");
					std::cin >> control_command.xpid.target;
					printf("Please input Y target : ");
					std::cin >> control_command.ypid.target;
					printf("Please input Yaw target:");
					std::cin >> control_command.yawpid.target;
					ros::spinOnce();
					while(!mission_state)
					{
						control_command.xpid.feedback = drone_pos.vector.x;
						control_command.ypid.feedback = drone_pos.vector.y; 
						control_command.zpid.target = Heigth;
						control_command.zpid.feedback = drone_realH;
						control_command.yawpid.feedback = drone_yaw;
					        drone_control_pub.publish(control_command);
						ros::spinOnce();
						pid_rate.sleep();
					}
					ROS_INFO("Task 3 is done!\n");
					mission_code = 0;
					break;
				case 4:
					mission_state = 0;
					control_command.command = 4;
					while(!mission_state)
						{
						    control_command.qxpid.target = drone_landpos.X;
						    control_command.qypid.target = drone_landpos.Y;	
						    control_command.qxpid.header.frame_id = drone_landpos.id;
						    drone_control_pub.publish(control_command);
						    ros::spinOnce();
						}
					ROS_INFO("Task 4 is done!\n");
					mission_code = 0;
	                                break;
			 	case 5:
					control_command.command = 5;
					drone_control_pub.publish(control_command);
					mission_state = 0;
					while(!mission_state)
						ros::spinOnce();
					ROS_INFO("Task 5 is done!\n");
					break;
				default : break;
					
                                        
					
			}
			control_command.command = 0;
	}
}
