#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "stdio.h"
#include "dji_sdk/dji_drone.h"
#include "cstdlib"
#include "stdlib.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"



float speedX,speedY,speedZ;
void keyboard_callback(const geometry_msgs::Twist& kb_ctr)
{
	ROS_INFO("Linear  X: %f , Y: %f , Z: %f \n ",kb_ctr.linear.x,kb_ctr.linear.y,kb_ctr.linear.z);
	ROS_INFO("Angular X: %f , Y: %f , Z: %f  \n",kb_ctr.angular.x,kb_ctr.angular.y,kb_ctr.angular.z);
 //       speedX = kb_ctr.linear.x;
//	speedY = kb_ctr.linear.y;
//	speedZ = kb_ctr.angular.z;
}


using namespace DJI::onboardSDK;

 ros::Subscriber kb_sub ;

int main(int argc, char **argv)
{
	int main_operate_code = 0;
	int temp32;
	ros::init(argc, argv, "keyboard_drone");
	ROS_INFO("Keyboard_Control_Test\n");

	ros::NodeHandle my_node;
	DJIDrone* test_drone = new DJIDrone(my_node);
	kb_sub = my_node.subscribe("/cmd_vel",100,keyboard_callback);
      
        ros::spinOnce();

	while(ros::ok())
	{
		ros::spinOnce();
		std::cout << "(1-Rq Ctr 2-tack off 3-land 4-muilt 5-for 6-round 7-rightround 8-left 9-right): ";
		while(!(std::cin >> temp32))
		{
			std::cin.clear();
			std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			std::cout << "Invalid input .Try again:";
		}
		 
		if(temp32>0 && temp32<10)
		{
			main_operate_code = temp32;	
		}
		else
		{
			printf("Error - Out of range input !\n");

			continue;
		}
		switch(main_operate_code)
		{
			case 1:
			test_drone->request_sdk_permission_control();

			break;
			case 2:
			test_drone->takeoff();
			break;
			case 3:
			test_drone->landing();
			break;
			case 4:
				for(int i=0;i<190;i++)
				{
			  	   if(i<170)
				{
				//test_drone->attitude_control(0x4b,0,0,0,10);
				test_drone->velocity_control(1,3,0,0,9);
				}
				   else
				{
				//test_drone->attitude_control(0x4b,0,0,0,0);
				test_drone->velocity_control(1,0,0,0,0);
				}
				  usleep(20000);
				}
				break;
			case 5:
				for(int i=0;i<190;i++)
				{
				  if(i<170)
			test_drone->attitude_control(0x4b,1,0,0,0);
			//		test_drone->velocity_control(2,-1,0,0,0);
				  else
				test_drone->attitude_control(0x4b,0,0,0,0);
//					test_drone->velocity_control(2,0,0,0,0);
				 usleep(20000);
				}
				break;
			case 6:
				for(int i=0;i<140;i++)
				{
					if(i<125)
		//	test_drone->attitude_control(0xa,0,0,0,3);
		//		test_drone->velocity_control(2,0,0,0,90);
			test_drone->attitude_control(0x4b,0,0,0,3);
					else
			//		test_drone->attitude_control(0xa,0,0,0,0);
//				test_drone->velocity_control(2,0,0,0,0);
			test_drone->attitude_control(0x4b,0,0,0,0);
			        usleep(20000);	
				}
			break;
			case 7:
				for(int i=0;i<40;i++)
				{
					if(i<25)
					//test_drone->attitude_control(0xa,0,0,0,90);	
					test_drone->velocity_control(2,0,0,0,-90);
					else
					//test_drone->attitude_control(0xa,0,0,0,0);
					test_drone->velocity_control(2,0,0,0,0);
				usleep(20000);
				}
			break;
			case 8:
				for(int i=0;i<120;i++)
				{
					if(i<100)
			//		test_drone->attitude_control(0x40,0,0,0.5,0);
		//			test_drone->attitude_control(0x4b,0,0,1,0);
					test_drone->velocity_control(2,0,1,0,0);
					else
		
					test_drone->velocity_control(2,0,0,0,0);
				//	test_drone->attitude_control(0x40,0,0,0,0);
		//			test_drone->attitude_control(0x4b,0,0,0,0);
				usleep(20000);
				}
			break;
			case 9:
				for(int i=0;i<120;i++)
				{
					if(i<100)
			
					test_drone->velocity_control(2,0,-1,0,0);
					//test_drone->attitude_control(0x40,0,0,-0.5,0);	
					else
					
					test_drone->velocity_control(2,0,0,0,0);
					//test_drone->attitude_control(0x40,0,0,0,0);
				usleep(20000);
				}
			break;
			default:
				break;
		}
			main_operate_code = -1;
	}
	//ros::spin();
/*	
	ROS_INFO("Request Control!! \n");
	sleep(2);
	test_drone->request_sdk_permission_control();

	ROS_INFO("Already Control!!\n");

	ROS_INFO("Auto takeoffing!!!!\n");

	

	ros::spinOnce();

	while(1)
	{

	ros::spinOnce();
	
        if(speedX)
	{
		test_drone->attitude_control(0x40,0,1,0,0);
	}		
	
	if(speedY)
	{	
                test_drone->attitude_control(0x40,1,0,0,0);
	
        }
	if(speedZ)
	{
		test_drone->attitude_control(0x40,0,0,1,0);
	}
		

        }	
*/	
	return 0;
}
