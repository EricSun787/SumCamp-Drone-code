#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <stdlib.h>
#include <dji_sdk/GlobalPosition.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Int32.h>
#include <iostream>
//#include <conio.h>

#include <apriltags/AutoChargeMsg.h>
#include <apron/AutoChargeMsg.h>
#include <apriltags/Control.h>

#include "rmg2_pid_controller.h"
#include <rmg2/drone_serial_data.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>


using namespace DJI::onboardSDK;
using namespace std;
int kbhit(void)
{
struct termios oldt,newt;
int ch;
int oldf;
tcgetattr(STDIN_FILENO, &oldt);
newt = oldt;
newt.c_lflag &= ~(ICANON | ECHO); 
tcsetattr(STDIN_FILENO, TCSANOW, &newt);  
oldf = fcntl(STDIN_FILENO, F_GETFL, 0);  
fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);  
ch = getchar();  
tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  
fcntl(STDIN_FILENO, F_SETFL, oldf);  
if(ch != EOF)  
{  
ungetc(ch, stdin);  
return 1;  
}  
return 0;  
}  

float Drone_Yaw        = 0;
float UHeight          = 0;
float RealH            = 0;
float Heigth           = 2.5;

geometry_msgs::Vector3Stamped Drone_pos;

apriltags::AutoChargeMsg      Q_pos;
apron::AutoChargeMsg          L_pos;
apriltags::Control  	      C_state;
std_msgs::Int32               Q_num;


int Qflag = 0;
volatile int QorL  = 0; //0---Qpos  1---Landpos
int Q_id  = 0;
float Qposx_bias = 0.20;
float Qposy_bias = 0.02;

float Lposx_bias = 0.0865;
float Lposy_bias = 0.02;

float Dposx_bias = 0;
float Dposy_bias = 0;
float Dposz_bias = 0;

PID pidX,pidY,pidZ,pidYaw,pidQY,pidQX,pidQYaw;
	
rmg2::rmg2_pid pidx,pidy,pidz,pidyaw,pidqy,pidqx,pidqyaw;
rmg2::drone_serial_data hand_data;

	ros::Subscriber drone_pos_sub;
	ros::Subscriber drone_yaw_sub;
	ros::Subscriber drone_UH_sub;
	ros::Subscriber Qpos_sub;
	ros::Subscriber Landpos_sub;

	ros::Subscriber mission_sub;	
	ros::Subscriber Height_sub;
	ros::Publisher mission_complete_pub;

	ros::Publisher hand_pub;
	ros::Publisher camera_select_pub;
	ros::Publisher Q_num_pub;
int pos_get=0;
int yaw_get=0;
int UH_get =0;
int Qpos_get =0;
int Landpos_get =0;
int aim_count=0;
int qpos_count=0;
int landpos_count=0;
int pos_count=0;
float pid_acc    = 0.035;
geometry_msgs::Vector3Stamped dp;
void Ready()
{
	while(!(pos_get)||!(yaw_get)||!(UH_get))
	{
		ros::spinOnce();
		printf("Waiting for Ready!\n");
		sleep(1);
	}
}
void Menu_Dis(void)
{
	// case 3 budanceshi  case 6 huipingtai
	printf("--------Main Menu--------------\n");
	printf("-----[1]-----Take off----------\n");
	printf("-----[2]-----Position----------\n");
	printf("-----[3]-----AutoFeed-Qpos-----\n");
	printf("-----[4]-----Auto-Landpos------\n");

	printf("-----[5]-----Full Auto feed----\n");
	printf("-----[6]-----Go home-----------\n");
	printf("-----[7]-----Set Bias----------\n");


	printf("-----[8]-----LandPos------------\n");
	printf("-----[9]-----Request control----\n");
	printf("-----[10]----Qpos---------------\n");
	printf("-----[11]----Seach Qpos---------\n");
	printf("-----[12]----Seach Landpos------\n");
	printf("-----[13]-----Release amo-------\n");
	printf("-----[14]-----Take amo----------\n");
	printf("-----[15]-----Landing-----------\n");
	printf("-----[16]-----Mission stage 2---\n");
	printf("Please input your command: ");
}

void drone_pos_callback(const geometry_msgs::Vector3Stamped& msg)
{
	pidx.feedback = (msg.vector.x - (Dposx_bias));
	pidy.feedback = (msg.vector.y - (Dposy_bias));
	pidz.feedback =  -(msg.vector.z-(Dposz_bias));
	dp = msg;
	if(!pos_get)
	   pos_get =1;
}
void drone_yaw_callback(const geometry_msgs::TransformStamped& msg)
{
	pidyaw.feedback = msg.transform.rotation.z;	
	if(!yaw_get)
		yaw_get =1;
}
void drone_UH_callback(const dji_sdk::GlobalPosition& msg)
{
	UHeight = msg.height;
	if(!UH_get)
		UH_get =1;
}
void Qpos_callback(const apriltags::AutoChargeMsg& msg)
{
	if(!C_state.state)
		{
			pidqx.target = (msg.X_head - Qposx_bias);
			pidqy.target = (msg.Y_right + Qposy_bias);
			pidqyaw.target = -msg.yaw;
		if(qpos_count < 5)
		qpos_count++;
		else
			Qpos_get =1;
	}
	else if(C_state.state)
		{
			pidqx.target = (msg.X_head - Lposx_bias);
			pidqy.target = (msg.Y_right - Lposy_bias);
		printf("X: %f , Y: %f  \n",pidqx.target,pidqy.target);
		if(landpos_count < 5)
		landpos_count++;
		else 
			Landpos_get = 1;	
			printf("Landpos_get : %d  \n",Landpos_get);
			
		}
		
		
			Q_id         = msg.id; 
}
/*void landpos_callback(const apron::AutoChargeMsg& msg)
{
	if(QorL)
		{
			pidqx.target = msg.X;
			pidqy.target = msg.Y;	
		}
	if(!Landpos_get)
		Landpos_get =1;
}*/

//No.3 -> 2   
//No.2 -> 0  1.5 -0.3 0.3
//No.5 -> 1  3.0 -0.8 0.3

int main(int argc,char** argv)
{
	float vx = 0;
	float vy = 0;
	float vz = 0;
	float vyaw = 0;
	int   mode = 0;
	int   seach_get = 0;
	int   key = 0;
	int   i=0;
	int   trash;
	C_state.state = 0;
	pidZ.setLimit(0.6);
	pidYaw.setLimit(10);
	pidQX.setParam(0.6,0.06,0.06);
	pidQY.setParam(0.6,0.06,0.06);
	pidQX.setLimit(0.2);
	pidQY.setLimit(0.2);
	pidQYaw.setLimit(15);
	pidQYaw.setParam(1.3,0.8,0.8);//1.3 0.8 0.8
	pidYaw.setParam(10,2,2);
	
	pidz.target = Heigth;
	pidqyaw.feedback = 0;

	ros::init(argc,argv,"Drone_control_core");
	ros::NodeHandle nh;
	
	DJIDrone* drone = new DJIDrone(nh);
	
	ros::Rate pos_rate(50);
	ros::Rate qpos_rate(20);
	ros::Rate landpos_rate(20);
	
	drone_pos_sub = nh.subscribe("/guidance/motion_data",100,drone_pos_callback);
	drone_yaw_sub = nh.subscribe("/guidance/imu",100,drone_yaw_callback);
	drone_UH_sub  = nh.subscribe("/dji_sdk/global_position",100,drone_UH_callback);
	Qpos_sub      = nh.subscribe("/chargedata_aircraft",100,Qpos_callback);
//	Landpos_sub   = nh.subscribe("/chargedata_apron",100,landpos_callback);
		
	hand_pub      = nh.advertise<rmg2::drone_serial_data>("/rmg2/drone_hand_control",5);
	camera_select_pub = nh.advertise<apriltags::Control>("/chargedata",10);
	Q_num_pub         = nh.advertise<std_msgs::Int32>("ApriltagSelect",10);
//	Ready();	
	while(ros::ok())
	{
		Menu_Dis();
		std::cin >> mode;
		trash = getchar();
			
		switch(mode)
		{
			case 1:  //Take off mode
				drone->request_sdk_permission_control();
		
				hand_data.data = 2;
				hand_pub.publish(hand_data);
				sleep(1);
				ROS_INFO("Already get permission!\n");
				ROS_INFO("Warning!! Drone will take off after 3s !!");
				sleep(2);
				ROS_INFO("Taking off!!\n");
				drone->takeoff();
				for(i=0;i<50;i++)
				{
					if(i<30)
						drone->attitude_control(0x4b,0,0,1,0);
					else
						drone->attitude_control(0x4b,0,0,0,0);
				usleep(20000);
				}
				std::cout << "Take off done !";
				break;
			case 2:  //Position
				std::cout << "Input X target : ";
				std::cin >> pidx.target;
				std::cout << "Input Y target : ";
				std::cin >> pidy.target;
				std::cout << "Input Z target : ";
				std::cin >> pidz.target;
				std::cout << "Press anykey to exit";
				trash = getchar();
				printf("Please input Q_id:  ");
				std::cin>>Q_num.data;			
				trash = getchar();
				for(i=0;i<5;i++)
				{
					Q_num_pub.publish(Q_num);
					usleep(200);
				}
				do{
					ros::spinOnce();
					vx = pidX.rmg2_pid_controller(pidx);
					vy = pidY.rmg2_pid_controller(pidy);
					vz = pidZ.rmg2_pid_controller(pidz);					  //  vyaw = pidYaw.rmg2_pid_controller(pidyaw);
					printf("vx[%f],vy[%f],vz[%f],vyaw[%f] \n",vx,vy,vz,vyaw);
					drone->velocity_control(1,vx,vy,vz,0);
				pos_rate.sleep();
				}while(!kbhit());
				trash = getchar();
				std::cout << "Position done !";
				break;
			case 10:   //Qpos 
			//	QorL = 0;
				drone->request_sdk_permission_control();
				printf("Please input Q_id:  ");
				std::cin>>Q_num.data;			
				trash = getchar();
				for(i=0;i<5;i++)
				{
					Q_num_pub.publish(Q_num);
					usleep(200);
				}
				C_state.state = 0;
			        for(i=0;i<5;i++)
					{
					camera_select_pub.publish(C_state);
					usleep(200);
					}
					
				std::cout << "Press anykey to exit";
				sleep(1);
				pidz.target = -0.1;
				//trash = getchar();
				do{
					ros::spinOnce();
					vx = pidQX.rmg2_pid_controller(pidqx);
					vy = pidQY.rmg2_pid_controller(pidqy);
					vz = pidZ.rmg2_pid_controller(pidz);
					vyaw = pidQYaw.rmg2_pid_controller(pidqyaw);
					printf("vx[%f],vy[%f],vyaw[%f] \n",vx,vy,vyaw);
					drone->attitude_control(0x4b,vx,vy,vz,vyaw);
					qpos_rate.sleep();
					
				}while(!kbhit());
				trash = getchar();
				std::cout << "Qpos done !";
				break;
			case 8: // Landpos 
			//	QorL = 1;
				drone->request_sdk_permission_control();
				C_state.state = 1;
			        for(i=0;i<5;i++)
					{
					camera_select_pub.publish(C_state);
					usleep(200);
					}
	
				pidz.target = 1.0;
				pidyaw.target = 0;
				std::cout << "Press anykey to exit";
				sleep(1);	
			//	trash = getchar();
				do{
					ros::spinOnce();
					vx = pidQX.rmg2_pid_controller(pidqx);
					vy = pidQY.rmg2_pid_controller(pidqy);
					vyaw = pidYaw.rmg2_pid_controller(pidyaw);
					vz = pidZ.rmg2_pid_controller(pidz);
					printf("vx[%f],vy[%f],vz[%f],vyaw[%f] \n",vx,vy,vz,vyaw);
					drone->attitude_control(0x4b,vx,vy,vz,0);
				landpos_rate.sleep();
				}while(!kbhit());
				trash = getchar();
				std::cout << "Landpos done !";
				break;
			case 5:
				printf("Please input Q_id(1/0/2):  ");
				std::cin>>Q_num.data;			
				trash = getchar();
				for(i=0;i<5;i++)
				{
					Q_num_pub.publish(Q_num);
					usleep(200);
				}

				C_state.state = 0;
			        for(i=0;i<5;i++)
					{
					camera_select_pub.publish(C_state);
					usleep(200);
					}
				if(Q_num.data == 1)
				{
					pidx.target = 3.0;
					pidy.target = -0.8;
					pidz.target = 0.3;
				}
				else if(Q_num.data == 0)
				{
					pidx.target = 1.5;
					pidy.target = -0.3;
					pidz.target = 0.3;
				}
				else 
				{
			     			pidx.target = 1.9;
						pidy.target = -0.5;
						pidz.target = 0.3;
				}
				do{
					ros::spinOnce();
					vx = pidX.rmg2_pid_controller(pidx);
					vy = pidY.rmg2_pid_controller(pidy);
					vz = pidZ.rmg2_pid_controller(pidz);					  //  vyaw = pidYaw.rmg2_pid_controller(pidyaw);
					printf("vx[%f],vy[%f],vz[%f],vyaw[%f] \n",vx,vy,vz,vyaw);
					drone->velocity_control(1,vx,vy,vz,0);
				pos_rate.sleep();
				}while(!kbhit());
				trash = getchar();

				std::cout << "Press anykey to exit";
				sleep(1);
				pidz.target = 0.15;
					
				aim_count = 0;
				do{
					ros::spinOnce();
					vx = pidQX.rmg2_pid_controller(pidqx);
					vy = pidQY.rmg2_pid_controller(pidqy);
					vz = pidZ.rmg2_pid_controller(pidz);
			//		vyaw = pidQYaw.rmg2_pid_controller(pidqyaw);
				if((absd(pidqx.target) < 0.02)&&(absd(pidqy.target) < 0.02))
{
						   aim_count++;
						   printf("count : %d\n",aim_count);
						   if(aim_count == 18)
							break;
}
				else
					aim_count = 0;
					printf("vx[%f],vy[%f],vyaw[%f] \n",pidqx.target,pidqy.target,vyaw);
					drone->attitude_control(0x4b,vx,vy,vz,0);
					qpos_rate.sleep();
					
				}while(!kbhit());
					drone->attitude_control(0x4b,0,0,0,0);
				hand_data.data = 1;
				hand_pub.publish(hand_data);
				printf("Get Trash.....\n");
				trash = getchar();
				

/****************************Jin***Chang************************/
			//	pidx.target = 1.6;
			//	pidy.target = 0.0;
			//	pidz.target = 0.5;
/***************************************************************/
	
/***************************Yuanchang***************************/
		//		pidx.target = -2.0;
		//		pidy.target = 0.5;
		//		pidz.target = 0.4;

/*
				qpos_count = 0;
				Qpos_get = 0;
				do{
					ros::spinOnce();
					vx = pidX.rmg2_pid_controller(pidx);
					vy = pidY.rmg2_pid_controller(pidy);
					vz = pidZ.rmg2_pid_controller(pidz);
				   if(vx<0.1&&vy<0.1)
					pos_count++;
				if(pos_count == 100)
				{
					printf("Select seach dir : \n");
					printf("[1]-------Forward  \n");
					printf("[2]-------Backward \n");
					printf("[3]-------Left     \n");
					printf("[4]-------Right    \n");

					std::cin >> seach_get;
				trash = getchar();
					
					switch(seach_get)
					{	
					  case 1:
						pidx.target += 0.5; break;
					  case 2:
						pidx.target -= 0.5; break;
					  case 3:
						pidy.target -= 0.5; break;
					  case 4:	
						pidy.target += 0.5; break;
					  default: break;	
					}
				       pos_count = 0;
				}
				printf("Count : %d \n",pos_count);
					//printf("vx[%f],vy[%f],vz[%f],vyaw[%f] \n",vx,vy,vz,vyaw);
				drone->velocity_control(1,vx,vy,vz,0);
				pos_rate.sleep();
				}while(!(Qpos_get));
			//	pidz.target = 0.3;


*****************************Jin*****Chang**************************
				pidz.target = 0.4;
********************************************************************
				do{
					ros::spinOnce();
					vx = pidQX.rmg2_pid_controller(pidqx);
					vy = pidQY.rmg2_pid_controller(pidqy);
					vz = pidZ.rmg2_pid_controller(pidz);
					vyaw = pidQYaw.rmg2_pid_controller(pidqyaw);
				if((pidqx.target < pid_acc)&&(pidqy.target < pid_acc))
{
						   aim_count++;
						   printf("count : %d\n",aim_count);
						   if(aim_count == 100)
							pidz.target = 0.10;
	///////////////////////////////////////////////////// Jinchang
						   if(aim_count == 200)
							break;
}
					printf("vx[%f],vy[%f],vyaw[%f] \n",vx,vy,vyaw);
					drone->attitude_control(0x4b,vx,vy,vz,vyaw);
					qpos_rate.sleep();
					
				}while(!kbhit());
				aim_count   = 0;
					drone->attitude_control(0x4b,0,0,0,0);
				hand_data.data = 1;
				hand_pub.publish(hand_data);
				hand_pub.publish(hand_data);
				trash = getchar(); 
			//	Qpos_get = 0;
			//	qpos_count = 0;*/
				break;
			case 6:
				drone->request_sdk_permission_control();
	pidQX.setParam(0.50,0.10,0.10);
	pidQY.setParam(0.50,0.10,0.10);
				C_state.state = 1;
			        for(i=0;i<5;i++)
					{
					camera_select_pub.publish(C_state);
					usleep(200);
					}
				pidx.target = 0;
				pidy.target = 0;
				pidz.target = 1.5;
				Landpos_get = 0;
				landpos_count = 0;
				pos_count   = 0;
				sleep(1);
				do{
					ros::spinOnce();
					vx = pidX.rmg2_pid_controller(pidx);
					vy = pidY.rmg2_pid_controller(pidy);
					vz = pidZ.rmg2_pid_controller(pidz);
							  //  vyaw = pidYaw.rmg2_pid_controller(pidyaw);
				   if(vx<0.1&&vy<0.1)
					pos_count++;
				if(pos_count == 100)
				{
					break;
				}
				printf("Count : %d \n",pos_count);
					//printf("vx[%f],vy[%f],vz[%f],vyaw[%f] \n",vx,vy,vz,vyaw);
				drone->velocity_control(1,vx,vy,vz,0);
				pos_rate.sleep();
				}while(!kbhit());
				pos_count = 0;
				Landpos_get =0;
				while(!Landpos_get)
				{
					
				   if(vx<0.1&&vy<0.1)
					pos_count++;
				  if(pos_count == 100)
					{
					printf("Select seach dir : \n");
					printf("[1]-------Forward  \n");
					printf("[2]-------Backward \n");
					printf("[3]-------Left     \n");
					printf("[4]-------Right    \n");

					std::cin >> seach_get;
				trash = getchar();
					
					switch(seach_get)
					{	
					  case 1:
						pidx.target += 0.5; break;
					  case 2:
						pidx.target -= 0.5; break;
					 case 3:
						pidy.target -= 0.5; break;
					  case 4:	
						pidy.target += 0.5; break;
					  default: break;	
					}
					pos_count = 0;

					}
					vx = pidX.rmg2_pid_controller(pidx);
					vy = pidY.rmg2_pid_controller(pidy);
					vz = pidZ.rmg2_pid_controller(pidz);
					drone->velocity_control(1,vx,vy,vz,0);
					ros::spinOnce();
		
					pos_rate.sleep();
				}
					drone->velocity_control(1,0,0,0,0);
				pidz.target = 1.5;
				do{
					ros::spinOnce();
					vx = pidQX.rmg2_pid_controller(pidqx);
					vy = pidQY.rmg2_pid_controller(pidqy);
					vz = pidZ.rmg2_pid_controller(pidz);
	if((absd(pidqx.target) < pid_acc)&&(absd(pidqy.target) < pid_acc))
{
						   aim_count++;
						   printf("count : %d\n",aim_count);
						   if(aim_count == 5){
							pidz.target = 1.10;
							aim_count = 0;
  							}
/////////////////////////////////////////////////////////Jin//////Chang
						   if(aim_count ==15)
							break;
}
	else
		aim_count = 0;
			//		printf("vx[%f],vy[%f],vz[%f],vyaw[%f] \n",vx,vy,vz,vyaw);
					drone->attitude_control(0x4b,vx,vy,vz,0);
				landpos_rate.sleep();
				}while(!kbhit());
				drone->attitude_control(0x4b,0,0,0,0);
				aim_count = 0;
				drone->landing();
				sleep(3);
				ros::spinOnce();
				Dposx_bias = dp.vector.x;
				Dposy_bias = dp.vector.y;
				Dposz_bias = dp.vector.z;
				printf("Get Trash....\n");	
				trash = getchar();
				
			

				break;
			case 7 :
				std::cout << "Input X Bias : ";
				std::cin >> Dposx_bias;
				std::cout << "Input Y Bias : ";
				std::cin >> Dposy_bias;
				std::cout << "Input Z Bias : ";
				std::cin >> Dposz_bias;
				trash = getchar();
				break;
			case 14:
				hand_data.data = 1;
				hand_pub.publish(hand_data);
				sleep(8);
				std::cout << "Take amo done!";
				break;
			case 13:
				hand_data.data = 2;
				hand_pub.publish(hand_data);
				sleep(8);
				std::cout << "Release amo done!";
				break;
			case 15:  //land
				drone->landing();
				sleep(3);
				std::cout << "Drone already landing";
				break;
			case 4:
				drone->request_sdk_permission_control();
				C_state.state = 1;
			        for(i=0;i<5;i++)
					{
					camera_select_pub.publish(C_state);
					usleep(200);
					}
	
				pidz.target = 1.5;
				pidyaw.target = 0;
				sleep(1);	
			//	trash = getchar();
				do{
					ros::spinOnce();
					vx = pidQX.rmg2_pid_controller(pidqx);
					vy = pidQY.rmg2_pid_controller(pidqy);
					vz = pidZ.rmg2_pid_controller(pidz);
				if((pidqx.target < pid_acc)&&(pidqy.target < pid_acc))
{
						   aim_count++;
						   printf("count : %d\n",aim_count);
						   if(aim_count == 100)
							pidz.target = 0.9;
						   if(aim_count == 250)
							break;
}
			//		printf("vx[%f],vy[%f],vz[%f],vyaw[%f] \n",vx,vy,vz,vyaw);
					drone->attitude_control(0x4b,vx,vy,vz,0);
				landpos_rate.sleep();
				}while(!kbhit());
				aim_count = 0;
				drone->landing();
				break;
			case 9:
				drone->request_sdk_permission_control();
				printf("Done!!\n");
				break;
			case 3:
				drone->request_sdk_permission_control();
				printf("Please input Q_id:  ");
				std::cin>>Q_num.data;			
				trash = getchar();
				for(i=0;i<5;i++)
				{
					Q_num_pub.publish(Q_num);
					usleep(200);
				}
				C_state.state = 0;
			        for(i=0;i<5;i++)
					{
					camera_select_pub.publish(C_state);
					usleep(200);
					}
					
				std::cout << "Press anykey to exit";
				sleep(1);
				pidz.target = 0.15;
					
				aim_count = 0;
				do{
					ros::spinOnce();
					vx = pidQX.rmg2_pid_controller(pidqx);
					vy = pidQY.rmg2_pid_controller(pidqy);
					vz = pidZ.rmg2_pid_controller(pidz);
			//		vyaw = pidQYaw.rmg2_pid_controller(pidqyaw);
				if((absd(pidqx.target) < 0.02)&&(absd(pidqy.target) < 0.02))
{
						   aim_count++;
						   printf("count : %d\n",aim_count);
						   if(aim_count == 18)
							break;
				//			pidz.target = 0.10;
				//		   if(aim_count == 200)
				//			break;
}
				else
					aim_count = 0;
					printf("vx[%f],vy[%f],vyaw[%f] \n",pidqx.target,pidqy.target,vyaw);
					drone->attitude_control(0x4b,vx,vy,vz,0);
					qpos_rate.sleep();
					
				}while(!kbhit());
					drone->attitude_control(0x4b,0,0,0,0);
				hand_data.data = 1;
				hand_pub.publish(hand_data);
				printf("Get Trash.....\n");
				trash = getchar();
				aim_count = 0;
				std::cout << "Release amo done!";
				
				break;
			case 11:
				C_state.state = 0;
			        for(i=0;i<5;i++)
					{
					camera_select_pub.publish(C_state);
					usleep(200);
					}
				break;
			case 12:
				C_state.state = 1;
			        for(i=0;i<5;i++)
					{
					camera_select_pub.publish(C_state);
					usleep(200);
					}
				break;
			case 16:
				drone->request_sdk_permission_control();
				printf("Please input Q_id:  ");
				std::cin>>Q_num.data;			
				trash = getchar();
				for(i=0;i<5;i++)
				{
					Q_num_pub.publish(Q_num);
					usleep(200);
				}

				C_state.state = 0;
			        for(i=0;i<5;i++)
					{
					camera_select_pub.publish(C_state);
					usleep(200);
					}
				switch(Q_num.data)
				{
					case 0:
			     			pidx.target = 1.9;
						pidy.target = -0.5;
						pidz.target = 0.1;break;
					case 1:
			     			pidx.target = 1.6;
						pidy.target = 0.0;
						pidz.target = 0.1;break;
					case 2:
			     			pidx.target = 1.9;
						pidy.target = -0.5;
						pidz.target = 0.3;break;
					default:break;
				}
	pidQX.setParam(0.6,0.06,0.06);
	pidQY.setParam(0.6,0.06,0.06);
	pidQX.setLimit(0.2);
	pidQY.setLimit(0.2);
					qpos_count = 0;
					Qpos_get = 0;
				do{
					ros::spinOnce();
					vx = pidX.rmg2_pid_controller(pidx);
					vy = pidY.rmg2_pid_controller(pidy);
					vz = pidZ.rmg2_pid_controller(pidz);
				   if(vx<0.1&&vy<0.1)
					pos_count++;
				if(pos_count == 100)
				{
					printf("Select seach dir : \n");
					printf("[1]-------Forward  \n");
					printf("[2]-------Backward \n");
					printf("[3]-------Left     \n");
					printf("[4]-------Right    \n");

					std::cin >> seach_get;
				trash = getchar();
					
					switch(seach_get)
					{	
					  case 1:
						pidx.target += 0.5; break;
					  case 2:
						pidx.target -= 0.5; break;
					  case 3:
						pidy.target -= 0.5; break;
					  case 4:	
						pidy.target += 0.5; break;
					  default: break;	
					}
				       pos_count = 0;
				}
				printf("pos Count : %d \n",pos_count);
					//printf("vx[%f],vy[%f],vz[%f],vyaw[%f] \n",vx,vy,vz,vyaw);
				drone->velocity_control(1,vx,vy,vz,0);
				pos_rate.sleep();
				}while(!(Qpos_get));
				printf("Gei Qpos!\n");
				aim_count   = 0;
				do{
					ros::spinOnce();
					vx = pidQX.rmg2_pid_controller(pidqx);
					vy = pidQY.rmg2_pid_controller(pidqy);
					vz = pidZ.rmg2_pid_controller(pidz);
			//		vyaw = pidQYaw.rmg2_pid_controller(pidqyaw);
				if((absd(pidqx.target) < 0.05)&&(absd(pidqy.target)) < 0.05)
{
						   aim_count++;
						   printf("count : %d\n",aim_count);
						   if(aim_count == 20)
							break;
}
					printf("vx[%f],vy[%f],vyaw[%f] \n",vx,vy,vyaw);
					drone->attitude_control(0x4b,vx,vy,vz,0);
					qpos_rate.sleep();
					
				}while(!kbhit());
				aim_count   = 0;
					drone->attitude_control(0x4b,0,0,0,0);
				hand_data.data = 1;
				hand_pub.publish(hand_data);
				printf("Get Trash!\n");
				trash = getchar();

				pidQX.setParam(0.50,0.10,0.10);
				pidQY.setParam(0.50,0.10,0.10);
				C_state.state = 1;
			        for(i=0;i<5;i++)
					{
					camera_select_pub.publish(C_state);
					usleep(200);
					}
				pidx.target = 0;
				pidy.target = 0;
				pidz.target = 1.5;
				Landpos_get = 0;
				landpos_count = 0;
				pos_count   = 0;
				sleep(1);
				printf("Get Trash...\n");
				trash = getchar();
				do{
					ros::spinOnce();
					vx = pidX.rmg2_pid_controller(pidx);
					vy = pidY.rmg2_pid_controller(pidy);
					vz = pidZ.rmg2_pid_controller(pidz);
							  //  vyaw = pidYaw.rmg2_pid_controller(pidyaw);
				   if(vx<0.1&&vy<0.1)
					pos_count++;
				if(pos_count == 100)
				{
					break;
				}
				printf("Count : %d \n",pos_count);
					//printf("vx[%f],vy[%f],vz[%f],vyaw[%f] \n",vx,vy,vz,vyaw);
				drone->velocity_control(1,vx,vy,vz,0);
				pos_rate.sleep();
				}while(!kbhit());
				pos_count = 0;
				Landpos_get =0;
				while(!Landpos_get)
				{
					
				   if(vx<0.1&&vy<0.1)
					pos_count++;
				  if(pos_count == 100)
					{
					printf("Select seach dir : \n");
					printf("[1]-------Forward  \n");
					printf("[2]-------Backward \n");
					printf("[3]-------Left     \n");
					printf("[4]-------Right    \n");

					std::cin >> seach_get;
				trash = getchar();
					
					switch(seach_get)
					{	
					  case 1:
						pidx.target += 0.5; break;
					  case 2:
						pidx.target -= 0.5; break;
					 case 3:
						pidy.target -= 0.5; break;
					  case 4:	
						pidy.target += 0.5; break;
					  default: break;	
					}
					pos_count = 0;

					}
					vx = pidX.rmg2_pid_controller(pidx);
					vy = pidY.rmg2_pid_controller(pidy);
					vz = pidZ.rmg2_pid_controller(pidz);
					drone->velocity_control(1,vx,vy,vz,0);
					ros::spinOnce();
		
					pos_rate.sleep();
				}
					drone->velocity_control(1,0,0,0,0);
				pidz.target = 1.5;
				do{
					ros::spinOnce();
					vx = pidQX.rmg2_pid_controller(pidqx);
					vy = pidQY.rmg2_pid_controller(pidqy);
					vz = pidZ.rmg2_pid_controller(pidz);
	if((absd(pidqx.target) < pid_acc)&&(absd(pidqy.target) < pid_acc))
{
						   aim_count++;
						   printf("count : %d\n",aim_count);
						   if(aim_count == 5){
							pidz.target = 1.00;
							aim_count = 0;
							}
						   if(aim_count == 20)
							break;
}
	else
		aim_count = 0;
			//		printf("vx[%f],vy[%f],vz[%f],vyaw[%f] \n",vx,vy,vz,vyaw);
					drone->attitude_control(0x4b,vx,vy,vz,0);
				landpos_rate.sleep();
				}while(!kbhit());
				drone->attitude_control(0x4b,0,0,0,0);
				aim_count = 0;
				drone->landing();
				sleep(4);
				ros::spinOnce();
				Dposx_bias = dp.vector.x;
				Dposy_bias = dp.vector.y;
				Dposz_bias = dp.vector.z;
				break;
			case 17:
				drone->request_sdk_permission_control();
	pidQX.setParam(0.6,0.06,0.06);
	pidQY.setParam(0.6,0.06,0.06);
	pidQX.setLimit(0.4);
	pidQY.setLimit(0.4);
				C_state.state = 1;
			        for(i=0;i<5;i++)
					{
					camera_select_pub.publish(C_state);
					usleep(200);
					}
				pidx.target = 0;
				pidy.target = 0;
				pidz.target = 1.4;
				Landpos_get = 0;
				landpos_count = 0;
				pos_count   = 0;
				sleep(1);
				do{
					ros::spinOnce();
					vx = pidX.rmg2_pid_controller(pidx);
					vy = pidY.rmg2_pid_controller(pidy);
					vz = pidZ.rmg2_pid_controller(pidz);
							  //  vyaw = pidYaw.rmg2_pid_controller(pidyaw);
				   if(vx<0.1&&vy<0.1)
					break;
				drone->velocity_control(1,vx,vy,vz,0);
				pos_rate.sleep();
				}while(!kbhit());
				drone->velocity_control(1,0,0,0,0);
				aim_count = 0;
				do{
					ros::spinOnce();
					vx = pidQX.rmg2_pid_controller(pidqx);
					vy = pidQY.rmg2_pid_controller(pidqy);
					vz = pidZ.rmg2_pid_controller(pidz);
				if(((absd(pidqx.target) < pid_acc)&&(absd(pidqy.target) < pid_acc))  )
{
						   aim_count++;
						   printf("count : %d\n",aim_count);
						   if(aim_count == 10)
							pidz.target = 1.10;
/////////////////////////////////////////////////////////Jin//////Chang
						   if(aim_count == 50)
							break;
}
			//		printf("vx[%f],vy[%f],vz[%f],vyaw[%f] \n",vx,vy,vz,vyaw);
					drone->attitude_control(0x4b,vx,vy,vz,0);
				landpos_rate.sleep();
				}while(1);
				aim_count = 0;
				drone->landing();
				break;
				
				
		
			default : break;	
	

				
		}	
		mode = 0;
				Qpos_get = 0;
				qpos_count = 0;
		
	}
	
		
}
