#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <dji_sdk/GlobalPosition.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <apriltags/AutoChargeMsg.h>

#include "rmg2_pid_controller.h"
/*
topic:
	/guidance/imu  Yaw
	/dji_sdk/global_positon height  Chaoshengbo
	/motion_data   Height
		
*/


 using namespace DJI::onboardSDK;
 using namespace std;

 rmg2::rmg2_pid pidX,pidY,pidZ,pidH,pidQX,pidQY;
 float Yaw=0;
 float UHeight=0;
 int   Qflag = 1;

 ros::Subscriber drone_pos_sub;
 ros::Subscriber drone_yaw_sub;
 ros::Subscriber drone_absoH_sub;
 ros::Subscriber drone_relaH_sub;

 ros::Subscriber Qpos_sub;
 void dpos_callback(const geometry_msgs::Vector3Stamped& dpos)
{
	pidX.feedback = dpos.vector.x;
        pidY.feedback = dpos.vector.y;
      	ROS_INFO("X.feedback is %f , Y.feedback is %f  \n",pidX.feedback,pidY.feedback); 
	
		
}
 void drone_yaw_callback(const geometry_msgs::TransformStamped& dyaw)
{
	Yaw = dyaw.transform.rotation.z;
	printf("Yaw is : %f  \n",Yaw);	
}
 void drone_absoH_callback(const geometry_msgs::Vector3Stamped& dabH)
{
	//pidH.feedback = -(dabH.vector.z);
        //rintf("Height is %f   \n",pidH.feedback);	
}
 void drone_relaH_callback(const dji_sdk::GlobalPosition& UH)
{
	pidH.feedback = UH.height;
	//UHeight = UH.height;
	printf("UHeight is %f   \n",pidH.feedback);
	
}
 void Qpos_callback(const apriltags::AutoChargeMsg& Qpos)
{
	pidQX.target = Qpos.X_head;
	pidQY.target = Qpos.Y_right;
	pidQX.header.frame_id = Qpos.id;
        Qflag = 1;
	
}

 int main(int argc, char** argv)
{
      
 float vx=0;
 float vy=0;
 float vz=0;
 float vh=0;
 bool ack;
 pidX.limit           = 0.4f;
 pidX.Kp              = 1.0f;
 pidX.Ki	       = 0.1f;
 pidX.Kd	       = 0.1f;
 //pidX.target          = 1.0f;
 
 pidY.limit		= 0.4;
 pidY.Kp		= 1.0;
 pidY.Ki 		= 0.1;
 pidY.Kd		= 0.1;


 pidZ.limit		= 0.4;
 pidZ.Kp		= 1.0;
 pidZ.Ki 		= 0.1;
 pidZ.Kd		= 0.1;

 pidH.target            = 2.5;
 pidH.limit		= 0.8;
 pidH.Kp		= 1.0;
 pidH.Ki 		= 0.1;
 pidH.Kd		= 0.1;

 pidQX.feedback         = 0;    
 pidQY.feedback         = 0;

 pidQX.Kp		=0.8;   // pidQX.Kp = 1.0 
 pidQY.Kp		=0.8;

 pidQX.Ki		=0.06;  // pidQY.Ki = 0.06 
 pidQY.Ki		=0.06;

 pidQX.Kd		=0.06;  // pidX.Kd  = 0.06
 pidQY.Kd 		=0.06;

 pidQX.limit		=1.0;   // pidX.limit = 0.4
 pidQY.limit		=1.0;
// pidY.target            = -2.0;

	ros::init(argc,argv , "PID_SET");

        int temp32;

	ros::NodeHandle nh;

	DJIDrone* test_drone = new DJIDrone(nh);

	drone_pos_sub = nh.subscribe("/guidance/motion_data",10,dpos_callback);

  	drone_yaw_sub = nh.subscribe("/guidance/imu",20,drone_yaw_callback);
  	drone_absoH_sub = nh.subscribe("/guidance/motion_data",10,drone_absoH_callback);
  	drone_relaH_sub = nh.subscribe("/dji_sdk/global_position",10,drone_relaH_callback);
        ros::Rate pid_rate(50);
	ros::Rate Qpos_rate(20);

 	 Qpos_sub      = nh.subscribe("/chargedata_aircraft",10,Qpos_callback);

        while(ros::ok())
	{  
        
	    printf("Please input 1 to request control !\n");
	    std::cin >> temp32;
	    
		if(temp32 == 1)
			 test_drone->request_sdk_permission_control();
		sleep(1);
		ROS_INFO("Already get control permission! perpare to take off\n");
                ROS_INFO("Please input 2 to takeoff! \n");
	      
          	 
		
		while(!(std::cin>> temp32))
		{
		      std::cin.clear();
		      std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
		      std::cout << "Invalid input. Try again: ";
		}
		if(temp32 == 2)
		{
			
		printf("Taking off!!\n");
		    test_drone->takeoff();
		    sleep(3);	
		    for(int i=0;i<200;i++)
			{
		
			if(i<150)
				test_drone->attitude_control(0x4b,0,0,1,0);
			else
				test_drone->attitude_control(0x4b,0,0,0,0);
			usleep(20000);
    			}
		}
		printf("The drone is already took off!!\n");
	      while(!(temp32 == 0))
		{
		  
               

		 printf("Position Mode : 3-------Flowing Q Mode : 4 \n");
		std::cin>>temp32;
		if(temp32 == 3)
		{
		
            	printf("Please input X target : ");
	    	std::cin >> pidX.target;
	    	printf("Please input y target : ");
	    	std::cin >> pidY.target;
                
	    	printf("Please input speed limit : ");
	        std::cin >> pidX.limit;
            	pidY.limit = pidX.limit;		
	    	printf("The test data is : targetX[%f], targetY[%f],limit[%f]\n",pidX.target,pidY.target,pidX.limit);
		sleep(3);
		printf("please input 9 to start mission:");
		std::cin>>temp32;
		while(pidX.feedback>(pidX.target+0.02)||pidX.feedback<(pidX.target-0.02)||pidY.feedback>(pidY.target+0.02)||pidY.feedback<(pidY.target-0.02))	
			{
			   ros::spinOnce();
			   vx = rmg2_Xpid_control(pidX);
			   vy = rmg2_Ypid_control(pidY);
		           
			   test_drone->velocity_control(1,vx,vy,0,0);
			   printf(" vx[%f]  ,  vy[%f]  \n",vx,vy);
			   pid_rate.sleep();	
			}
			   test_drone->velocity_control(1,0,0,0,0);
			sleep(1);
			
			
		}
		else if(temp32 == 4)
		{
		    printf("please input 9 to start mission:");
			std::cin >> temp32;
		
		     while(ros::ok())
			{
			  Qflag = 0;
			  ros::spinOnce();
		//	  vh = rmg2_Hpid_control(pidH);
			  vx = rmg2_Xpid_control(pidQX);
			  vy = rmg2_Ypid_control(pidQY);
			
			  test_drone->velocity_control(1,vx,vy,0,0);
			  printf(" vx[%f]  ,  vy[%f]  \n",vx,vy);
			  Qpos_rate.sleep();	
			}
	       
			test_drone->velocity_control(1,0,0,0,0);
			sleep(1);
		}

               }
		
		printf("if you want landing input 5:");
		std::cin >> temp32;
		if(temp32 == 5)
			{
			test_drone->landing();
			return 0;
		}	
	}

		
}
