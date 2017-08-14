#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <dji_sdk/RCChannels.h>


 ros::Subscriber d_pos_sub;
 ros::Subscriber c_signal_sub;
 ros::Publisher  d_pos_pub;
 
 dji_sdk::RCChannels  c_signal;

void DPos_callback(const geometry_msgs::Vector3Stamped& Dpos)
{
       //	geometry_msgs::Vector3Stamped  d_pos;
	geometry_msgs::Pose     d_pos;

	//d_pos.header.frame_id = "d_pos";
	//d_pos.header.stamp    = ros::Time::now();

	//d_pos.vector.x        = Dpos.vector.x;
	//d_pos.vector.y	      = Dpos.vector.y;
	d_pos.position.x        = Dpos.vector.x;
	d_pos.position.y	= Dpos.vector.y;

	if(c_signal.gear == -4545)
	{
		d_pos.position.z = 1;
                ROS_INFO("Qpos is X:%f  Y:%f  \n",d_pos.position.x,d_pos.position.y);
	        d_pos_pub.publish(d_pos);
		sleep(1);
	}
	else
	{
		d_pos.position.z = 0;
//		d_pos_pub.publish(d_pos);
	}
	
}
void c_signal_callback(const dji_sdk::RCChannels& signal)
{
	c_signal = signal;
}

int main(int argc , char** argv)
{
	ros::init(argc,argv,"transposNode");
	
	ros::NodeHandle tpnode;
	
	c_signal_sub = tpnode.subscribe("/dji_sdk/rc_channels",1,c_signal_callback);

	d_pos_sub = tpnode.subscribe("/guidance/motion_data",1000,DPos_callback);

//	d_pos_pub = tpnode.advertise<geometry_msgs::Vector3Stamped>("/Transpos/Qpos",1);
        d_pos_pub = tpnode.advertise<geometry_msgs::Pose>("/Transpos/Qpos",1);
        ros::spin();
}
