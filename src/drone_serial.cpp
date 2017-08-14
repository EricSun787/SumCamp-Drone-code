#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <rmg2/drone_serial_data.h>

char sendbuff[3] = {0};
rmg2::drone_serial_data control_data;
int i;
boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev,"/dev/ttyTHS2");


void drone_hand_control_callback(const rmg2::drone_serial_data& msg)
{
	control_data = msg;

	ROS_INFO("GET DATA %d \n",control_data.data);
	if(control_data.data == 1)
		sendbuff[1] = 0x31;
	else if(control_data.data == 2)
		sendbuff[1] = 0x32;

	for(i=0;i<20;i++)
		{
		usleep(20);
		ROS_INFO("Sending!\n");
		boost::asio::write(sp,boost::asio::buffer(sendbuff,3));
		}
	
}

int main(int argc,char** argv)
{
	
	sp.set_option(boost::asio::serial_port::baud_rate(115200));
	sp.set_option(boost::asio::serial_port::flow_control());
	sp.set_option(boost::asio::serial_port::parity());
	sp.set_option(boost::asio::serial_port::stop_bits());
	sp.set_option(boost::asio::serial_port::character_size(8));
	sendbuff[0] = 0x23;
	sendbuff[1] = 0x30;
	sendbuff[2] = 0x2A;
	
	
	ros::init(argc,argv,"Drone_serial_write");

	ros::NodeHandle n;
	
	ros::Subscriber serial_sub = n.subscribe("/rmg2/drone_hand_control",5,drone_hand_control_callback);
	ros::Rate rate(300);
	while(ros::ok())
	{
	//        sendbuff[1] = 0x30;
		ros::spinOnce();
		//boost::asio::write(sp,boost::asio::buffer(sendbuff,3));
		rate.sleep();
		
	}

	
	
}
