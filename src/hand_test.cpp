#include <ros/ros.h>
#include <rmg2/drone_serial_data.h>

rmg2::drone_serial_data send_data;
int temp32;

using namespace std;

int main(int argc,char** argv)
{

	ros::init(argc,argv,"test_serial");
	ros::NodeHandle test;
	
	ros::Publisher test_pub = test.advertise<rmg2::drone_serial_data>("/rmg2/drone_hand_control",5);

 	while(ros::ok())
		{
			send_data.data = 0;
			printf("Please input  command: ");
			std::cin >> send_data.data;

                        test_pub.publish(send_data);
	
			
		}	
}
