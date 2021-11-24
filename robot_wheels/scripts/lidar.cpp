#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"

std_msgs::Float64 err;
std_msgs::Float64 com;

void posCallback(const control_msgs::JointControllerState::ConstPtr& msg){
	err.data=msg->process_value;
	if(err.data>6.19){
		com.data=0;
		std::cout<<"lena"<<std::endl;
	}
	else if(err.data<0.1){
		com.data=6.28;
		std::cout<<"houni"<<std::endl;
	}
}
int main(int argc, char **argv){
	ros::init(argc, argv, "lidar");
	ros::NodeHandle n;
 	ros::Rate loop_rate(1);
	ros::Publisher lidar_pub = n.advertise<std_msgs::Float64>( "/robot_wheels/joint_lidar_position_controller/command", 1);
	ros::Subscriber pos_sub=n.subscribe("/robot_wheels/joint_lidar_position_controller/state", 1 , posCallback);
 	while (ros::ok()){

	    ros::spinOnce();
	    //lidar_pub.publish(pos);
	    loop_rate.sleep();
		lidar_pub.publish(com);

  }

  return 0;



}