#include "ros/ros.h"

#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/PointCloud2.h>
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"
#include"tf/tf.h"


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "math.h"
#include <iostream>
#include <fstream>

//#define inc 0.00290277777
float inc=0.00290277777;
class pointCl{
	public :
		float yaw=0;
		float x=0;
		float y=0;
		float z=0;
		float lid_yaw=0;

		ros::NodeHandle n;
		ros::Publisher pubPc 		= n.advertise<sensor_msgs::PointCloud2>("point_cloud", 1,this);
		ros::Subscriber laser_sub 	= n.subscribe("/laser/scan", 1, &pointCl::laserScancallBack,this);
 		ros::Subscriber odom_sub	= n.subscribe("/odom", 1 , &pointCl::odomCallback,this);
 		ros::Subscriber pos_sub		= n.subscribe("/robot_wheels/joint_lidar_position_controller/state", 1 , &pointCl::posCallback,this);
		sensor_msgs::PointCloud2 cloud_msg;
		pcl::PointCloud<pcl::PointXYZRGB> cloud;


		pointCl();
		void laserScancallBack(const sensor_msgs::LaserScan::ConstPtr&);
		void posCallback(const control_msgs::JointControllerState::ConstPtr&);
		void odomCallback(const nav_msgs::Odometry::ConstPtr&);
};
pointCl::pointCl() = default;

void pointCl::posCallback(const control_msgs::JointControllerState::ConstPtr& msg){
	this->lid_yaw=msg->process_value;
}


void pointCl::laserScancallBack(const sensor_msgs::LaserScan::ConstPtr& msg){
	float dummy_x=0,dummy_y=0,dummy_z=0;
	std::uint8_t r=255;
	std::uint8_t 	g=0;
	std::uint8_t b=0;
	pcl::PointCloud<pcl::PointXYZRGB> cloud_;
	for(int i=0;i<570;i+=8){
				if(msg->ranges[i]!=std::numeric_limits<double>::infinity()){
					

					dummy_x =  (msg->ranges[i] * sin(1.5708 - i * inc) + 0.75)*cos(1.5708 + this->yaw + this->lid_yaw) + this->x;
					dummy_y = -(msg->ranges[i] * sin(1.5708 - i * inc) + 0.75)*sin(1.5708 + this->yaw + this->lid_yaw) + this->y;
					dummy_z = -(msg->ranges[i] * cos(1.25   + i * inc) - 0.3) + this->z;
					
				//	myfile<<(trunc(obj.x*100))/100<<';'<<(trunc(obj.y*100)/100)<<';'<<(trunc(obj.z*100)/100)<<'\n';
					cloud_.push_back (pcl::PointXYZRGB (dummy_x, dummy_y, dummy_z,r,g,b)); 
					
				}
			}
	//pcl_conversions::moveFromPCL(this->cloud, this->cloud_msg);

	//pcl_conversions::toPCL(ros::Time::now(), this->cloud.header.stamp);
	pcl::toROSMsg(cloud_,this->cloud_msg);
	this->cloud_msg.header.frame_id = "odom";
  	this->cloud_msg.height = cloud.width = 1;
	this->pubPc.publish(this->cloud_msg);
}
void pointCl::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){

	this->yaw=-(tf::getYaw(msg->pose.pose.orientation)+1.5708);
	this->x=msg->pose.pose.position.x;
	this->y=msg->pose.pose.position.y;
	this->z=msg->pose.pose.position.z;

}

int main(int argc, char **argv){
    std::ofstream myfile;
	ros::init(argc, argv, "plot");
	pointCl a=pointCl();
 	ros::Rate loop_rate(30);

 	myfile.open ("/home/seif/Desktop/mesh.txt");
 	
	while (ros::ok()){

	    while (a.pubPc.getNumSubscribers() < 1)
	    {
	      if (!ros::ok())
	      {
	        return 0;
	      };

	      ROS_WARN_ONCE("Please create a subscriber to the marker");
	      sleep(1);
	    }; 

	    ros::spinOnce();

	    loop_rate.sleep();
  }

  myfile.close();
  return 0;



}
