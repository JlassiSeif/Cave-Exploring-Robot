#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"
#include "nav_msgs/Odometry.h"
#include"tf/tf.h"
#include <iostream>
#include <fstream>
#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"


// visualization_msgs::MarkerArray ma;


class markerClass{
    public :
        visualization_msgs::Marker marker;
        uint32_t shape;
        geometry_msgs::Point obj;
        float yaw;
        float x;
        float y;
        float inc;
        float lid;
        std::ofstream myfile;
        ros::NodeHandle n;
        ros::Publisher marker_pub;
	    ros::Subscriber laser_sub;
 	    ros::Subscriber odom_sub;
 	    ros::Subscriber pos_sub;

        markerClass();
        void posCallback(const control_msgs::JointControllerState::ConstPtr&);
        void odomCallback(const nav_msgs::Odometry::ConstPtr&);
        void laserScancallBack(const sensor_msgs::LaserScan::ConstPtr&);

};

markerClass::markerClass(){
    this->shape = visualization_msgs::Marker::CUBE_LIST;
    this->x=0;
    this->y=0;
    this->yaw=0;
    this->lid=0;
    this->inc=0.00290277777;
    this->marker.header.frame_id = "odom";

    this->marker.ns = "cubes";
    this->marker.type = this->shape;
    this->marker.action = visualization_msgs::Marker::ADD;
    this->marker.pose.orientation.x = 0.0;
    this->marker.pose.orientation.y = 0.0;
    this->marker.pose.orientation.z = 0.0;
    this->marker.pose.orientation.w = 1.0;
    this->marker.scale.x = 0.03;
    this->marker.scale.y = 0.03;
    this->marker.scale.z = 0.03;
    this->marker.color.r = 0.0f;
    this->marker.color.g = 1.0f;
    this->marker.color.b = 0.0f;
    this->marker.color.a = 1.0;

    this->marker.id = 1;

    

 	this->marker_pub = this->n.advertise<visualization_msgs::Marker>( "visualization_marker", 1,this);
	this->laser_sub = this->n.subscribe("/laser/scan", 1, &markerClass::laserScancallBack,this);
 	this->odom_sub=this->n.subscribe("/odom", 1 , &markerClass::odomCallback,this);
 	this->pos_sub=this->n.subscribe("/robot_wheels/joint_lidar_position_controller/state", 1 , &markerClass::posCallback,this);
};
void markerClass::posCallback(const control_msgs::JointControllerState::ConstPtr& msg){
	this->lid=msg->process_value;
};


void markerClass::laserScancallBack(const sensor_msgs::LaserScan::ConstPtr& msg){
        this->marker.header.stamp = ros::Time::now();
        this->marker.lifetime = ros::Duration();

    for(int i=0;i<570;i+=8){
                if(msg->ranges[i]!=std::numeric_limits<double>::infinity()){
                    
                    this->obj.x=(msg->ranges[i]*sin(1.5708-i*inc)+0.75)*cos(1.5708+yaw+lid)+x;

                    this->obj.y =-((msg->ranges[i]*sin(1.5708-i*inc)+0.75)*sin(1.5708+yaw+lid)-y);
                    this->obj.z= -(msg->ranges[i]*cos(i*inc+1.25)-0.3);
                    //myfile<<(trunc(obj.x*100))/100<<';'<<(trunc(obj.y*100)/100)<<';'<<(trunc(obj.z*100)/100)<<'\n';
                    std::cout<<obj<<std::endl;

                    this->marker.points.push_back(obj);
                }
            }
	
};
void markerClass::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){

    this->yaw=-(tf::getYaw(msg->pose.pose.orientation)+1.5708);
    this->x=msg->pose.pose.position.x;
    this->y=msg->pose.pose.position.y;

};

int main(int argc, char **argv){

	ros::init(argc, argv, "plot");
    ros::Rate loop_rate(5);
    markerClass a=markerClass();
 	a.myfile.open ("/home/seif/Desktop/mesh.txt");
 	while (ros::ok()){
	    while (a.marker_pub.getNumSubscribers() < 1)
	    {
	      if (!ros::ok())
	      {
	        return 0;
	      }
	      ROS_WARN_ONCE("Please create a subscriber to the marker");
	      sleep(1);
	    } 
		 
		a.marker_pub.publish(a.marker);
	    ros::spinOnce();

	    loop_rate.sleep();
  }

  a.myfile.close();
  return 0;



}