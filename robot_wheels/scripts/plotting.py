#!/usr/bin/env python3

import rospy
import open3d as o3d
import numpy as np
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point
from control_msgs.msg import JointControllerState
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from math import floor
from open3d_ros_helper import open3d_ros_helper as orh
import colorsys
yaw=0.0
x=0.0
y=0.0

inc=0.00290277777
lid=0.0

marker=Marker()

f=open("pc.txt", "w")
points=np.zeros((1,3))

def odomCallback(msg):
    global x
    global y
    global yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (a, b, c) = euler_from_quaternion (orientation_list)
    c=-c-1.5708
    yaw=c
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y


def posCallback(msg):
    global lid
    lid=msg.process_value

def laserCallback(data):


    global points


    i=0
    while(i<570):
        if not (np.isposinf((data.ranges[i]))):
            point=Point()

            point.x=(data.ranges[i]*np.sin(1.5708-i*inc)+0.75)*np.cos(1.5708+yaw+lid)+x

            point.y =-((data.ranges[i]*np.sin(1.5708-i*inc)+0.75)*np.sin(1.5708+yaw+lid)-y)
            point.z= -(data.ranges[i]*np.cos(i*inc+1.25)-0.3)
            point.x=(floor(point.x*100)/100)
            point.y=(floor(point.y*100)/100)
            point.z=(floor(point.z*100)/100)
            points=np.concatenate((points,[[point.x,point.y,point.z]]),axis=0)
            f.write(str(point.x)+" "+str(point.y)+" "+str(point.z)+"\n")

        i+=8

if __name__ == '__main__' :
    rospy.init_node('plotting',anonymous=True)
    rospy.Subscriber("/laser/scan", LaserScan, laserCallback)
    rospy.Subscriber("/odom", Odometry, odomCallback)
    rospy.Subscriber("/robot_wheels/joint_lidar_position_controller/state", JointControllerState, posCallback)
    pcl_pub = rospy.Publisher("/pointcloud", PointCloud2,queue_size=10)
    marker_pub=rospy.Publisher("/visualization_marker",Marker,queue_size=10)
    rate=rospy.Rate(1)
    i=0
    while(points.shape[0]<10):
        pass

    try:

        while(not rospy.is_shutdown()):
            copy=points
            pcd = o3d.geometry.PointCloud()

            max_height=np.max(copy[:,2])
            colors=[]
            for line in copy:

                H=line[2]/max_height

                color = colorsys.hsv_to_rgb(H, 0.5, 0.5)
                colors.append(color)


            colors=np.array(colors)

            pcd.points = o3d.utility.Vector3dVector(copy)
            pcd.colors = o3d.utility.Vector3dVector(colors)


            downpcd = pcd.voxel_down_sample(voxel_size=0.9)
            downpcd , _ =downpcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=2.0)

            rospc = orh.o3dpc_to_rospc(downpcd)
            rospc.header.frame_id="odom"
            rospc.header.stamp=rospy.get_rostime()
            pcl_pub.publish(rospc)
            rate.sleep()



    except KeyboardInterrupt :
        filename="finalpointcloud.ply"
        print("saving to "+filename)
        o3d.io.write_point_cloud(filename, downpcd)
        print("shutdown")
