#include <ros/ros.h>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <sensor_msgs/PointCloud2.h>

using namespace std;

void filter(const sensor_msgs::PointCloud2ConstPtr& input){
	static int count;
	static int num;
	char file_name[100] = {}; 
	pcl::PointCloud<pcl::PointXYZ>::Ptr raw_pc (new pcl::PointCloud<pcl::PointXYZ>);

	if( (num%20) == 0){
		//ros PointCloud2 -> PCL
		pcl::fromROSMsg(*input, *raw_pc);
		//save pcd file
		// sprintf(file_name, "/home/amsl/vel_pcd/%d.pcd", count++);
		sprintf(file_name, "/home/amsl/sq_pcd/%d.pcd", count++);
		pcl::io::savePCDFileBinary(file_name, *raw_pc);
		cout << "save pcd file " << file_name << endl;
		if(num >= 20){
			num = 1;
		}
	}
	num++;
}

void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
	filter(msg);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "save_PointCloud2pcd");
	ros::NodeHandle n;

	//ros::Subscriber pc_sub = n.subscribe("/rm_ground", 1, pcCallback);
	ros::Subscriber pc_sub = n.subscribe("/sq_lidar/obstacles", 1, pcCallback);
	//ros::Subscriber pc_sub = n.subscribe("/velodyne_points", 1, pcCallback);

	ros::spin();

	return 0;
}
