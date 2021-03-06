#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/thread.hpp>

#include <tf/transform_listener.h>

using namespace std;

class lidarLCL
{
	private:
		ros::Time t;
		tf::TransformListener tflistener;
		ros::Publisher lidar_pub;
		ros::Subscriber lidar_sub;

	public:
		lidarLCL(ros::NodeHandle& n);
		void lidarCallback(const sensor_msgs::PointCloud2& pc2_msg);
};

lidarLCL::lidarLCL(ros::NodeHandle& n){
	// lidar_sub = n.subscribe("/dynamic_cluster/centroid", 1, &lidarLCL::lidarCallback, this);
	lidar_sub = n.subscribe("/cluster/centroid", 1, &lidarLCL::lidarCallback, this);
	lidar_pub = n.advertise<sensor_msgs::PointCloud2>("/dynamic_cluster/centroid/lcl", 1);
}

void lidarLCL::lidarCallback(const sensor_msgs::PointCloud2& pc2_msg){
	cout << "	lidar Subscribed !:" << endl;
	sensor_msgs::PointCloud pc_msg;
	sensor_msgs::PointCloud pc_global;
	sensor_msgs::PointCloud2 pc2_global;

	t = pc2_msg.header.stamp;
	// PointCloud2 -> PointCloud
	sensor_msgs::convertPointCloud2ToPointCloud(pc2_msg, pc_msg);
	pc_msg.header.frame_id = "/centerlaser_";
	pc_msg.header.stamp = t;
	// 点群を/base_link座標 -> /map座標へ
	try{
		tflistener.waitForTransform("/odom", "/centerlaser_", t, ros::Duration(1.0));
		tflistener.transformPointCloud("/odom", t, pc_msg, "/centerlaser_", pc_global);
		// PointCloud -> PointCloud2
		sensor_msgs::convertPointCloudToPointCloud2(pc_global, pc2_global);
		pc2_global.header.frame_id = "/odom";
		lidar_pub.publish(pc2_global);
	}catch(tf::TransformException ex){
		ROS_ERROR("%s\n", ex.what());
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "centerlaser2odom_pos");
	ros::NodeHandle n;

	lidarLCL llcl(n);

	ros::spin();

	return 0;
}
