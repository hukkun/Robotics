#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/pcl_base.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

// pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

ros::Publisher pub_pc;

using namespace std;

void lcl_callback(sensor_msgs::PointCloud2::Ptr msg)
{
	// pcl::fromROSMsg(msg, *cloud);
	ros::Time tm = msg->header.stamp;

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(*msg, *cloud);
	// cout<<"sub_lcl"<<endl;
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());

	// pcl::io::loadPCDFile ("d_kan_sq1_indoor_success.pcd", *cloud);

	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
	ne.setInputCloud (cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	pcl::PointCloud<pcl::PointXYZINormal>::Ptr output (new pcl::PointCloud<pcl::PointXYZINormal>);
	// pcl::PointCloud<pcl::PointXYZINormal> output;

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.2);

	// Compute the features
	ne.compute (*cloud_normals);

	size_t size = cloud->points.size();
	pcl::PointXYZINormal tmp;

	for(size_t i=0; i<size; i++){
		tmp.x = cloud->points[i].x;
		tmp.y = cloud->points[i].y;
		tmp.z = cloud->points[i].z;
		tmp.intensity = cloud->points[i].intensity;
		tmp.normal_x = cloud_normals->points[i].normal_x;
		tmp.normal_y = cloud_normals->points[i].normal_y;
		tmp.normal_z = cloud_normals->points[i].normal_z;
		tmp.curvature = cloud_normals->points[i].curvature;
		// output.points.push_back(tmp);
		output->points.push_back(tmp);
	}

	sensor_msgs::PointCloud2 pc_;
	pcl::toROSMsg(*output, pc_);
	output->points.clear();
	pc_.header.frame_id = "/centerlaser_";
	//ros_pc_n.header.stamp = ros::Time::now();
	pc_.header.stamp = tm;

	pub_pc.publish(pc_);

	// cloud_normals->points.size () should have the same size as the input cloud->points.size ()*

	// pcl::io::savePCDFileBinary("d_kan_normal.pcd", output);
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "Normal_estimation_sq");
	ros::NodeHandle n;
	ros::Rate roop(10);
	ros::Subscriber sub = n.subscribe("/sq_lidar/points/lcl",1,lcl_callback);
	//	pub = n.advertise<sensor_msgs::PointCloud>("perfect_velodyne",1);
	//pub_2 = n.advertise<sensor_msgs::PointCloud2>("perfect_velodyne/color",1);
	pub_pc = n.advertise<sensor_msgs::PointCloud2>("/sq_lidar/points/lcl/normal",1);
	// pub_mk = n.advertise<visualization_msgs::Marker>("perfect_sq/normal_vector",1);
	//while(ros::ok()){	
	ros::spin();
	//	roop.sleep();
	//}
	return 0;
}
