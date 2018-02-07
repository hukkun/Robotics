#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/pcl_base.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
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
		// pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);
		// pcl::PointCloud<pcl::PointXYZI>::Ptr leaf_cloud (new pcl::PointCloud<pcl::PointXYZI>);
		pcl::fromROSMsg(*msg, *cloud);
		// cout<<"sub_lcl"<<endl;
		// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());

		// pcl::io::loadPCDFile ("d_kan_sq1_indoor_success.pcd", *cloud);

		// pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
		// sor.setInputCloud (cloud);
		// // sor.setMeanK (50);
		// sor.setMeanK (10);
		// sor.setStddevMulThresh (0.3);
		// sor.filter (*filtered_cloud);
		//
		// // sor.setStddevMulThresh (1.2);
		// pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
		// voxel_filter.setInputCloud(filtered_cloud);
		// voxel_filter.setLeafSize(0.05, 0.05, 0.05);
		// voxel_filter.filter(*leaf_cloud);
		//
		// ne.setInputCloud (cloud);
		// ne.setInputCloud (filtered_cloud);

		pcl::PointCloud<pcl::PointXYZI>::Ptr output (new pcl::PointCloud<pcl::PointXYZI>);
		size_t size = cloud->points.size();
		// size_t size = filtered_cloud->points.size();
		// size_t size = filtered_cloud->points.size();
		pcl::PointXYZI tmp;

		for(size_t i=0; i<size; i++){
				// if(cloud->points[i].intensity > 500){ 
				if(cloud->points[i].intensity > 450){ 
						tmp.x = cloud->points[i].x;
						tmp.y = cloud->points[i].y;
						tmp.z = cloud->points[i].z;
						tmp.intensity = cloud->points[i].intensity;
						// tmp.x = leaf_cloud->points[i].x;
						// tmp.y = leaf_cloud->points[i].y;
						// tmp.z = leaf_cloud->points[i].z;
						// tmp.intensity = leaf_cloud->points[i].intensity;
						// tmp.x = filtered_cloud->points[i].x;
						// tmp.y = filtered_cloud->points[i].y;
						// tmp.z = filtered_cloud->points[i].z;
						// tmp.intensity = filtered_cloud->points[i].intensity;
						// output.points.push_back(tmp);
						output->points.push_back(tmp);
				}
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
		ros::init(argc, argv, "sq_lidar_filter");
		ros::NodeHandle n;
		ros::Rate roop(50);
		ros::Subscriber sub = n.subscribe("/sq_lidar/points/lcl",1,lcl_callback);
		// ros::Subscriber sub = n.subscribe("/point_union",1,lcl_callback);
		//	pub = n.advertise<sensor_msgs::PointCloud>("perfect_velodyne",1);
		//pub_2 = n.advertise<sensor_msgs::PointCloud2>("perfect_velodyne/color",1);
		pub_pc = n.advertise<sensor_msgs::PointCloud2>("/sq_lidar/points/lcl_filtered",1);
		// pub_mk = n.advertise<visualization_msgs::Marker>("perfect_sq/normal_vector",1);
		//while(ros::ok()){	
		ros::spin();
		//	roop.sleep();
		//}
		return 0;
}
