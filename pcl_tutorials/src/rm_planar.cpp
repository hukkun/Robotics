#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

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


ros::Publisher pub_pc;

using namespace std;

typedef pcl::PointXYZINormal PointA;
typedef pcl::PointCloud<PointA>  CloudA;
typedef pcl::PointCloud<PointA>::Ptr  CloudAPtr;

void lcl_callback(sensor_msgs::PointCloud2::Ptr msg)
{
	// pcl::fromROSMsg(msg, *cloud);
	ros::Time tm = msg->header.stamp;

	//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
	CloudAPtr cloud (new CloudA);
	CloudAPtr cloud_filtered (new CloudA);
	CloudAPtr cloud_f (new CloudA);
	CloudAPtr cloud_p (new CloudA);
	CloudAPtr cloud_plane (new CloudA);

	pcl::fromROSMsg(*msg, *cloud);

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZINormal> vg;
	vg.setInputCloud (cloud);
	vg.setLeafSize (0.01f, 0.01f, 0.01f);
	vg.filter (*cloud_filtered);

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZINormal> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (10000);
	seg.setDistanceThreshold (0.15);

	int i = 0, nr_points = (int) cloud_filtered->points.size ();
	clock_t start, end;

	start = clock();
	// While 30% of the original cloud is still there
	while (cloud_filtered->points.size () > 0.2 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the inliers
	    pcl::ExtractIndices<pcl::PointXYZINormal> extract;
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloud_plane);
		
		std::cerr << "PointCloud representing the planar component: " << cloud_plane->width * cloud_plane->height << " data points." << std::endl;


		// Create the filtering object
		extract.setNegative (true);
		extract.filter (*cloud_f);
		cloud_filtered.swap (cloud_f);
		i++;

	}
	end = clock();
	
	cout<<"end!!!!!!!!!!!!"<<endl;
	cout<<"time = "<<(double)(end - start)/CLOCKS_PER_SEC<<endl;


	//pub2msg
	sensor_msgs::PointCloud2 pc_;
	pcl::toROSMsg(*cloud_f, pc_);
	cloud_f->points.clear();
	pc_.header.frame_id = "/centerlaser_";
	pc_.header.stamp = tm;
	pub_pc.publish(pc_);

}

int main (int argc, char** argv)
{
	int a = 0;
	ros::init(argc, argv, "rm_planar");
	ros::NodeHandle n;
	ros::Rate roop(1);
	ros::Subscriber sub = n.subscribe("sq_lidar/obstacles",1,lcl_callback);
	//	pub = n.advertise<sensor_msgs::PointCloud>("perfect_velodyne",1);
	//pub_2 = n.advertise<sensor_msgs::PointCloud2>("perfect_velodyne/color",1);
	pub_pc = n.advertise<sensor_msgs::PointCloud2>("/sq_lidar/points/lcl/rm_planar",1);
	// pub_mk = n.advertise<visualization_msgs::Marker>("perfect_sq/normal_vector",1);
	//while(ros::ok()){	
	//if((a = getchar() == '\n')) cout<<"pressed ENTER!!!!!!!!!!!"<<endl;
	ros::spin();
	//	roop.sleep();
	//}
	return 0;
}
