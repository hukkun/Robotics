#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/ModelCoefficients.h>  
#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h>  
#include <pcl/features/normal_3d.h>  
#include <pcl/filters/extract_indices.h>  
#include <pcl/filters/voxel_grid.h>  
#include <pcl/kdtree/kdtree.h>  
#include <pcl/sample_consensus/method_types.h>  
#include <pcl/sample_consensus/model_types.h>  
#include <pcl/segmentation/sac_segmentation.h>  
#include <pcl/segmentation/extract_clusters.h>  
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

void savePCDfile(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_data, int j){
	char name[100];
	// sprintf(name, "/home/amsl/vel_pcd/cluster/%d/cluster.pcd", j);
	sprintf(name, "/home/amsl/sq_pcd/cluster/%d/cluster.pcd", j);
	pcl::io::savePCDFileBinary(name, *cloud_data);
	cout << "PCD file saved." << endl;
}

void coloring(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_cluster, vector<pcl::PointIndices> cluster_indices, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_save){
	int j = 0;
	float colors[10][3] ={{255, 0, 0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}, {255,255,255}, {128,0,128}, {128,128,128}, {128,128,0}};  
	cloud_save->points.resize(cloud_cluster->points.size());
	for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)  
    {
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
			// cloud_cluster->points[*pit].r =	cloud_save->points[*pit].r = colors[j%10][0];	cloud_save->points[*pit].x = cloud_cluster->points[*pit].x;
			// cloud_cluster->points[*pit].g =	cloud_save->points[*pit].g = colors[j%10][1];	cloud_save->points[*pit].y = cloud_cluster->points[*pit].y;
			// cloud_cluster->points[*pit].b =	cloud_save->points[*pit].b = colors[j%10][2];	cloud_save->points[*pit].z = cloud_cluster->points[*pit].z;
			cloud_cluster->points[*pit].r =	cloud_save->points[*pit].r = 255*0.333*(j%4);		cloud_save->points[*pit].x = cloud_cluster->points[*pit].x;
			cloud_cluster->points[*pit].b =	cloud_save->points[*pit].b = 255*0.1*(j%11);	cloud_save->points[*pit].y = cloud_cluster->points[*pit].y;
			if(j<10){ cloud_cluster->points[*pit].g =	cloud_save->points[*pit].g = 255*0.5;	cloud_save->points[*pit].z = cloud_cluster->points[*pit].z;}
			else { cloud_cluster->points[*pit].g =	cloud_save->points[*pit].b = 255;		cloud_save->points[*pit].z = cloud_cluster->points[*pit].z;}
			cloud_save->points[*pit].normal_x = cloud_cluster->points[*pit].normal_x;
			cloud_save->points[*pit].normal_y = cloud_cluster->points[*pit].normal_y;
			cloud_save->points[*pit].normal_z = cloud_cluster->points[*pit].normal_z;
		}
//		cout << j << ": PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << endl;
//		savePCDfile(cloud_save, j);
		j++;
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "save_clusterWithNormal");
	ros::NodeHandle n;
	
	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/cluster", 2);

	struct timeval start_time, finish_time;

	char pcd_name[20] = {};
	printf("Input PCD file name:");
	scanf("%s", pcd_name);
	char file_name1[100];
	sprintf(file_name1, "/home/amsl/vel_pcd/%s", pcd_name);
	// sprintf(file_name1, "/home/amsl/sq_pcd/%s", pcd_name);
	
	// Measure start time
	gettimeofday(&start_time, NULL);

	// Read in the cloud data  
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::io::loadPCDFile(file_name1, *cloud);
	cout << "PointCloud Input data has: " << cloud->points.size () << " data points." << endl;  
	
    // Creating the KdTree object for the search method of the extraction  
	pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZINormal>);  
	tree->setInputCloud(cloud);  
	
	vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZINormal> ec; 
	//ec.setClusterTolerance (0.3); // [m]
	// ec.setClusterTolerance (0.5); // [m]
	ec.setClusterTolerance (0.20); // [m]
	ec.setMinClusterSize (30);
	ec.setMaxClusterSize (2500000);
	ec.setSearchMethod (tree);
	ec.setInputCloud( cloud);
	ec.extract (cluster_indices);
	cout << "Cluster number : " << cluster_indices.size() << endl;

	// Coloring by Cluster and save pcd file
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_save (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	sensor_msgs::PointCloud2 pc_pub;
	pcl::copyPointCloud(*cloud, *cloud_cluster);
	coloring(cloud_cluster, cluster_indices, cloud_save);
	
	pcl::toROSMsg(*cloud_save, pc_pub);
	//pc_pub.header.frame_id = "/velodyne";
	// pc_pub.header.frame_id = "/centerlaser_";
	pc_pub.header.frame_id = "/map";
	
	// Measure finish time
	gettimeofday(&finish_time, NULL);
	// Calculate processing duration
	float dt = (finish_time.tv_sec - start_time.tv_sec) + (finish_time.tv_usec - start_time.tv_usec)*1e-6;
	cout << "process duration is " << dt << "[sec]" << endl;
	
	ros::Rate loop_rate(1);
	while(ros::ok()){
		pc_pub.header.stamp = ros::Time::now();
		pub.publish(pc_pub);
		cout << "published!" <<endl;

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;  
}
