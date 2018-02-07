#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

using namespace std;

void pc_callback(sensor_msgs::PointCloud2::ptr cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);

  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
  
}

int main(int argc, char** argv)
{
	cout << "Normal Estimation Start" << endl;
	ros::init(argc, argv, "normal_estimation");
	ros::NodeHandle n;
	ros::Rate roop(2)
    ros::Subscriber sub = n.subscrive("/sq_lidar/points/lcl",1);
	ros::Publiser pub = n.advertise<sensor_msgs::PointCloud2>("/sq_lidar/point/lcl/after",1);

	ros::spin();

	return 0;

}
