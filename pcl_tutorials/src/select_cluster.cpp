#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Core>
#include <boost/thread.hpp>

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZINormal PointA;
typedef pcl::PointCloud<PointA>  CloudA;
typedef pcl::PointCloud<PointA>::Ptr  CloudAPtr;

float cluster_size[100];
// vector<float> cluster_size;

float MIN_SIZE = 3.3;
float MAX_SIZE = 3.4;
// double MIN_SIZE = 0.1;
// double MAX_SIZE = 1.4;
// double MIN_SIZE;
// double MAX_SIZE;
// double MIN_SIZE = 3.1;
// double MAX_SIZE = 3.2;

// template <class T>
// void getParam(ros::NodeHandle &n, string param, T &val){
//     string str;
//     if(!n.hasParam(param)){
//         cout << param << " don't exist." << endl;
//     }   
//
//     if(!n.getParam(param, str)){
//         cout << "NG" << endl;
//     }   
//     std::stringstream ss(str);
//     T rsl;
//     ss >> rsl;
//     val = rsl;
//     cout << param << " = " << str << endl;
// }
//
//
// bool getParams(ros::NodeHandle& n)
// {
// 	getParam(n, "MIN_SIZE", MIN_SIZE);
// 	getParam(n, "MAX_SIZE", MAX_SIZE);
// 	// n.param("MIN_SIZE", MIN_SIZE, 3.1);
// 	// n.param("MAX_SIZE", MAX_SIZE, 3.4);
// 	// n.getParam("MIN_SIZE", MIN_SIZE);
// 	// n.getParam("MAX_SIZE", MAX_SIZE);
// 	return true;
// }

void pubPC2Msg(ros::Publisher& pub, CloudA& p_in, std::string& frame_id, ros::Time& time)
{
	sensor_msgs::PointCloud2 p_out;
	toROSMsg(p_in, p_out);
	p_out.header.frame_id = frame_id;
	p_out.header.stamp    = time;
	pub.publish(p_out);
}

double getClusterInfo(CloudA pt, PointA &cluster)
{
	Vector3f centroid;
	centroid[0] = pt.points[0].x;
	centroid[1] = pt.points[0].y;
	centroid[2] = pt.points[0].z;

	Vector2f min_p;
	min_p[0] = pt.points[0].x;
	min_p[1] = pt.points[0].y;

	Vector3f max_p;
	max_p[0] = pt.points[0].x;
	max_p[1] = pt.points[0].y;
	max_p[2] = pt.points[0].z;

	for(size_t i=1;i<pt.points.size();i++){
		centroid[0] += pt.points[i].x;
		centroid[1] += pt.points[i].y;
		centroid[2] += pt.points[i].z;
		if (pt.points[i].x<min_p[0]) min_p[0] = pt.points[i].x;
		if (pt.points[i].y<min_p[1]) min_p[1] = pt.points[i].y;

		if (pt.points[i].x>max_p[0]) max_p[0] = pt.points[i].x;
		if (pt.points[i].y>max_p[1]) max_p[1] = pt.points[i].y;
		if (pt.points[i].z>max_p[2]) max_p[2] = pt.points[i].z;
	}

	cluster.x        = centroid[0]/(float)pt.points.size();
	cluster.y        = centroid[1]/(float)pt.points.size();
	cluster.z        = centroid[2]/(float)pt.points.size();
	cluster.normal_x = max_p[0]-min_p[0];
	cluster.normal_y = max_p[1]-min_p[1];
	cluster.normal_z = max_p[2];
	return cluster.normal_x * cluster.normal_y * cluster.normal_z;
}

void cpu_clustering(CloudAPtr pcl_in, CloudA& cluster_centroid, CloudA& cluster_pt, CloudA& dynamic_cluster, CloudA& static_cluster)
{
	// downsampled point's
	pcl::VoxelGrid<pcl::PointXYZINormal> vg;  
	CloudAPtr ds_cloud(new CloudA);  
	vg.setInputCloud (pcl_in);  
	vg.setLeafSize (0.07f, 0.07f, 0.07f);
	vg.filter (*ds_cloud);


	// downsampled point's z -> 0
	vector<float> tmp_z;
	tmp_z.resize(ds_cloud->points.size());
	for(int i = 0; i < (int)ds_cloud -> points.size(); i++){
		tmp_z[i] = ds_cloud -> points[i].z;
		ds_cloud->points[i].z  = 0.0;
	}

	// clustering
	// creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<PointA>::Ptr tree (new pcl::search::KdTree<PointA>);
	tree -> setInputCloud(ds_cloud);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointA> ec;
	ec.setClusterTolerance (0.07); // 7cm 
	ec.setMinClusterSize (20);
	ec.setMaxClusterSize (16000);
	ec.setSearchMethod (tree);
	ec.setInputCloud(ds_cloud);
	ec.extract(cluster_indices);

	// reset z value
	for(int i = 0; i < (int)ds_cloud -> points.size(); i++){
		ds_cloud -> points[i].z = tmp_z[i];
	}

	// get cluster information
	size_t num = 0;
	cluster_centroid.resize(cluster_indices.size());

	for(size_t iii = 0; iii < cluster_indices.size(); iii++){

		//cluster cenrtroid
		CloudAPtr cloud_cluster(new CloudA);
		cloud_cluster -> points.resize(cluster_indices[iii].indices.size());


		//cluster points
		cluster_pt.points.resize(cluster_indices[iii].indices.size()+num);
		dynamic_cluster.points.resize(cluster_indices[iii].indices.size()+num);
		static_cluster.points.resize(cluster_indices[iii].indices.size()+num);

		for(size_t jjj = 0; jjj < cluster_indices[iii].indices.size(); jjj++){
			int p_num = cluster_indices[iii].indices[jjj]; // クラスタ毎の点番号
			cloud_cluster -> points[jjj] = ds_cloud -> points[p_num];
			cluster_pt.points[num+jjj] = ds_cloud -> points[p_num];
		}

		//get bounding box's centroid
		cluster_size[iii] = getClusterInfo(*cloud_cluster, cluster_centroid[iii]);

		for(size_t jjj = 0; jjj < cluster_indices[iii].indices.size(); jjj++){
			int p_num = cluster_indices[iii].indices[jjj];
			if(MIN_SIZE < cluster_size[iii] && cluster_size[iii] < MAX_SIZE) { // bounding boxのサイズから動的クラスタの選択
				dynamic_cluster.points[num+jjj] = ds_cloud -> points[p_num];
			}

			else {
				static_cluster.points[num+jjj] = ds_cloud -> points[p_num];
				// cout<<"else "<<num+jjj<<endl;
			}
		}

		// the number of points which constitute cluster[iii]
		cluster_centroid[iii].curvature=cloud_cluster->points.size();
		cluster_centroid[iii].intensity = num;
		num = cluster_pt.points.size();//save previous size
	}

}

boost::mutex pt_mutex;
bool Callback_flag = false;
CloudAPtr rmg_pt (new CloudA);

void objectCallback(const sensor_msgs::PointCloud2::Ptr &msg)
{
	boost::mutex::scoped_lock(pt_mutex);
	pcl::fromROSMsg(*msg,*rmg_pt);
	Callback_flag=true;
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "clustering");
	ros::NodeHandle n;

	ros::Publisher cluster_centroid_pub = n.advertise<sensor_msgs::PointCloud2>("/cluster/centroid",1); // クラスタの重心
	ros::Publisher cluster_points_pub   = n.advertise<sensor_msgs::PointCloud2>("/cluster/points",1); // クラスタリングされた点群
	ros::Publisher dynamic_cluster_pub   = n.advertise<sensor_msgs::PointCloud2>("/dynamic_cluster/points",1); // 動的物体と思われるクラスタ
	ros::Publisher static_cluster_pub   = n.advertise<sensor_msgs::PointCloud2>("/static_cluster/points",1); // 静的物体と思われるクラスタ
	// ros::Publisher flag_pub = nh.advertise<std_msgs::Bool>("/object_recognition/flag",1);
	ros::Subscriber object_sub = n.subscribe("sq_lidar/obstacles", 1, objectCallback);

	ros::Rate loop_rate(20);

	// getParams(n);
	// cout<<"MIN_SIZE = "<<MIN_SIZE<<endl;
	// cout<<"MAX_SIZE = "<<MAX_SIZE<<endl;

	while (ros::ok()){
		if (Callback_flag){
			// clock_t start=clock();
			CloudA cluster_centroid, cluster_points, dynamic_cluster, static_cluster;
			cpu_clustering(rmg_pt, cluster_centroid, cluster_points, dynamic_cluster, static_cluster);
			// float cluster_size = cluster_centroid.normal_x * cluster_centroid.normal_y * cluster_centroid.normal_z;

			std::string frame_id = "centerlaser_";
			ros::Time time= ros::Time::now();
			pubPC2Msg(cluster_centroid_pub, cluster_centroid, frame_id, time);
			pubPC2Msg(cluster_points_pub, cluster_points, frame_id, time);
			pubPC2Msg(dynamic_cluster_pub, dynamic_cluster, frame_id, time);
			pubPC2Msg(static_cluster_pub, static_cluster, frame_id, time);
			// cout<<(double)(clock()-start)/CLOCKS_PER_SEC<<endl<<endl;
			Callback_flag = false;        
		} 
		ros::spinOnce();
		loop_rate.sleep();
	}
	return (0);
}
