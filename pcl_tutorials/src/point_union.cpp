/*
 *2つの点群を組み合わせる
 *intensity あり
 *normal    なし
 */

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

bool cb_flag = false;

CloudAPtr rmg_pt1 (new CloudA);
CloudAPtr rmg_pt2 (new CloudA);
CloudAPtr rmg_pt3 (new CloudA);

void pubPC2Msg(ros::Publisher& pub, CloudA& p_in, std::string& frame_id, ros::Time& time)
{
	sensor_msgs::PointCloud2 p_out;
	toROSMsg(p_in, p_out);
	p_out.header.frame_id = frame_id;
	p_out.header.stamp    = time;
	pub.publish(p_out);
}

void point_union(CloudAPtr pcl_in1, CloudAPtr pcl_in2, CloudAPtr pcl_in3, CloudA& pcl_out)
{

	PointA temp;

	for(size_t i=0;i<pcl_in1->points.size();i++){
		temp.x = pcl_in1->points[i].x;
		temp.y = pcl_in1->points[i].y;
		temp.z = pcl_in1->points[i].z;
		temp.intensity = pcl_in1->points[i].intensity;
		pcl_out.points.push_back(temp);
	}

	for(size_t i=0;i<pcl_in2->points.size();i++){
		temp.x = pcl_in2->points[i].x;
		temp.y = pcl_in2->points[i].y;
		temp.z = pcl_in2->points[i].z;
		temp.intensity = pcl_in2->points[i].intensity;
		pcl_out.points.push_back(temp);
	}

	for(size_t i=0;i<pcl_in3->points.size();i++){
		temp.x = pcl_in3->points[i].x;
		temp.y = pcl_in3->points[i].y;
		temp.z = pcl_in3->points[i].z;
		temp.intensity = pcl_in3->points[i].intensity;
		pcl_out.points.push_back(temp);
	}

}

void cloud1Callback(const sensor_msgs::PointCloud2::Ptr &msg)
{
	pcl::fromROSMsg(*msg,*rmg_pt1);
	cb_flag = true;
}

void cloud2Callback(const sensor_msgs::PointCloud2::Ptr &msg)
{
	pcl::fromROSMsg(*msg,*rmg_pt2);
}

void cloud3Callback(const sensor_msgs::PointCloud2::Ptr &msg)
{
	pcl::fromROSMsg(*msg,*rmg_pt3);
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "point_union");
	ros::NodeHandle nh,n;

	ros::Publisher point_pub = n.advertise<sensor_msgs::PointCloud2>("/point_union",1);
	ros::Subscriber point1_sub = n.subscribe("sq_lidar/clear", 1, cloud1Callback);
	ros::Subscriber point2_sub = n.subscribe("static_cluster/points", 1, cloud2Callback);
	ros::Subscriber point3_sub = n.subscribe("static/points", 1, cloud2Callback);

	ros::Rate loop_rate(20);

	while (ros::ok()){
		if (cb_flag){
			// clock_t start=clock();
			CloudA pcl_out;
			point_union(rmg_pt1, rmg_pt2, rmg_pt3, pcl_out);


			std::string frame_id = "centerlaser_";
			ros::Time time= ros::Time::now();
			pubPC2Msg(point_pub, pcl_out, frame_id, time);
			// cout<<(double)(clock()-start)/CLOCKS_PER_SEC<<endl<<endl;
			cb_flag = false;        
		} 
		ros::spinOnce();
		loop_rate.sleep();
	}

	return (0);
}
