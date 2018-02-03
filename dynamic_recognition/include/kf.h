#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl_ros/point_cloud.h>
#include <Eigen/Core>

struct Position{
	double x;
	double y;
	double yaw;
};

typedef struct{
	int count;
	int confidence; // when human is detected, transition +1, else -1 
    int label;		// Static or Dynamic
    int color_id;
	bool update_flag;
	bool update_comp_flag;
	float width;
	float height;
	float length;
	float likelihood;
	float track_num;
	float r;
	float yaw;
	double dx;
	double dy;
	geometry_msgs::Point32 pre_vel; // previous velocity 
	Eigen::Vector4f x;				// state for filter
	Eigen::Matrix4f P;				// uncertainity 
	Eigen::Vector4f u;				// (0,0,accel_x,accel_y) 
	Eigen::Matrix2f R;				// measurement uncertainty
	std::vector<pcl::PointXYZINormal> pre_position;//pre_position
	Position global_position;
	pcl::PointCloud<pcl::PointXYZINormal> comp;//cluster component
}clusterInfo;

struct KalmanFilter{
	double dt;
	Eigen::Matrix4f A;
	Eigen::MatrixXf Z;
	Eigen::MatrixXf y;
	Eigen::MatrixXf K;
	Eigen::Matrix2f S;
	Eigen::Matrix4f F; //next state function
	Eigen::MatrixXf H; //measurement function
	Eigen::Matrix2f R; //measurement uncertainty
};

typedef pcl::PointXYZINormal PointI;
typedef pcl::PointCloud<PointI> CloudI;
typedef pcl::PointCloud<PointI>::Ptr CloudIPtr;

void setParameter(KalmanFilter& kf, double dt);
void initCluster(clusterInfo& cluster, PointI& init_pt);
void Prediction(clusterInfo& cluster, KalmanFilter& kf);
void MeasurementUpdate(	PointI& obj_centroid, 
						clusterInfo& cluster,
						CloudI& cluster_pt,
						KalmanFilter& kf);
void visualizeArrowIn2D(ros::Publisher& pub, std::vector<clusterInfo>& clusters);
void visualizeArrowIn3D(ros::Publisher& pub, std::vector<clusterInfo>& clusters);
void visualizeErrorEllipse(ros::Publisher& ee_pub, std::vector<clusterInfo>& clusters);
void visualize_tracking_trajectory(ros::Publisher& trajectory_pub, std::vector<clusterInfo>& clusters);
