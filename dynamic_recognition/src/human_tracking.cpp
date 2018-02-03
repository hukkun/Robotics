#include "kf.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ceres_msgs/AMU_data.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <visualization_msgs/MarkerArray.h>

//erase flag parameter
#define ALPHA 5
#define BETA 2
#define TH_LIKE -10//-5//1
#define TH_R 2
#define MAX_P 1.1 
/*
 * #define MIN_CONF -5
 */
#define MIN_CONF -8
#define MAX_RANGE 70
//association parameter
#define DIFF_W 0.4200 
#define DIFF_H 0.2000
#define DIFF_L 0.4200

#define SKIP 0

int  color_id = 0;
bool using_colors[10];

const double dt = 0.2;

using namespace std;
using namespace Eigen;

inline float calcDistance(clusterInfo& p1, PointI& p2)
{
	return sqrt((p1.x(0)-p2.x) * (p1.x(0)-p2.x) + (p1.x(1)-p2.y) * (p1.x(1)-p2.y));
}

inline float calcDistance(PointI cluster, PointI human_pos)
{
	return sqrt((cluster.x-human_pos.x) * (cluster.x-human_pos.x) + (cluster.y-human_pos.y) * (cluster.y-human_pos.y));
}

inline bool compareSize(clusterInfo& p1, PointI& p2)
{
	if (fabs(p2.normal_z-p1.height)<DIFF_H
			&& fabs(p2.normal_x-p1.width)<DIFF_W
			&& fabs(p2.normal_y-p1.length)<DIFF_L)
		return true;
	else return false;
}

inline void getClusterComponent(CloudI& obj_comp, CloudI& obj, PointI& obj_info)
{
	/*
	 * intensity == start index, curvature == component size
	 */
	int num_comp = obj_info.curvature;
	obj_comp.points.resize(num_comp);
	for (int j=0; j<num_comp; ++j)
		obj_comp.points[j] = obj.points[j + obj_info.intensity];
}

/*
 * 同一クラスタかどうかの比較をし，同一クラスタならば観測を更新する
 * getClusterComponentで何を行っているのか調べる
 */
void trackCluster(		std::vector<clusterInfo>& clusters,
		CloudI& obj,
		CloudI& obj_centroid,
		Position& robot,
		KalmanFilter& kf)
{
	int  associated_id = 0;
	int  num_clusters  = clusters.size();
	bool update_flag   = false;

	for (size_t i=0; i<obj_centroid.points.size(); i++){

		// float min_dist = 1.0; //距離がX[m]以内であれば，同一クラスタとして判断
		float min_dist = 2.0; //距離がX[m]以内であれば，同一クラスタとして判断

		for (int j=0; j<num_clusters; j++){

			if ( compareSize(clusters[j], obj_centroid.points[i]) ){ // width, length, height比較し，定めた値から外れた場合，距離による比較を行わない
				float dist = calcDistance(clusters[j], obj_centroid.points[i]); // clusters[i]と得られたクラスタの距離を比較           

				if ( dist < min_dist ){ // より距離の近いクラスタを同一クラスタとする
					update_flag   = true;
					associated_id = j; // 比較されたclusters[j]の中で最も近いクラスタid
					min_dist      = dist;
				}
			}
		}

		if (update_flag){ // 同一クラスタの判断があれば観測を更新

			CloudI obj_comp;
			getClusterComponent(obj_comp, obj, obj_centroid.points[i]);
			MeasurementUpdate(obj_centroid.points[i], clusters[associated_id], obj_comp, robot, kf); // 観測を更新
			update_flag = false;
		}
	}
}

inline void updateClassConfidence(	std::vector<clusterInfo>& clusters,
									std::vector<bool>& update_list)
{
	int num_clusters = clusters.size();

	for (int i=0; i<num_clusters; i++){

		if (update_list[i]) clusters[i].confidence++; // clusters[i]の信頼度を高める

		else {
			if (clusters[i].confidence>0) clusters[i].confidence=0; // clusters[i]の信頼度を0にする
			else clusters[i].confidence--;
		}
	}
}

/*
 * 人認識されたクラスタ -> 同一人物がいるかの判断 -> いない場合 clustersに追加
 */
void checkClusterClass(std::vector<clusterInfo>& clusters,
					   CloudI& human, 
					   CloudI& human_centroid,
					   Position& robot)
{
	int num_humans   = human_centroid.points.size();
	int num_clusters = clusters.size();
	vector<bool> update_list(num_clusters); // clustersの数だけupdata_listを作る

	for (int i=0; i<num_humans; ++i){ // 人認識された数回す
		int associated_id = 0;
		float min_dist = 1.0;

		bool update_flag = false;

		for (int j=0; j<num_clusters; ++j){ // clustersの数だけ比較

			if (update_list[j] == false){ // 同一人物の可能性が高いクラスタの抽出 -> updata_listに登録
				float dist = calcDistance(clusters[j].pre_position[0], human_centroid.points[i]);

				if (dist < min_dist && dist < TH_R){
					associated_id = j;
					min_dist = dist;
					update_flag = true;
				}
			}
		}

		if (update_flag){ // 上記の比較をパスするクラスタの決定
			update_list[associated_id] = true;
		}

		else { // 同一人物の可能性があるクラスタが見つからない場合 -> 初期の観測と考えclustersに格納
			int pre_size = clusters.size();
			int num		 = 1;
			int swap	 = 0;

			clusters.resize(pre_size + 1);
			initCluster(clusters[pre_size], human_centroid.points[i], robot); // clustersにクラスタを追加
			clusters[pre_size].color_id = color_id; // 色の決定
			swap = color_id;
			using_colors[color_id] = true;
			for(int j=0; j<10; j++) cout<<"| using colors["<<j<<"] = "<<using_colors[j]<<" |"<<endl;

			// while(using_colors[color_id]){
			// 	color_id = (swap + num) % 10;			// 10色
			// 	num ++;
			// 	if(num == 10) break;
			// }

			for(int j=0; j<10; j++){
				if(using_colors[j] == false){
					color_id = j;
					using_colors[j] = true;
					break;
				}
			}

			for(int j=0; j<10; j++) cout<<"| using colors["<<j<<"] = "<<using_colors[j]<<" |"<<endl;

			cout<<"i = " <<i<<" color_id --> "<<clusters[pre_size].color_id<<endl;
		}
	}	
	updateClassConfidence(clusters, update_list); // クラスタの信頼度の計算
}

void predictCluster(std::vector<clusterInfo>& clusters, KalmanFilter& kf)
{
	int  num_clusters = clusters.size();
	bool erase_flag   = false;

	for (int i=0; i<num_clusters; ++i){
		Prediction(clusters[i], kf);

		if (fabs(clusters[i].x(0)) > MAX_RANGE || fabs(clusters[i].x(1)) > MAX_RANGE){
			cout<<"out of range"<<endl;
			erase_flag = true;
		}

		else if (clusters[i].P(0,0) >= MAX_P){ 
			cout<<"too large R: "<<clusters[i].r<<endl;
			erase_flag =  true;
		}

		// else if (clusters[i].confidence<MIN_CONF){
		// 	cout<<"due to likelihood"<<endl;
		// 	erase_flag = true;
		// }

		if (erase_flag){
			cout<<"i = "<<i<<" eliminate "<<endl;	
			using_colors[clusters[i].color_id] = false;
			clusters.erase(clusters.begin()+i); //メモリの開放 開放された場所は詰められる
			for(int j=0; j<10; j++) cout<<"| using colors["<<j<<"] = "<<using_colors[j]<<" |"<<endl;

			// i++; // 何に使われているか謎
			erase_flag = false;
		}
	}
}

inline void pubPointCloud2(	ros::Publisher& pub,
		pcl::PointCloud<pcl::PointXYZINormal>& pcl_pt,
		const char* frame_id)
{
	sensor_msgs::PointCloud2 ros_pt;
	toROSMsg(pcl_pt, ros_pt);
	ros_pt.header.frame_id=frame_id;
	ros_pt.header.stamp=ros::Time::now();
	pub.publish(ros_pt);
}

inline void deletePointsInRviz(CloudI& tmp)
{
	tmp.points.resize(1);
	tmp.points[0].x         = -1000;
	tmp.points[0].y         = -1000;
	tmp.points[0].z         = -1000;
	tmp.points[0].intensity = 1;
}

void pubTrackingResults(std::vector<clusterInfo>& clusters, ros::Publisher& pos_pub, ros::Publisher& pt_pub)
{
	CloudI track_pt, h_comp_pcl;
	if (clusters.size()){
		int pre_num=0;
		int num_clusters = clusters.size();
		track_pt.points.resize(num_clusters);
		for (int i=0; i<num_clusters; ++i){
			track_pt.points[i].x         = clusters[i].x(0);
			track_pt.points[i].y         = clusters[i].x(1);
			track_pt.points[i].z         = -1.3;
			track_pt.points[i].intensity = clusters[i].track_num;
			track_pt.points[i].curvature = clusters[i].width;

			if (clusters[i].update_flag){
				int num_comp = clusters[i].comp.points.size();
				h_comp_pcl.points.resize(pre_num + num_comp);

				for (int j=0; j<num_comp; ++j){
					h_comp_pcl.points[j+pre_num]=clusters[i].comp.points[j];
				}

				pre_num += num_comp;

				clusters[i].update_flag=false;
			}
		}
	}
	else {
		deletePointsInRviz(track_pt);
		deletePointsInRviz(h_comp_pcl);
	}

	pubPointCloud2(pos_pub, track_pt, "velodyne");
	pubPointCloud2(pt_pub, h_comp_pcl, "velodyne");
}

CloudI _human_centroid; 
bool callback_flagH=false;

void HumanPositionCallback(const sensor_msgs::PointCloud2::Ptr &msg)
{
	// boost::mutex::scoped_lock(mutex1);
	pcl::fromROSMsg(*msg, _human_centroid);
	callback_flagH=true;
}

CloudI _human; 

void HumanPointsCallback(const sensor_msgs::PointCloud2::Ptr &msg)
{
	pcl::fromROSMsg(*msg, _human);
}

CloudI _obj_centroid;
bool callback_flagC=false;

void ObjectCentroidCallback(const sensor_msgs::PointCloud2::Ptr &msg)
{
	pcl::fromROSMsg(*msg, _obj_centroid);
	callback_flagC=true;
}

CloudI _obj;
bool callback_flagC2=false;

void ObjectPointCallback(const sensor_msgs::PointCloud2::Ptr &msg)
{
	pcl::fromROSMsg(*msg, _obj);
	callback_flagC2=true;
}

double _robot_vel;
double _robot_angular;
bool callback_amu=false;
bool callback_odom=false;

void OdometryCallback(const nav_msgs::Odometry &msg)
{
	_robot_vel  = msg.twist.twist.linear.x;
	callback_odom=true;
}

void AMUCallback(ceres_msgs::AMU_data::ConstPtr msg)
{
	_robot_angular = msg->dyaw;
	callback_amu=true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "human_tracking");
	ros::NodeHandle nh;

	ros::Subscriber human_pos_sub  = nh.subscribe("/human_recognition/positive_position",1,HumanPositionCallback);
	ros::Subscriber human_pt_sub   = nh.subscribe("/human_recognition/positive_pt", 1, HumanPointsCallback);
	ros::Subscriber object_pos_sub = nh.subscribe("/cluster/centroid",1,ObjectCentroidCallback);
	ros::Subscriber object_pt_sub  = nh.subscribe("/cluster/points",1,ObjectPointCallback);
	ros::Subscriber odom_sub       = nh.subscribe("/tinypower/odom",1, OdometryCallback);
	ros::Subscriber amu_sub        = nh.subscribe("/AMU_data",1, AMUCallback);

	ros::Publisher ee_pub         = nh.advertise<sensor_msgs::PointCloud>("/tracking/error_ellipse", 1);
	ros::Publisher track_pos_pub  = nh.advertise<sensor_msgs::PointCloud2>("/tracking/position", 1);
	ros::Publisher track_pt_pub   = nh.advertise<sensor_msgs::PointCloud2>("/tracking/points",1);
	ros::Publisher track_vel_pub  = nh.advertise<visualization_msgs::MarkerArray>("/tracking/velocity", 1);
	ros::Publisher trajectory_pub = nh.advertise<visualization_msgs::MarkerArray>("/tracking/trajectory", 1);

	KalmanFilter kf;
	setParameter(kf, dt); // whileループに組み込んだほうが良さそう?(dtの値の関係から)

	vector<clusterInfo> clusters(0);

	ros::Time t1, t2;
	t1 = ros::Time::now();

	_robot_angular = 0;
	Position robot={0,0,0};
	bool start_flag=false;

	cout<<"start"<<endl;
	ros::Rate loop_rate(1.0/dt);

	for(int i=0; i<10; i++){
		using_colors[i] = false;
	}

	while (ros::ok()){

		// センサ移動量の考慮
		if (callback_odom || callback_amu){
			t2 = ros::Time::now();
			double time_2 = (double)t2.nsec*1.0e-9 + t2.sec;
			double time_1 = (double)t1.nsec*1.0e-9 + t1.sec;
			double dt = (time_2 - time_1);

			int num = clusters.size();

			// double dyaw = _robot_angular * dt * M_PI / 180;
			double dyaw = _robot_angular * dt * M_PI / 180;

			for (int i=0; i<num; i++){
				double r       = sqrt(clusters[i].x(0)*clusters[i].x(0) + clusters[i].x(1)*clusters[i].x(1)); // clusters[i]のx,yから距離rを算出
				double pre_yaw = atan2(clusters[i].x(1), clusters[i].x(0));									  // clusters[i]のx,yから角度thetaを算出
				// double pre_yaw = atan2(clusters[i].x(0), clusters[i].x(1));									  // clusters[i]のx,yから角度thetaを算出

				// clusters[i].x(0)  = r * cos (pre_yaw + dyaw); // AMUによる角度の補正
				// clusters[i].x(1) -= _robot_vel;				  // wheel odometoryによる直進方向の補正
				// clusters[i].x(1)  = r * sin (pre_yaw + dyaw); // AMUによる角度の補正
				// clusters[i].x(0) -= _robot_vel;				  // wheel odometoryによる直進方向の補正
				clusters[i].x(0) += _robot_vel;				  // wheel odometoryによる直進方向の補正
				clusters[i].x(0)  = r * sin (pre_yaw + dyaw); // AMUによる角度の補正
				clusters[i].x(1)  = r * cos (pre_yaw + dyaw); // AMUによる角度の補正
			}

			t1 = t2;

			_robot_vel     = 0;
			_robot_angular = 0;
			callback_odom  = false;
			callback_amu   = false;
			start_flag     = true;
		}

		if (!start_flag) t1 = ros::Time::now();

		// 観測の更新
		if (callback_flagC && callback_flagC2){ 
			CloudI obj, obj_centroid;
			{
				obj = _obj;	
				obj_centroid = _obj_centroid;	
			}
			trackCluster(clusters, obj, obj_centroid, robot, kf);
			// std::cout<<"~~~~~~~~~~~~~~~~~~~~~~~"<<std::endl;
			callback_flagC  = false;
			callback_flagC2 = false;
			CloudI sw1, sw2;
			sw1.swap(_obj);
			sw2.swap(_obj_centroid);
		}

		// 追跡するクラスタの決定
		if (callback_flagH){ 
			CloudI human, human_centroid;
			{
				human = _human;	
				human_centroid = _human_centroid;	
			}
			checkClusterClass(clusters, human, human_centroid, robot);
			callback_flagH=false;
			CloudI sw1, sw2;
			sw1.swap(_human);
			sw2.swap(_human_centroid);
		}

		// 予測の更新 
		if (clusters.size()){ 
			predictCluster(clusters, kf);
		}

		pubTrackingResults(clusters, track_pos_pub, track_pt_pub);
		visualizeArrowIn3D(track_vel_pub, clusters);
		// visualizeErrorEllipse(ee_pub, clusters);
		// visualize_tracking_trajectory(trajectory_pub, clusters);

		ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout<<"end"<<std::endl;	
	return 0;
}
