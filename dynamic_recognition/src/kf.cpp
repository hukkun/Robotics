#include "kf.h"
#include <Eigen/Dense>
#include <pcl/point_types.h>

// erase flag parame
#define ALPHA 5
#define BETA  2

// association param
#define SKIP     4
#define PRE_SIZE 4

// dynamic detector param
#define MIN_VELOCITY 0.3
#define MAX_VELOCITY 2

using namespace std;

void setParameter(KalmanFilter& kf, double dt)
{
	float ini_r = 0.01;

	kf.F << 1.0, 0.0,  dt, 0.0,
			0.0, 1.0, 0.0,  dt,
			0.0, 0.0, 1.0, 0.0,
			0.0, 0.0, 0.0, 1.0;

	kf.H.resize(2,4);
	kf.H << 1.0, 0.0, 0.0, 0.0,
			0.0, 1.0, 0.0, 0.0;
	
	kf.R << ini_r, 0.0,
			 0.0, ini_r;
	
	// kf.A<< 0, 0, dt*dt*0.5,         0,
	// 	   0, 0,         0, dt*dt*0.5,
	// 	   0, 0,        dt,         0,
	// 	   0, 0,         0,         dt;
	
	kf.Z.resize(1,2);
	kf.K.resize(2,4);
	kf.dt=dt;
}

void initCluster(clusterInfo& cluster, PointI& init_pt)
{
	float ini_r = 0.01; // 観測誤差
	float ini_p = 1.0;  //初期位置の分散値
	cluster.x << init_pt.x, init_pt.y, 0.0, 0.0;
	cluster.P << 0.0, 0.0,   0.0,  0.0,
		 		 0.0, 0.0,   0.0,  0.0,
		 		 0.0, 0.0, ini_p,  0.0,
		 		 0.0, 0.0,   0.0, ini_p;
	cluster.R << ini_r,   0.0,
				   0.0, ini_r;
	cluster.pre_vel.x       = 0;
	cluster.pre_vel.y       = 0;
	cluster.u               = Eigen::VectorXf::Zero(4);
	cluster.track_num       = 1;
	cluster.confidence      = 1;
	cluster.width           = init_pt.normal_x;
	cluster.length          = init_pt.normal_y;
	cluster.height          = init_pt.normal_z;
	cluster.count           = 0;
	cluster.pre_position.resize(PRE_SIZE);
	cluster.pre_position[0] = init_pt;
	cluster.label = 0; // static: 0  dynamic: 1
	cluster.update_comp_flag = true;
}

void Prediction(clusterInfo& cluster, KalmanFilter& kf)
{
	cluster.x = kf.F * cluster.x;

	cluster.P     = kf.F * cluster.P * kf.F.transpose();
	cluster.count = 0;

	// cluster.likelihood = ALPHA*cluster.confidence + BETA*cluster.track_num;

	double cluster_vel = sqrt(cluster.x(2)*cluster.x(2) + cluster.x(3)*cluster.x(3));

	// if(cluster_vel > MIN_VELOCITY && cluster_vel < MAX_VELOCITY) cluster.label = 1; // dynamic
	cluster.label = 1;
}

void MeasurementUpdate(	PointI& obj_centroid, 
						clusterInfo& cluster,
						CloudI& cluster_pt,
						KalmanFilter& kf)
{
	Eigen::MatrixXf measurements(1,2);
	measurements << obj_centroid.x, obj_centroid.y;
	
	kf.Z = measurements.row(0); 
	kf.y = kf.Z.transpose() - (kf.H * cluster.x);
	kf.S = kf.H * cluster.P * kf.H.transpose() + kf.R;
	kf.K = cluster.P * kf.H.transpose() * kf.S.inverse();
	
	cluster.x = cluster.x + (kf.K * kf.y);
	
	cluster.P = (Eigen::MatrixXf::Identity(4,4) - (kf.K * kf.H)) * cluster.P;

	// double min = 2.0;
	// if (cluster.P(2,2)<min) cluster.P(2,2)=min;
	// if (cluster.P(3,3)<min) cluster.P(3,3)=min;
	
	cluster.u(2)            = (cluster.x(2)-cluster.pre_vel.x)/kf.dt;
	cluster.u(3)            = (cluster.x(3)-cluster.pre_vel.y)/kf.dt;
	cluster.pre_vel.x       = cluster.x(2);
	cluster.pre_vel.y       = cluster.x(3);
	cluster.width           = obj_centroid.normal_x;
	cluster.length          = obj_centroid.normal_y;
	cluster.height          = obj_centroid.normal_z;
	cluster.track_num++;
	
	// 今まで座標をPRE_SIZEの数だけ格納
	for (int i=0; i<(PRE_SIZE-1); ++i) cluster.pre_position[i+1] = cluster.pre_position[i];
	// cluster.pre_position[0] = obj_centroid;
	cluster.pre_position[0].x = cluster.x(0);
	cluster.pre_position[0].y = cluster.x(1);
	cluster.pre_position[0].z = -1.3;
}

