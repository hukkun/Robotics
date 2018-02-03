#include "ekf.h"
#include <Eigen/Dense>
#include <pcl/point_types.h>
// #include "mmath/binarion.h"

// association param
#define PRE_SIZE 5

// dynamic detector param
// #define MIN_VELOCITY 0.3
// #define MIN_VELOCITY 0.28
#define MIN_VELOCITY 0.20
// #define MAX_VELOCITY 2
#define MAX_VELOCITY 0.4

using namespace std;
using namespace Eigen;

MatrixXf EKF::move(MatrixXf x, MatrixXf u, float dt)
{
	/* 動作予測
	 * x : 状態
	 * u : 制御
	 * dt: 前ステップとの時間差
	 */
	MatrixXf A(3,1);
	MatrixXf I = MatrixXf::Identity(3,3);
	MatrixXf B(3,2);

	float theta = x.coeffRef(2,0) + u.coeffRef(1,0)*dt/2;
	
	B << dt*cos(theta),  0,
		 dt*sin(theta),  0,
		             0, dt;
	
	A = I*x + B*u;
	
	return A;
}

MatrixXf EKF::jacobF(MatrixXf x, MatrixXf u, float dt)
{
	/* ヤコビ行列
	 * x : 状態
	 * u : 制御
	 * dt: 前ステップとの時間差
	 */
	MatrixXf F(3,3);
	float b     = dt*u.coeffRef(0,0);
	float theta = x.coeffRef(2,0); 
	
	F << 1, 0, -b*sin(theta),
		 0, 1,  b*cos(theta),
		 0, 0,             1;

	return F;
}

MatrixXf EKF::jacobG(MatrixXf x, MatrixXf u, float dt)
{
	/* ヤコビ行列
	 * x : 状態
	 * u : 制御
	 * dt: 前ステップとの時間差
	 */
	MatrixXf G(3,3);
	float b     = u.coeffRef(0,0)*dt;
	float theta = u.coeffRef(1,0)*dt/2 + x.coeffRef(2,0);

	G << 1, 0, -b*sin(theta),
		 0, 1,  b*cos(theta),
		 0, 0,             1;
	return G;
}

MatrixXf EKF::jacobV(MatrixXf x, MatrixXf u, float dt)
{
	/* ヤコビ行列
	 * x : 状態
	 * u : 制御
	 * dt: 前ステップとの時間差
	 */
	MatrixXf V(3,2);
	float theta = u.coeffRef(1,0)*dt/2 + x.coeffRef(2,0);
	float v     = u.coeffRef(0,0);
	
	V << dt*cos(theta), (-v*dt*dt)*sin(theta)/2,
		 dt*sin(theta),  (v*dt*dt)*cos(theta)/2,
		             0,                      dt;
		 
	return V;
}

MatrixXf EKF::jacobM(MatrixXf u, double s_input[])
{
	/* ヤコビ行列
	 * u : 制御
	 * s_input: 制御系の計測誤差パラメータ
	 */
	MatrixXf M(2,2);
	float v  = u.coeffRef(0,0);
	float w  = u.coeffRef(1,0);
	float a1 = (float)s_input[0];
	float a2 = (float)s_input[1];
	float a3 = (float)s_input[2];
	float a4 = (float)s_input[3];
	
	M << a1*v*v + a2*w*w,               0,
		               0, a3*v*v + a4*w*w;
		 
	return M;
}

MatrixXf EKF::jacobH(MatrixXf x)
{
	/* ヤコビ行列
	 * x : 状態
	 */
	MatrixXf H(3,3);

	H<< 1, 0, 0,
	    0, 1, 0,
	    0, 0, 1;
	 
	 return H;
}

void initCluster( clusterInfo& cluster,
				  PointI& init_pt)
{
	// float init_r      = 0.01; // 観測誤差
	// float init_p      = 0.001;  //初期位置の分散値
	float init_p     = 10;  //初期位置の分散値
	float init_theta = 0;
	// cluster.x << init_pt.x, init_pt.y, 0.0, 0.0;
	cluster.x << init_pt.x, init_pt.y, init_theta;
	cluster.P << init_p,    0.0,   0.0,
		 		    0.0, init_p,   0.0,
		 		    0.0,    0.0, init_p;
	// cluster.R << init_r,    0.0,
	// 			    0.0, init_r;
	cluster.pre_vel.x       = 0;
	cluster.pre_vel.y       = 0;
	cluster.u               = Eigen::VectorXf::Zero(2);
	cluster.track_num       = 1;
	cluster.confidence      = 1;
	cluster.width           = init_pt.normal_x;
	cluster.length          = init_pt.normal_y;
	cluster.height          = init_pt.normal_z;
	cluster.count           = 0;
	// cluster.pre_position.resize(PRE_SIZE);
	cluster.pre_position.resize(1);
	cluster.pre_position[0] = init_pt;
	cluster.label = 0; // static: 0  dynamic: 1
	cluster.update_comp_flag = true;
	cluster.init_flag = true;
	cluster.human_update_flag = true;

	cluster.velocity = 0;
}

void Prediction( clusterInfo& cluster,
				 double dt,
				 double s_input[])
{
	EKF ekf;
	/* u   : (v, w)の転置行列　v:並進速度, w:角速度
	 * x   : (x, y, θ)の転置行列
	 * dt	   : 前ステップからの経過時間
	 * s_input : 動作モデルのノイズパラメータ
	 */


	MatrixXf Gt = MatrixXf::Zero(3,3);
	MatrixXf Vt = MatrixXf::Zero(3,2);
	MatrixXf Mt = MatrixXf::Zero(2,2);
	

	// Gt = ekf.jacobG(cluster.x, cluster.u, dt, pitch);
	Gt = ekf.jacobF(cluster.x, cluster.u, dt);
	Vt = ekf.jacobV(cluster.x, cluster.u, dt);
	Mt = ekf.jacobM(cluster.u, s_input);


	cluster.x = ekf.move(cluster.x, cluster.u, dt);
	cluster.P = Gt*cluster.P*Gt.transpose() + Vt*Mt*Vt.transpose();

	// cluster.label = 1;
}

void MeasurementUpdate(	clusterInfo& cluster,
						PointI& obj_centroid,
						double dt,
						double s_measurement[])
{
	/* x	: 状態(x, y, yaw)の転置行列
	 * u	: 制御(v, w)の転置行列
	 * s_measurement: 観測ノイズ
	 * sigma: 推定誤差
	 */

	EKF ekf;


	MatrixXf Z = MatrixXf::Zero(3,1);	// 観測 (x,y,θ)
	MatrixXf Q = MatrixXf::Zero(3,3);
	MatrixXf H = MatrixXf::Zero(3,3);
	MatrixXf y = MatrixXf::Zero(3,1);
	MatrixXf S = MatrixXf::Zero(3,3);
	MatrixXf K = MatrixXf::Zero(3,3);
	MatrixXf I = MatrixXf::Identity(3,3);

	Z.coeffRef(0,0) = obj_centroid.x;
	Z.coeffRef(1,0) = obj_centroid.y;
	Z.coeffRef(2,0) = atan2((obj_centroid.y-cluster.x[1]), (obj_centroid.x-cluster.x[0]));


	// cluster.u[0] = sqrt(pow((obj_centroid.y-cluster.x[1]), 2) + pow((obj_centroid.x-cluster.x[0]), 2))/dt;
	// cluster.u[0] = sqrt(pow((obj_centroid.y-cluster.x[1]), 2) + pow((obj_centroid.x-cluster.x[0]), 2))/dt;
	cluster.u[0] = cluster.velocity;

	if(cluster.init_flag){
	// cluster.u[1] = atan2((obj_centroid.y-cluster.x[1]), (obj_centroid.x-cluster.x[0]))/dt;
	cluster.u[1] = 0;
	cluster.init_flag = false;
	}

	// else cluster.u[1] = (atan2((obj_centroid.y-cluster.x[1]), (obj_centroid.x-cluster.x[0])) - cluster.x[2])/dt;
	else cluster.u[1] = (Z.coeffRef(2,0) - cluster.x[2])/dt;
	
	Q.coeffRef(0,0) = (float)s_measurement[0];
	Q.coeffRef(1,1) = (float)s_measurement[1];
	Q.coeffRef(2,2) = (float)s_measurement[2];

	H = ekf.jacobH(cluster.x);
	y = Z - H*cluster.x;
	S = H*cluster.P*H.transpose() + Q;
	K = cluster.P*H.transpose()*S.inverse();

	cluster.x = cluster.x + K*y;
	cluster.P = (I - K*H)*cluster.P;

	int pre_pos_size = cluster.pre_position.size();
	if(pre_pos_size > 1){
		cluster.velocity = sqrt(pow((cluster.pre_position[pre_pos_size-1].x-cluster.pre_position[pre_pos_size-2].x), 2.0) + pow((cluster.pre_position[pre_pos_size-1].y-cluster.pre_position[pre_pos_size-2].y), 2.0)) / cluster.dt;
		cluster.pre_position[pre_pos_size-1].intensity = cluster.velocity; // 速度を格納
		if(pre_pos_size == 5){
			double sum = 0;
			for(int i=0; i<pre_pos_size; i++) sum += cluster.pre_position[i].intensity;
			cluster.velocity = sum/pre_pos_size;
		}
	}

	if(cluster.velocity > MIN_VELOCITY && cluster.velocity < MAX_VELOCITY) cluster.label = 1;
	if(cluster.velocity > 2.0) cluster.label = 0;

	// cout<<"pre_position = "<<cluster.pre_position.size()<<endl;
		// for (int j=0; j<cluster.pre_position.size(); j++){
		// 	// cout<<"label = "<<clusters[i].label<<endl;
        //
			// if(cluster.confidence > 0 && cluster.label == 1){ // label 0:static 1:dynamic
		// 		cout<<" confidence = "<<cluster.confidence<<" pre_pos["<<j<<"] x = "<<cluster.pre_position[j].x<<" y = "<<cluster.pre_position[j].y<<endl;
		// 	}
		// }
}
