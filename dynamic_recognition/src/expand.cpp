#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>


float sum;
float before;

double qr;
double qp;
double qy;

double init_yaw;
double _robot_vel;
double _robot_angular;

bool   init_flag1 = true;
bool   init_flag2 = true;

using namespace std;

float expand(float after){ // imuのyawを連続値にする

	if(init_flag1){
		before = after;
		sum += after;
		init_flag1 = false;
	}

	else{
		if((before * after) < 0){

			if(fabs(before) > M_PI/2){ //180度付近
				if(before > 0){
					sum += (M_PI*2 - before + after);
				}
				else{
					sum -= (M_PI*2 + before - after);
				}
			}
			else{
				sum += (before - after);
			}
		}

		else{
			sum += (after - before);
		}

		before = after;
	}
	return sum;
}

void OdometryCallback(const nav_msgs::Odometry &msg)
{
	_robot_vel    = msg.twist.twist.linear.x;
}

void ImuCallback(const sensor_msgs::Imu::ConstPtr msg)
{
	tf::Quaternion quat(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	tf::Matrix3x3(quat).getRPY(qr, qp, qy);

	if(init_flag2){
		init_yaw   = qy;
		init_flag2 = false;
	}

	cout<<"init_yaw = "<<init_yaw<<" qy = "<<qy<<endl;

	qy -= init_yaw;
	
	_robot_angular = expand(qy);

	cout<<"yaw = "<<qy<<" expand yaw = "<<_robot_angular<<endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "expand");
	ros::NodeHandle nh;

	ros::Subscriber odom_sub = nh.subscribe("/odom",1, OdometryCallback);
	ros::Subscriber amu_sub  = nh.subscribe("/imu/data",1, ImuCallback);

	ros::spin();
	return 0;
}
