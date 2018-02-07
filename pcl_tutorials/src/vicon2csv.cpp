#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <inttypes.h>
// #include <object_msgs/ObjectParameter.h> 
// #include <object_msgs/ObjectParameterArray.h> 

#define RATE 5

using namespace std;

FILE *fp;
bool cb_flag   = false;
bool init_flag1 = true;
bool init_flag2 = true;
geometry_msgs::TransformStamped vicon;

float x; 
float y; 
float z; 
float q_x; 
float q_y; 
float q_z; 
float q_w; 

float pre_x;
float pre_y;
float pre_z;

double roll;
double pitch;
double yaw;
double velocity = 0;
double dt = 1;
// time_t now;
ros::Time now;

struct timeval start, finish;

geometry_msgs::Quaternion geo_q;

float distance(void)
{
	return sqrt((pre_x-x)*(pre_x-x) + (pre_y-y)*(pre_y-y));
}

void objectCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{

	if(init_flag2==false){
		gettimeofday(&finish, NULL);
		dt = ((double)finish.tv_sec - (double)start.tv_sec) + ((double)finish.tv_usec*1.0e-6 - (double)start.tv_usec*1.0e-6);
	}
	gettimeofday(&start, NULL);


	x   = msg -> transform.translation.x;
	y   = msg -> transform.translation.y;
	z   = msg -> transform.translation.z;
	// q_x = msg -> transform.rotation.x;
	// q_y = msg -> transform.rotation.y;
	// q_z = msg -> transform.rotation.z;
	// q_w = msg -> transform.rotation.w;
	geo_q = msg -> transform.rotation;
	now   = msg -> header.stamp;

	cout << "---------- Callback -----------" << endl;

	cb_flag    = true;
	init_flag2 = false;
}

void GetRPY(const geometry_msgs::Quaternion q, double &roll, double &pitch, double &yaw){
	tf::Quaternion quat(q.x, q.y, q.z, q.w);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

void vicon2csv(void)
{

	if(init_flag1){
		pre_x = x;
		pre_y = y;
		init_flag1 = false;
	}

	velocity = distance() / dt;
	GetRPY(geo_q, roll, pitch, yaw);
	fprintf(fp,"%lf,", ((double)now.sec + (double)now.nsec*1.0e-9));
	fprintf(fp,"%lf,%lf,%lf,%lf,%lf\n", x, y, yaw, velocity, dt);
	cout << "\x1b[31mCalculation time :" << dt << " [sec]\x1b[0m" << endl;

	pre_x = x;
	pre_y = y;
}

int main(int argc, char** argv)
{
    // fp = fopen("/home/amsl/velocity_estimation/infant_vicon.csv","w");
    fp = fopen("/home/amsl/velocity_estimation/human_vicon.csv","w");
    fprintf(fp,"time(vicon),x(vicon),y(vicon),yaw(vicon),velocity(vicon),dt(vicon)\n");

    ros::init(argc, argv, "vicon2csv");
    ros::NodeHandle n;
    ros::Rate loop_rate(RATE);

    // ros::Subscriber sub = n.subscribe("/vicon/infant/infant", 1, objectCallback);
    ros::Subscriber sub = n.subscribe("/vicon/human/human", 1, objectCallback);

    cout<<"start"<<endl;

	while(ros::ok()){
		if(cb_flag){
			vicon2csv();
		}
		// cout<<now<<endl;
		cb_flag = false;
		ros::spinOnce();
		loop_rate.sleep(); 
	}

	fclose(fp);
	return (0);
}

