#include <ros/ros.h>
#include <std_msgs/Bool.h>

using namespace std;

int main(int argc, char** argv){
	
	ros::init(argc, argv, "pub_flag");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<std_msgs::Bool>("/flag",1);
	ros::Rate loop_rate(10);

	int a = 0;
	std_msgs::Bool flag;
	flag.data = false;
	cout<<"start"<<endl;

	while(ros::ok()){
		cout<<"while start"<<endl;
		while((a = getchar()) != '\n'){
			flag.data = false;
			pub.publish(flag);
			cout<<"pub flase"<<endl;
		}
		cout<<"pressed ENTER!!!!!!!!!!!!!!!!"<<endl;
		flag.data = true;
		pub.publish(flag);
		cout<<"pub true"<<endl;

		// ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
