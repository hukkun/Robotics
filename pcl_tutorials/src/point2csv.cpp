#include <ros/ros.h>
#include <object_msgs/ObjectParameter.h> 
#include <object_msgs/ObjectParameterArray.h> 
// #include <tf/tf.h>
#include <tf2_msgs/TFMessage.h>

using namespace std;

FILE *fp;
bool cb_flag = false;
object_msgs::ObjectParameterArray objects_;
ros::Time now;

void objectCallback(const object_msgs::ObjectParameterArrayConstPtr& msg)
{
	objects_.objects.resize(msg->objects.size());
	objects_.objects = msg->objects;
	cb_flag = true;
}

void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
	now = msg -> transforms[0].header.stamp;
}

void point2csv(void)
{
	int num_object   = objects_.objects.size();
	bool update_flag = false;
	for(int i=0; i<num_object; i++){
		object_msgs::ObjectParameter object = objects_.objects[i];
		if(object.x != 0){
			fprintf(fp,"%lf,%ld,%lf,%lf,%lf,%lf", (now.sec + now.nsec*1.0e-9), object.id, object.x, object.y, object.yaw, object.velocity);
			update_flag = true;
		}
	}

	if(update_flag) fprintf(fp,"\n");
}

int main(int argc, char** argv)
{
    fp = fopen("/home/amsl/velocity_estimation/infant.csv","w");
    // fp = fopen("/home/amsl/velocity_estimation/human.csv","w");
    fprintf(fp,"time,id,x,y,yaw,velocity\n");

    ros::init(argc, argv, "point2csv");
    ros::NodeHandle n;
    ros::Rate loop_rate(5);

    ros::Subscriber sub1 = n.subscribe("/objects", 1, objectCallback);
    ros::Subscriber sub2 = n.subscribe("/tf", 1, tfCallback);

    cout<<"start"<<endl;

	while(ros::ok()){
		if(cb_flag){
			point2csv();
		}
		cb_flag = false;
		ros::spinOnce();
		loop_rate.sleep(); 
	}

	fclose(fp);
	return (0);
}

