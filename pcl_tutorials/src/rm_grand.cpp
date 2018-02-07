//#include <velodyne_height_map/heightmap.h>
#include <iostream>
#include <vector>
using namespace std;
namespace velodyne_height_map {

#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))
VPoint obs;

HeightMap::HeightMap(ros::NodeHandle node, ros::NodeHandle priv_nh)
{
  // get parameters using private node handle
  //priv_nh.param("cell_size", m_per_cell_, 1.5);//0.15);
  priv_nh.param("cell_size", m_per_cell_, 1.0);//0.15);
  priv_nh.param("full_clouds", full_clouds_, true);
  priv_nh.param("grid_dimensions", grid_dim_, 40);
  priv_nh.param("height_threshold", height_diff_threshold_, 0.50);//0.55);
  
  ROS_INFO_STREAM("height map parameters: "
                  << grid_dim_ << "x" << grid_dim_ << ", "
                  << m_per_cell_ << "m cells, "
                  << height_diff_threshold_ << "m threshold, "
                  << (full_clouds_? "": "not ") << "publishing full clouds");

  // Set up publishers  
  obstacle_publisher_ = node.advertise<VPointCloud>("rm_ground",1);
  clear_publisher_ = node.advertise<VPointCloud>("ground",1);  

  // subscribe to Velodyne data points
  velodyne_scan_ = node.subscribe("/velodyne_points", 10, 
                                  &HeightMap::processData, this,
                                  ros::TransportHints().tcpNoDelay(true));
}

HeightMap::~HeightMap() {}

void HeightMap::constructFullClouds(const VPointCloud::ConstPtr &scan,unsigned npoints, size_t &obs_ count,size_t &empty_count)
{
  float min[grid_dim_][grid_dim_];
  float max[grid_dim_][grid_dim_];
  bool init[grid_dim_][grid_dim_];
  memset(&init, 0, grid_dim_*grid_dim_);
    int clears=0;

  // build height map
  for (unsigned i = 0; i < npoints; ++i) {
    bool head_cut=(scan->points[i].z>0.8);
    bool rmzero=(scan->points[i].x==0.0&&scan->points[i].y==0.0);//&&scan->points[i].z<1.0);
    if(rmzero||head_cut)continue;
    //if((scan->points[i].z<-2.2&&scan->points[i].x<30.0&&scan->points[i].y>-12.0)) continue;
    //if(scan->points[i].z>1.0) continue;
    int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
    int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);

    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_) {
      if (!init[x][y]) {
        min[x][y] = scan->points[i].z;
        max[x][y] = scan->points[i].z;

        init[x][y] = true;
      } else {
        min[x][y] = MIN(min[x][y], scan->points[i].z);
        max[x][y] = MAX(max[x][y], scan->points[i].z);
      }
    }
  }

  // display points where map has height-difference > threshold
  for (unsigned i = 0; i < npoints; ++i) {
    bool rmzero=(scan->points[i].x!=0.0&&scan->points[i].y!=0.0);//&&scan->points[i].z<1.0);
    bool distance_cut=(scan->points[i].normal_z<16.0);
    //if(scan->points[i].z>0.7) continue;//if indoor, this use.
    int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
    int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y]) {
        //bool bottom_cut=(scan->points[i].z>min[x][y]+((max[x][y]-min[x][y])*0.1));
        bool bottom_cut;
        bottom_cut=(scan->points[i].z>min[x][y]+0.1);
        // if(12.0>scan->points[i].normal_z)bottom_cut=(scan->points[i].z>min[x][y]+0.1);
        // else bottom_cut=1;
        bool head_cut=(scan->points[i].z<0.8);
        if ((max[x][y] - min[x][y] > height_diff_threshold_)
              && rmzero && head_cut &&bottom_cut && distance_cut){
            obstacle_cloud_.points[obs_count].x = scan->points[i].x;
            obstacle_cloud_.points[obs_count].y = scan->points[i].y;
            obstacle_cloud_.points[obs_count].z = scan->points[i].z;
            obstacle_cloud_.points[obs_count].normal_x = scan->points[i].normal_x;
            obstacle_cloud_.points[obs_count].normal_y = scan->points[i].normal_y;
            obstacle_cloud_.points[obs_count].normal_z = scan->points[i].normal_z;
            obstacle_cloud_.points[obs_count].intensity = scan->points[i].intensity;
            obstacle_cloud_.points[obs_count].curvature = scan->points[i].curvature;
            obs_count++;
        }
        else{
            clear_cloud_.points[empty_count].x = scan->points[i].x;
            clear_cloud_.points[empty_count].y = scan->points[i].y;
            clear_cloud_.points[empty_count].z = scan->points[i].z;
            clear_cloud_.points[empty_count].normal_x = scan->points[i].normal_x;
            clear_cloud_.points[empty_count].normal_y = scan->points[i].normal_y;
            clear_cloud_.points[empty_count].normal_z = scan->points[i].normal_z;
            clear_cloud_.points[empty_count].intensity = scan->points[i].intensity;
            clear_cloud_.points[empty_count].curvature = scan->points[i].curvature;
            empty_count++;
        }
    }
    else{
        clears++;
        clear_cloud_.points[empty_count].x = scan->points[i].x;
        clear_cloud_.points[empty_count].y = scan->points[i].y;
        clear_cloud_.points[empty_count].z = scan->points[i].z;
        clear_cloud_.points[empty_count].normal_x = scan->points[i].normal_x;
        clear_cloud_.points[empty_count].normal_y = scan->points[i].normal_y;
        clear_cloud_.points[empty_count].normal_z = scan->points[i].normal_z;
        clear_cloud_.points[empty_count].intensity = scan->points[i].intensity;
        clear_cloud_.points[empty_count].curvature = scan->points[i].curvature;
        empty_count++;
    }

  }
      cout<<"all="<<original_cloud_.points.size()<<", obstacle="<<obs_count<<endl;

}

/** point cloud input callback */
void HeightMap::processData(const VPointCloud::ConstPtr &scan)
{
  if ((obstacle_publisher_.getNumSubscribers() == 0) && (clear_publisher_.getNumSubscribers() == 0)  && (original_publisher_.getNumSubscribers() == 0))return;

  // pass along original time stamp and frame ID
  obstacle_cloud_.header.stamp = scan->header.stamp;
  obstacle_cloud_.header.frame_id = scan->header.frame_id;

  // pass along original time stamp and frame ID
  clear_cloud_.header.stamp = scan->header.stamp;
  clear_cloud_.header.frame_id = scan->header.frame_id;

  original_cloud_.header.stamp = scan->header.stamp;
  original_cloud_.header.frame_id = scan->header.frame_id;

  // set the exact point cloud size -- the vectors should already have
  // enough space
  size_t npoints = scan->points.size();
  obstacle_cloud_.points.resize(npoints);
  clear_cloud_.points.resize(npoints);
  original_cloud_.points.resize(npoints);

  /*for(size_t i=0;i<npoints;i++){
    original_cloud.points[i].x=scan->points[i].x;
    original_cloud.points[i].y=scan->points[i].y;
    original_cloud.points[i].z=scan->points[i].z;
    original_cloud.points[i].normal_x=scan->points[i].normal_x;
    original_cloud.points[i].normal_y=scan->points[i].normal_y;
    original_cloud.points[i].normal_z=scan->points[i].normal_z;
    original_cloud.points[i].intensity=scan->points[i].intensity;
    original_cloud.points[i].curvature=scan->points[i].curvature;
  }*/
  size_t obs_count=0;
  size_t empty_count=0;
  // either return full point cloud or a discretized version
  constructFullClouds(scan,npoints,obs_count, empty_count);

  obstacle_cloud_.points.resize(obs_count);
  clear_cloud_.points.resize(empty_count);

  if (obstacle_publisher_.getNumSubscribers() > 0)obstacle_publisher_.publish(obstacle_cloud_);
  if (clear_publisher_.getNumSubscribers() > 0)clear_publisher_.publish(clear_cloud_);
  if (original_publisher_.getNumSubscribers() > 0)original_publisher_.publish(original_cloud_);
}

} // namespace velodyne_height_map
