#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>

#include <random>

#define N_RND_SAMPLES 20

sensor_msgs::PointCloud2 msg; // Temporary pcl msg 
sensor_msgs::PointCloud2 msg0, msg1, msg2; // Msgs to publish on topics
ros::Publisher lidar_pub0, lidar_pub1, lidar_pub2; // Publishers to push msgs on topics

std::random_device rd;  // Will be used to obtain a seed for the random number engine
std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()

/**
 * Callback function, gets called whenever a LiDAR message arrives.
 * The purpose of the function is to assign each arriving message to its corresponding
 * channel (0, 1, or 2). The channels correspond to the individual laser-scanners, 
 * as the Livox Mid-100 basically is just three Mid-40s. 
 */
void lidarMSGcallback(const sensor_msgs::PointCloud2::ConstPtr &m)  
{
  // Setup message header, includes the sequence number of the scan.
  std_msgs::Header header;
  header.frame_id = m->header.frame_id;
  header.stamp = m->header.stamp;
  // header.stamp = ros::Time::now(); for debugging and recorded data in simulation 
  
  // Data should be the same, only redirect the message
  msg.height = m->height;
  msg.width = m->width;
  msg.data = m->data;
  msg.fields = m->fields;
  msg.is_bigendian = m->is_bigendian;
  msg.is_dense = m->is_dense;
  msg.point_step = m->point_step;
  msg.row_step = m->row_step;  

  // Convert point cloud to approproate format
  // sensor_msgs::PointCloud2 uses binary "data" field, we need points in xyz
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(msg,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

  // Find out which cone:
  double sum = 0.0;
  double theta;

  // Random approach:
  size_t n_points = temp_cloud->points.size();
  std::uniform_int_distribution<> dis(0, n_points);
  // Use only 20 Points
  int sample;
  for (int i = 0; i < N_RND_SAMPLES; ++i) {
    sample = dis(gen);
  // }
  // for (int i = 0; i < temp_cloud->points.size(); ++i) {
  // pcl::PointXYZI p = temp_cloud->points[i];
    pcl::PointXYZI p = temp_cloud->points[sample];
    theta = atan2(p.y, p.x);
    sum += theta;
  }
  sum /= N_RND_SAMPLES;
  theta = RAD2DEG(sum);
  if (-10.0 <= theta && theta <= 10.0 ) {
    msg0 = msg;
    msg0.header = header;
    lidar_pub0.publish(msg0);
    //ROS_INFO("Avg. theta: %f, means center", theta);
  } else if (theta >= 10.0) {
    msg1 = msg;
    msg1.header = header;
    lidar_pub1.publish(msg1);
    //ROS_INFO("Avg. theta: %f, means right", theta);
  } else if (theta <= -10.0) {
    msg2 = msg;
    msg2.header = header;
    lidar_pub2.publish(msg2);
    //ROS_INFO("Avg. theta: %f, means left", theta);
  }
}

/* Program entry */
int main(int argc, char **argv)
{
  // Init ROS Node
  ros::init(argc, argv, "livox_mid100_splitter");
  ros::NodeHandle n;

  // Subscription to Mid-100 topic (usually "/livox/lidar")
  ros::Subscriber lidar_sub = n.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 1000, lidarMSGcallback);
  
  // Advertise three new topics, each corresponding to one Mid-100 cone.
  lidar_pub0 = n.advertise<sensor_msgs::PointCloud2>("/livox/lidar_c", 1000);
  lidar_pub1 = n.advertise<sensor_msgs::PointCloud2>("/livox/lidar_r", 1000);
  lidar_pub2 = n.advertise<sensor_msgs::PointCloud2>("/livox/lidar_l", 1000);

  // Set ROS loop
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Exit normally
  return 0;
}
