#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <ros/console.h>

ros::Publisher pub;

// Callback for the cloud_msg topic 
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);
  
  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(cloud_filtered, output);
  // Publish the data
  pub.publish (output);
  ROS_INFO("Publishing downsampled cloud");
}

int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "example_voxelgrid");
  ros::NodeHandle nh;

  // Change console log level
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }
  
  ROS_INFO("Downsampling node initiated");
  // Create a ROS subscriber for the input point cloud
  // Use the previously defined callback to manage the subsctiption to the mesages.
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_cb);
  ROS_INFO("Subscriber to PointCloud2 messages created");
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("robot_eyes/voxel_grid/downsample", 1);
  ROS_INFO("Publisher of downsampled pointcloud instanciated");
  // Spin
  ros::spin ();
}