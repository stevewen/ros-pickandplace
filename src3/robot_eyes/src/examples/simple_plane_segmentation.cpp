#include <ros/ros.h>

// Point cloud message definition
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// PCL segmentation lib and dpendencies 
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
// Logging capabilities
#include <ros/console.h>


// Global ros publisher instance
ros::Publisher pub;

// Callback for the cloud_msg topic 
void segment(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
  // Container for original & segmented data
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_segmented (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Convert to PCL PointCloud<PointXYZRGB> since the SAC segmentation API does not work with PCL::PCLPointCloud2
  // and we dont want to loose the color information.
  pcl::fromROSMsg(*cloud_msg, *cloud);

  // Create intance of a geometric model parameters/coefficients and a structure to place 
  // inliner indices in the cloud structure.
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Use a plane model for the fitting algorithm, the plane models are:
  //   Its Hessian Normal coeficients: [normal_x normal_y normal_z distance_to_origin]
  seg.setModelType (pcl::SACMODEL_PLANE);

  // Use RANSAC model fitting algorithm to detect:
  //    1. Model parameters ()
  //    2. Inliner points with a maximum distance to the fitted plane model of 1 [cm] 
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  
  // Perform segmentation
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);  
  // Warn user if no inliners were detected.
  ROS_DEBUG_COND( inliers->indices.size() < 0, "No inliners points were detected");
  ROS_DEBUG("Plane Model coefficients: [ normal_x:%.3f,normal_x:%.3f,normal_x:%.3f,normal_x:%.3f ]",
                                                             coefficients->values[0] ,
                                                             coefficients->values[1] ,
                                                             coefficients->values[2] ,
                                                             coefficients->values[3]);
  ROS_DEBUG("Number of inliers: %u, Total number of points in cloud: %d; %0.2f percentage of points belong to the detected plane model",
            (int) inliers->indices.size(), cloud->width * cloud->height, (float) inliers->indices.size() * 100/( cloud->width * cloud->height));

  // Create a duplicate of the input cloud to color segment.
  cloud_segmented = cloud;

  // Color all inliers of the detected plane.
  for(int inlier_point = 0; inlier_point < inliers->indices.size(); inlier_point++){
    // Create RGB color code.
    uint32_t rgb = (static_cast<uint32_t>(16) | static_cast<uint32_t>(8) | static_cast<uint32_t>(16));
    // Color point.
    cloud_segmented->points[inliers->indices[inlier_point]].rgb = *reinterpret_cast<float*>(&rgb);
  }

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud_segmented, output);
  // Publish the data
  pub.publish (output);
  ROS_DEBUG("Publishing segmented cloud");
}

int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "simple_plane_segmentation");
  ros::NodeHandle nh;

  // Change console log level to DEBUG. (Optional)
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }
  
  ROS_INFO("Plane segmentation Node initiaded");
  
  // Create a ROS subscriber for the input point cloud
  // Use the previously defined callback to manage the subsctiption to the mesages.
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, segment);
  ROS_DEBUG("Subscribed to PointCloud2 messages");

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("robot_eyes/plane_segmentation", 1);
  ROS_DEBUG("Publisher of downsampled pointcloud instanciated");

  // Spin
  ros::spin ();
}