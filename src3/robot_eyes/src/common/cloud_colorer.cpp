#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// Robot_eyes messages
#include <robot_eyes/segmented_cloud.h>
#include <robot_eyes/inliers_indices.h>

void generateClusterColor(int);

ros::Publisher pub;
std::vector<uint32_t> cluster_color_array;

/* colorCloud: This method is used as callback for the main subscriber of the `cloud_colorer` node, 
              the topic to which this method responds to uses message of type `robot_eyes::segmented_cloud`,
              and uses the cluster inliers to color the cloud to visualize clusters graphically.            
*/
void colorCloud(const robot_eyes::segmented_cloud segmented_cloud){
  // Container for original to segment.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  // Create a container for the data.
  pcl::fromROSMsg(segmented_cloud.cloud, *cloud);                                              //Cloud to color 
  std::vector<robot_eyes::inliers_indices> inliers = segmented_cloud.segmented_inliers;
  ROS_DEBUG( "cloud_colorer: coloring %d clusters", (int) inliers.size() );
  // Iterate over all inliers arrays.
  for(std::vector<robot_eyes::inliers_indices>::iterator cluster = inliers.begin(); cluster != inliers.end(); cluster++){
    // Color original cloud for visualization purposes
    generateClusterColor(inliers.size());
    // Color all inliers of a cluster with the same color 
    for(int inlier_point = 0; inlier_point < cluster->inliers.size(); inlier_point++){
        uint32_t rgb = cluster_color_array[std::distance(inliers.begin(), cluster)];
        cloud->points[cluster->inliers[inlier_point]].rgb = *reinterpret_cast<float*>(&rgb); // Color point.
    }
  }
  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud, output);
  // Publish the data.
  pub.publish (output);
}

int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "cloud_colorer");
  ros::NodeHandle nh;
  
  // Change console log level to DEBUG. (Optional)
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, colorCloud);
  // Create a ROS publisher for the output point cloud
  ROS_DEBUG("Publishing to output")
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  // Spin
  ros::spin ();
}


/* generateClusterColor: This method populates a vector of rgb colors ordered for the incomming clusters,
                         mantaining the color for the clusters if they come in the same order, for example
                         if cluster 1 is a table and 2 is a chair, meanwhile the ordet of the incomming clusters
                         mantains the colors will mantain. This method works for any number of clusters.
*/
void generateClusterColor(int number_of_clusters){
  while( cluster_color_array.size() < number_of_clusters ){
    uint8_t r = rand() % 255, g = rand() % 255, b = rand() % 255; 
    // Pack r/g/b into rgb
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    cluster_color_array.push_back(rgb);
  }
}