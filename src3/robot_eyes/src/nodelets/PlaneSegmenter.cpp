#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <pcl_ros/pcl_nodelet.h>


#include <robot_eyes/SegmentedCloud.h>

class CloudColorer : public pcl_ros::PCLNodelet{

    public:
        typedef robot_eyes::SegmentedCloud SegmentedCloud;
        typedef sensor_msgs::PointCloud2 PointCloud2;

        typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
        typedef PointCloud::Ptr PointCloudPtr;
        typedef PointCloud::ConstPtr PointCloudConstPtr;

        typedef pcl_msgs::PointIndices PointIndices;
        typedef PointIndices::Ptr PointIndicesPtr;
        typedef PointIndices::ConstPtr PointIndicesConstPtr;
    
    private:
    boost::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<PointCloud2, SegmentedCloud> > > sync_input_indices_a_;
    
    protected:

    std::vector<uint32_t> cluster_color_array;
    message_filters::Subscriber<PointCloud2> sub_input_filter_;
    message_filters::Subscriber<SegmentedCloud> sub_clusters_filter_;
        

    /** This method is a callback from the nodelet_topic_toolds::NodeletLazy class indicating that a subscriber has subscribed to this nodelet 
        publisher topics.
        We use this information to start procesing input and publishing output data (avoid CPU loading when no output is required)
    */
    virtual void subscribe(){
        // Subscribe to the input and normals using filters
        sub_input_filter_.subscribe (*pnh_, "input", max_queue_size_);
        sub_clusters_filter_.subscribe (*pnh_, "clusters", max_queue_size_);

        sync_input_indices_a_ = boost::make_shared <message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<PointCloud2, SegmentedCloud> > > (max_queue_size_);
        sync_input_indices_a_->connectInput (sub_input_filter_, sub_clusters_filter_);
        sync_input_indices_a_->registerCallback (boost::bind(&CloudColorer::input_indices_callback, this, _1, _2));
        pub_output_ = advertise<PointCloud2>(*pnh_,"colored", max_queue_size_);
    }

    /** This is a callback to a joint incomming message of sensor_msgs::Pointcloud2 and robot_eyes::SegmentedCloud, which are tied together by an
        ApproximateTime syncing policy, defined on the message_filters package.
        \param cloud: Input cloud
        \param segmented_cloud: Array of pcl_msgs::PointIndices indicating clusters inliers, and clusters id.
    */
    void input_indices_callback (const PointCloud2::ConstPtr &cloud, const SegmentedCloud::ConstPtr segmented_cloud){
        // Container for original to segment.
        PointCloudPtr cloud_pcl (new PointCloud);
        std::vector<PointIndices> clusters = segmented_cloud->clusters;
        std::vector<std::string> clusters_id = segmented_cloud->clusters_id;
        // Create a container for the data.
        pcl::fromROSMsg(*cloud, *cloud_pcl); 
        // Color original cloud for visualization purposes
        generateClusterColor(clusters.size());
        // Iterate over all inliers arrays.
        for(std::vector<pcl_msgs::PointIndices>::iterator cluster = clusters.begin(); cluster != clusters.end(); cluster++){  
            // Color all inliers of a cluster with the same color 
            for(int inlier_point = 0; inlier_point < cluster->indices.size(); inlier_point++){
                uint32_t rgb = cluster_color_array[std::distance(clusters.begin(), cluster)];
                cloud_pcl->points[cluster->indices[inlier_point]].rgb = *reinterpret_cast<float*>(&rgb); // Color point.
            }
        }
        // Convert to ROS data type.
        PointCloud2 output;
        pcl::toROSMsg(*cloud_pcl, output);
        // Publish the output cloud.
        pub_output_.publish(output);
    }

    /** This method is a callback from the nodelet_topic_toolds::NodeletLazy class indicating that a subscriber has unsubscribed to this nodelet 
        publisher topics
        We use this information to stop procesing input data (avoid CPU loading when no output is required)
    */
    virtual void unsubscribe(){
        sub_input_filter_.unsubscribe();
        sub_clusters_filter_.unsubscribe();
        pub_output_.shutdown();
    }
    

    // inline bool isValid( const SegmentedCloud::ConstPtr segmented_cloud , const std::string &topic_name = "indices"){
    //     if( segmented_cloud.clusters.size() == 0 ){
    //         NODELET_WARN("[%s] Empty clusters array, no clusters given as input, no color operation will be performed", getName ().c_str ());
    //         return false;
    //     }
    // }


    /* This method populates a vector of rgb colors ordered for the incomming clusters,
            mantaining the color for the clusters if they come in the same order, for example
            if cluster 1 is a table and 2 is a chair, meanwhile the ordet of the incomming clusters
            mantains the colors will mantain. This method works for any number of clusters.*/
    void generateClusterColor(int number_of_clusters){
        while( cluster_color_array.size() < number_of_clusters ){
            uint8_t r = rand() % 255, g = rand() % 255, b = rand() % 255; 
            // Pack r/g/b into rgb
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            cluster_color_array.push_back(rgb);
        }
    }
};
