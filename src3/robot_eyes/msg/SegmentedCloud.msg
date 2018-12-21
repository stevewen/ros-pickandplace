# Msg describing a segmented cloud.
std_msgs/Header header
pcl_msgs/PointIndices[]             clusters                    # Array of PointIndices objects holding each cluster inliers
string[]                            clusters_id                 # Array of strings identifying the clusters 