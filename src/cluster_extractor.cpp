#include <iostream>
#include <fstream>
#include <memory>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// for added pcl::PointCloud ros message function
#include <pcl_ros/point_cloud.h>


//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------
#define MAX_CLUSTERS 5
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

std::string filename;

//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------

class ClusterExtractor
{

private:
    ros::NodeHandle n_;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub[MAX_CLUSTERS];
    ros::Publisher point_pub[MAX_CLUSTERS];
    tf::TransformBroadcaster br;


public:
    ClusterExtractor()
    {
        ROS_DEBUG("Creating subscribers and publishers");
        cloud_sub = n_.subscribe("/outlier/cutoff/output", 10, &ClusterExtractor::cloudcb, this);
        br = tf::TransformBroadcaster();
        for(int i = 0; i < MAX_CLUSTERS; i++)
        {
            cloud_pub[i] = n_.advertise<PointCloud>("/cluster_" + std::to_string(i + 1) + "_cloud", 1);
            point_pub[i] = n_.advertise<geometry_msgs::PointStamped>("/cluster_" + std::to_string(i + 1) + "_point", 1);
        }
    }

    // this function gets called every time new pcl data comes in
    void cloudcb(const sensor_msgs::PointCloud2ConstPtr &scan)
    {
        ROS_DEBUG("Filtered cloud receieved");
        PointCloud::Ptr ros_cloud(new PointCloud);
        PointCloud::Ptr cloud (new PointCloud);

        // set time stamp and frame id
        ros::Time tstamp = ros::Time::now();

        // Convert to pcl
        ROS_DEBUG("Convert incoming cloud to pcl cloud");
        pcl::fromROSMsg(*scan, *cloud);
       
        ////////////////////////////////////////
        // STARTING CLUSTER EXTRACTION    //
        ////////////////////////////////////////
        ROS_DEBUG("Begin cluster extraction");

        // create a vector for storing the indices of the clusters
        std::vector<pcl::PointIndices> cluster_indices;

        // setup extraction:
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        //ec.setClusterTolerance (0.01); // cm
        ec.setClusterTolerance(0.03); // 0.05 was too much
        ec.setMinClusterSize (50);
        ec.setMaxClusterSize (5000);
        ec.setInputCloud (cloud);
        // perform cluster extraction
        ec.extract (cluster_indices);

        // run through the indices, create new clouds, and then publish them
        int j=0;
        int number_clusters=0;
        geometry_msgs::PointStamped pt;
        Eigen::Vector4f centroid;

        for(const auto & index : cluster_indices)
        {
            number_clusters = (int) cluster_indices.size();
            ROS_DEBUG("Number of clusters found: %d",number_clusters);
            PointCloud::Ptr cloud_cluster (new PointCloud);
            for (const auto & point : index.indices)
            {
                cloud_cluster->points.push_back(cloud->points[point]);
            }
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            cloud_cluster->header.frame_id = "camera_depth_optical_frame";


            // convert to rosmsg and publish:
            ROS_DEBUG("Publishing extracted cloud");
            // pcl::toROSMsg(*cloud_cluster, *ros_cloud);
            // ros_cloud->header.frame_id = scan->header.frame_id;
            if(j < MAX_CLUSTERS)
            {
                //cloud_pub[j].publish(ros_cloud);
                cloud_pub[j].publish(cloud_cluster);

                // compute centroid and publish
                pcl::compute3DCentroid(*cloud_cluster, centroid);
                pt.point.x = centroid(0);
                pt.point.y = centroid(1);
                pt.point.z = centroid(2);
                pt.header.stamp = scan->header.stamp;
                pt.header.frame_id = scan->header.frame_id;
                point_pub[j].publish(pt);

                // let's send transforms as well:
                tf::Transform transform;
                transform.setOrigin( tf::Vector3(centroid(0), centroid(1), centroid(2)) );
                tf::Quaternion q;
                q.setRPY(0, 0, 0);
                transform.setRotation(q);
                br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), scan->header.frame_id, "cluster_" + std::to_string(j + 1) + "_frame"));
            }
            j++;
        }
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cluster_extractor");
    ClusterExtractor extractor;
  
    ros::spin();
  
    return 0;
}
