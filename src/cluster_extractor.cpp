#include <iostream>
#include <fstream>
#include <memory>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nodelet_pcl_demo/ObjectArray.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// for added pcl::PointCloud ros message function
#include <pcl_ros/point_cloud.h>

// #include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>


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
    ros::Publisher objects_pub;
    tf::TransformBroadcaster br;
    tf::TransformListener tl; 

public:
   
    ClusterExtractor()
    {
        cloud_sub = n_.subscribe("/outlier/cutoff/output", 10, &ClusterExtractor::cloudcb, this);
        ROS_DEBUG("Creating subscribers and publishers");
        br = tf::TransformBroadcaster();
        for(int i = 0; i < MAX_CLUSTERS; i++)
        {
            cloud_pub[i] = n_.advertise<PointCloud>("/cluster_" + std::to_string(i + 1) + "_cloud", 1);
        }
        objects_pub = n_.advertise<nodelet_pcl_demo::ObjectArray>("/objects", 1);
    }

    moveit_msgs::CollisionObject addCylinder(float x, float y, float z, float r, float h, std::string id)
    {
        // BEGIN_SUB_TUTORIAL add_cylinder
        //
        // Adding Cylinder to Planning Scene
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Define a collision object ROS message.
        
        // geometry_msgs::TransformStamped transformStamped;
        // try{
        //     transformStamped = tfBuffer.lookupTransform("base_link", "camera_depth_optical_frame", ros::Time(0));
        // }
        // catch (tf2::TransformException &ex){
        //     ROS_WARN("%s", ex.what());
        //     ros::Duration(1.0).sleep();
        // }

        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "camera_depth_optical_frame";
        collision_object.id = id;

        // Define a cylinder which will be added to the world.
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        /* Setting height of cylinder. */
        primitive.dimensions[0] = h;
        /* Setting radius of cylinder. */
        primitive.dimensions[1] = r;

        // Define a pose for the cylinder (specified relative to frame_id).
        geometry_msgs::Pose cylinder_pose;
        /* Computing and setting quaternion from axis angle representation. */
        Eigen::Vector3d cylinder_z_direction(0., 0., 0.);
        Eigen::Vector3d origin_z_direction(0., 0., 1.);
        Eigen::Vector3d axis;
        axis = origin_z_direction.cross(cylinder_z_direction);
        axis.normalize();
        double angle = acos(cylinder_z_direction.dot(origin_z_direction));
        cylinder_pose.orientation.x = 2.2;
        cylinder_pose.orientation.y = axis.y() * sin(angle / 2);
        cylinder_pose.orientation.z = axis.z() * sin(angle / 2);
        cylinder_pose.orientation.w = cos(angle / 2);
        // cylinder_pose.orientation.x = transformStamped.transform.rotation.x;
        // cylinder_pose.orientation.y = transformStamped.transform.rotation.y;
        // cylinder_pose.orientation.z = transformStamped.transform.rotation.z;
        // cylinder_pose.orientation.w = transformStamped.transform.rotation.w;

        // Setting the position of cylinder.
        cylinder_pose.position.x = x;
        cylinder_pose.position.y = y;
        cylinder_pose.position.z = z;

        // Add cylinder as collision object
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(cylinder_pose);
        collision_object.operation = collision_object.ADD;
        // collision_objects.push_back(collision_object);
        // planning_scene_interface_.applyCollisionObject(collision_object);
        // END_SUB_TUTORIAL
        return collision_object;
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
        geometry_msgs::Vector3 size;

        Eigen::Vector4f centroid;
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.resize(MAX_CLUSTERS);

        // Create ObjectArray to store all of the centroids / types from 
        nodelet_pcl_demo::ObjectArray objects;

        tf::StampedTransform transform;

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
            
                //Populate the ObjectArray with an object
                // initialPoint.x = centroid(0);
                // initialPoint.y = centroid(1);
                // initialPoint.z = centroid(2);

                // calculate the min max 3D coordinates in x,y,z. In an ideal world with infinite time we would use a minimum volume bounding box.... but
                // Eigen::Vector4f min_pt;
                // Eigen::Vector4f max_pt;
                Eigen::Vector4f min_pt, max_pt;
                pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);
                size.x = max_pt[0] - min_pt[0];
                size.y = max_pt[1] - min_pt[1];
                size.z = max_pt[2] - min_pt[2];

                // look at the color for type encoding

                int type = 0;
                type += 1 * (cloud_cluster->points[0].r / 255);
                type += 2 * (cloud_cluster->points[0].g / 255);
                type += 3 * (cloud_cluster->points[0].b / 255);

                geometry_msgs::PointStamped transformedPoint;
                geometry_msgs::Point finalPoint;

                try{
                    //tl.lookupTransform("base_link", "camera_depth_optical_frame", ros::Time(0), transform)
                    tl.transformPoint("base_link", pt, transformedPoint);
                    finalPoint.x = transformedPoint.point.x;
                    finalPoint.y = transformedPoint.point.y;
                    finalPoint.z = transformedPoint.point.z;
                    objects.centroids.push_back(finalPoint);


                }
                catch (tf::TransformException ex) {
                    ROS_WARN("%s", ex.what());
                    ros::Duration(1.0).sleep();
                    continue;
                }

                objects.sizes.push_back(size);
                objects.types.push_back(type);

                objects.header.stamp = scan->header.stamp;
                objects.header.frame_id = scan->header.frame_id;


                // let's send transforms as well:
                tf::Transform transform;
                transform.setOrigin( tf::Vector3(centroid(0), centroid(1), centroid(2)) );
                tf::Quaternion q;
                q.setRPY(0, 0, 0);
                transform.setRotation(q);
                br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), scan->header.frame_id, "cluster_" + std::to_string(j + 1) + "_frame"));

                std::string id = "cylinder_" + std::to_string(j + 1);
                collision_objects.push_back(addCylinder(centroid(0), centroid(1), centroid(2), size.x - 0.05, size.z, id));
            }
            j++;
        // ros::Duration(2.0).sleep()        geometry_msgs::PointStamped initialPoint;
;
        }
        
        objects_pub.publish(objects);

        // remove the clusters that are not in the scene
        std::vector<moveit_msgs::CollisionObject> remove_objects;
        std::string id;
        
        for (int i = number_clusters + 1; i <= MAX_CLUSTERS; i++ ){
            moveit_msgs::CollisionObject obj; 
            id = "cylinder_" + std::to_string(i);
            obj.id = id;
            obj.operation = obj.REMOVE;
            collision_objects.push_back(obj);
        }
        // planning_scene_interface_.applyCollisionObjects(collision_objects);
        // planning_scene_interface_.applyCollisionObjects(remove_objects);
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cluster_extractor");
    
    ClusterExtractor extractor;
  
    ros::spin();
  
    return 0;
}
