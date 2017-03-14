/*
*
* Project Name:   Visual perception for the visually impaired
* Author List:    
* Filename:     remove_floor.cpp
* Functions:    callback, main 
* Global Variables: pub -> Ros publisher
*
*/
#include <ros/ros.h>
// PCL specific includes voxel
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// PCL specific includes planar segmentation
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher pub;
/*
*
* Function Name: callback
* Input: input -> A ros message service that provides point cloud data from kinect
* Output:  Publishes the point cloud after removing floor.
* Logic:   
* Example Call: Callback function. Manual calling not required. 
*
*/
void callback (const sensor_msgs::PointCloud2ConstPtr& input) {

  pcl::PCLPointCloud2* cloud_blob = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_blob);
  pcl::PCLPointCloud2 cloud_filtered_blob;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl_conversions::toPCL(*input, *cloud_blob);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (cloud_filtered_blob);

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (cloud_filtered_blob, *cloud_filtered);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (cloud_filtered);
    
  seg.segment(*inliers, *coefficients);

  // Exit if no plane found
  if (inliers->indices.size () == 0) return;

  // Extract points of found plane
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  // set setNegative to true to get outliers
  extract.setNegative(true);
  extract.filter(*cloud_f);
  pcl::PointCloud<pcl::PointXYZRGB> cloud_xyzrgb = *cloud_f;

  for (size_t i = 0; i < cloud_xyzrgb.points.size(); i++) {
    cloud_xyzrgb.points[i].r = 255;
  }

  // Publish the plane to a new topic.
  pcl::PCLPointCloud2 outcloud;
  pcl::toPCLPointCloud2 (cloud_xyzrgb, outcloud);
  pub.publish (outcloud);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "remove_floor");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, callback);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("plane_removed", 1);

  // Spin
  ros::spin();
}