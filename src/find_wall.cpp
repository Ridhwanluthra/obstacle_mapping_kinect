/*
*
* Project Name:   Visual perception for the visually impaired
* Author List:    Pankaj Baranwal, Ridhwan Luthra, Shreyas Sachan, Shashwat Yashaswi
* Filename:     cluster_distances.cpp
* Functions:    get_distance, cloud_cb, main
* Global Variables: pub, arr_pub, j
*
*/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <stdio.h>
#include <limits>
#include <vector>
#include <math.h>
#include <iostream>


#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float64MultiArray.h"

using namespace::std;

ros::Publisher pub, arr_pub, voxel_pub;

int j = 0;

int maxDistance = 2;

// void detect_wall(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blob){
//   BOOST_FOREACH (const pcl::PointXYZRGB& pt, msg->points){//to iterate trough all the points in the filtered point cloud published by publisher
//     double maxDistance = 0.0;
//     pcl::PointXYZRGB& maxPoint;
//     if(hypot(pt.z, pt.x) > maxDistance){
//       maxDistance = hypot(pt.z, pt.x);
//       maxPoint = pt;
//     }
//   }
//   BOOST_FOREACH (const pcl::PointXYZRGB& pt, msg->points){//to iterate trough all the points in the filtered point cloud published by publisher
//     double maxDistance = 0.0;
//     pcl::PointXYZRGB& maxPoint;
//     if (hypot(pt.z, pt.x) > maxDistance){
//       maxDistance = hypot(pt.z, pt.x);
//       maxPoint = pt;
//     }
//   }

// }

void detect_wall(std::vector<pcl::PointIndices> cluster_indices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blob){
  pcl::PointXYZRGB& pt_clusters[cluster_indices.size()];
  for (int it = 0; it < cluster_indices.size(); ++it)
  {

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> ext;
    ext.setInputCloud (cloud_clust_remove);
    ext.setIndices(boost::shared_ptr<pcl::PointIndices> (new pcl::PointIndices(cluster_indices[it])));
    // to extract just the cluster
    ext.setNegative (false);
    ext.filter (*cloud_f);
    range = get_distance(cloud_f, j);
    // to ignore far away clusters for brevity.
    if (range >= maxDistance){
      continue;
    }
    pcl::PointCloud<pcl::PointXYZRGB> cloud_xyzrgb = *cloud_f;
    pt_clusters[it] = cloud_xyzrgb.points[0];
  }
  BOOST_FOREACH (const pcl::PointXYZRGB& pt, cloud_blob->points){

  }
}



void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	// Leaf size == Distance between 2 pts in the point cloud.
	// maxClusterSize == Max number of points that are allowed to cluster together.
	// minClusterSize == Min number of points that should be there to form a cluster

  int minClusterSize = 100, maxClusterSize = 300, maxIterations = 150;
  double leaf_size = 0.03, distanceThreshold = 0.01, clusterTolerance = 0.05;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PCLPointCloud2* cloud_blob = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_blob);
  pcl::PCLPointCloud2 cloud_filtered_blob;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_planar (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl_conversions::toPCL(*input, *cloud_blob);  
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (leaf_size, leaf_size, leaf_size);
  sor.filter (cloud_filtered_blob);

  voxel_pub.publish(cloud_filtered_blob);

  pcl::fromPCLPointCloud2 (cloud_filtered_blob, *cloud_filtered);

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (maxIterations);
  seg.setDistanceThreshold (distanceThreshold);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (clusterTolerance);
  ec.setMinClusterSize (minClusterSize);
  ec.setMaxClusterSize (maxClusterSize);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clust_remove (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2 cloud_final;
  pcl::PCLPointCloud2 temp;
  *cloud_clust_remove = *cloud_filtered;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_xyzrgb;
  double range;
  std::cout<<"-------NEW FRAME------"<<std::endl<<std::endl;
  for (int it = 0; it < cluster_indices.size(); ++it)
  {

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> ext;
    ext.setInputCloud (cloud_clust_remove);
    ext.setIndices(boost::shared_ptr<pcl::PointIndices> (new pcl::PointIndices(cluster_indices[it])));
    // to extract just the cluster
    ext.setNegative (false);
    ext.filter (*cloud_f);
    range = get_distance(cloud_f, j);
    // to ignore far away clusters for brevity.
    if (range >= maxDistance){
      continue;
    }
    cloud_xyzrgb = *cloud_f;

    // iteratively colors the cluster red, green or blue.
    for (size_t i = 0; i < cloud_xyzrgb.points.size(); i++) {
      if (it % 3 == 0) {
        cloud_xyzrgb.points[i].r = 255;
      }
      else if (it % 3 == 1) {
        cloud_xyzrgb.points[i].g = 255;
      }
      else if (it % 3 == 2) {
        cloud_xyzrgb.points[i].b = 255;  
      }
    }

    pcl::toPCLPointCloud2 (cloud_xyzrgb, temp);
    pcl::concatenatePointCloud(cloud_final, temp, cloud_final);
  }
  j++;
  pub.publish (cloud_final);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cluster_dist");
  ros::NodeHandle nh;


  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  voxel_pub = nh.advertise<sensor_msgs::PointCloud2> ("voxeled", 1);

  // publishing  details
  arr_pub = nh.advertise<std_msgs::Float64MultiArray> ("cluster_distances", 10);
  // pub = 

  // Spin
  ros::spin ();
}