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
#include <pcl/io/png_io.h>
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
int st = 0;

void detect_wall(std::vector<pcl::PointIndices> cluster_indices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered, pcl::PCLPointCloud2 cloud_blob){
  int rad_to_deg = 1;
  int thresholdDistance = 2;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blob_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud_blob_pcl);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clust_remove (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2 cloud_final;
  pcl::PCLPointCloud2 temp;
  *cloud_clust_remove = *cloud_filtered;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_xyzrgb;
  double range;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
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
    cloud_xyzrgb = *cloud_f;
    pcl::PointXYZRGB leftmost_point[cluster_indices.size()];
    pcl::PointXYZRGB rightmost_point[cluster_indices.size()];
    pcl::PointXYZRGB topmost_point[cluster_indices.size()];
    pcl::PointXYZRGB bottommost_point[cluster_indices.size()];
    double minDistance[cluster_indices.size()];
    for (int i = 0; i < cluster_indices.size(); ++i)
    {
      leftmost_point[i] = cloud_xyzrgb.points[1];
    }
    for (int it = 0; it < cluster_indices.size(); ++it)
    {
      // iteratively colors the cluster red, green or blue.
      BOOST_FOREACH (const pcl::PointXYZRGB& pt, cloud_xyzrgb.points){
        // minimum distance
        if (hypot(pt.z, pt.x) < minDistance[it]){
          minDistance[it] = hypot(pt.z, pt.x);
        }
        // for (size_t i = 0; i < cloud_xyzrgb.points.size(); i++) {
        if (((atan2(pt.x, pt.z))*rad_to_deg) < ((atan2(leftmost_point[it].x, leftmost_point[it].z))*rad_to_deg)){
          leftmost_point[it] = pt;
        }
        // Check if point is rightmost
        if (((atan2(pt.x, pt.z))*rad_to_deg) > ((atan2(leftmost_point[it].x, leftmost_point[it].z))*rad_to_deg)){
          rightmost_point[it] = pt;
        }
        // Check if point is topmost
        if(((atan2(pt.y, pt.z))*rad_to_deg) < ((atan2(topmost_point[it].y, topmost_point[it].z))*rad_to_deg)){
          topmost_point[it] = pt;
        }
        // Check if point is bottommost
        if(((atan2(pt.y, pt.z))*rad_to_deg) > ((atan2(topmost_point[it].y, topmost_point[it].z))*rad_to_deg)){
          bottommost_point[it] = pt;
        }
      }
    }
  }

    pcl::PointCloud<pcl::PointXYZRGB> cloud_final[cluster_indices.size()];

    BOOST_FOREACH (const pcl::PointXYZRGB& pt, cloud_blob_pcl->points){
      for (int i = 0; i < cluster_indices.size(); i++)
      {
        // to ignore far away clusters for brevity.
        if (minDistance[i] > thresholdDistance)
        {
          cout << i << " FOUND" << endl;
          continue;
        }
        cout << i << " CHECKING" << endl;

        cout <<  ((atan2(pt.x, pt.z))*rad_to_deg) << "  " << ((atan2(leftmost_point[i].x, leftmost_point[i].z))*rad_to_deg)
        << "  " << ((atan2(rightmost_point[i].x, rightmost_point[i].z))*rad_to_deg) << endl;
        cout << ((atan2(pt.y, pt.z))*rad_to_deg) << "  " << ((atan2(topmost_point[i].y, topmost_point[i].z))*rad_to_deg)
        << "  " << ((atan2(bottommost_point[i].y, bottommost_point[i].z))*rad_to_deg) << endl;
        if (((atan2(pt.x, pt.z))*rad_to_deg) > ((atan2(leftmost_point[i].x, leftmost_point[i].z))*rad_to_deg) && 
          ((atan2(pt.x, pt.z))*rad_to_deg) < ((atan2(rightmost_point[i].x, rightmost_point[i].z))*rad_to_deg) &&
          ((atan2(pt.y, pt.z))*rad_to_deg) < ((atan2(topmost_point[i].y, topmost_point[i].z))*rad_to_deg) && 
          ((atan2(pt.y, pt.z))*rad_to_deg) > ((atan2(bottommost_point[i].y, bottommost_point[i].z))*rad_to_deg))
        {
          cout << i << " ADDED" << endl;
          cloud_final[i].push_back(pt);
          break;
        }
      }
    }
    pcl::PCLPointCloud2 temp;
    pcl::toPCLPointCloud2 (cloud_final[0], temp);
    pub.publish (temp);
}

void convertToImage(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cb){
  if (st==0)
  {
    pcl::io::savePNGFile("test_file.jpg", *cloud_cb, "rgb");
  }
  st++;
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
  detect_wall(cluster_indices, cloud_filtered, *cloud_blob);

  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clust_remove (new pcl::PointCloud<pcl::PointXYZRGB>);
  // pcl::PCLPointCloud2 cloud_final;
  // pcl::PCLPointCloud2 temp;
  // *cloud_clust_remove = *cloud_filtered;
  // pcl::PointCloud<pcl::PointXYZRGB> cloud_xyzrgb;
  // // double range;
  // std::cout<<"-------NEW FRAME------"<<std::endl<<std::endl;
  // for (int it = 0; it < cluster_indices.size(); ++it)
  // {
  //   // Extract the planar inliers from the input cloud
  //   pcl::ExtractIndices<pcl::PointXYZRGB> ext;
  //   ext.setInputCloud (cloud_clust_remove);
  //   ext.setIndices(boost::shared_ptr<pcl::PointIndices> (new pcl::PointIndices(cluster_indices[it])));
  //   // to extract just the cluster
  //   ext.setNegative (false);
  //   ext.filter (*cloud_f);
  //   // range = get_distance(cloud_f, j);
  //   // // to ignore far away clusters for brevity.
  //   // if (range >= maxDistance){
  //   //   continue;
  //   // }
  //   cloud_xyzrgb = *cloud_f;

  //   // iteratively colors the cluster red, green or blue.
  //   for (size_t i = 0; i < cloud_xyzrgb.points.size(); i++) {
  //     if (it % 3 == 0) {
  //       cloud_xyzrgb.points[i].r = 255;
  //     }
  //     else if (it % 3 == 1) {
  //       cloud_xyzrgb.points[i].g = 255;
  //     }
  //     else if (it % 3 == 2) {
  //       cloud_xyzrgb.points[i].b = 255;  
  //     }
  //   }

  //   pcl::toPCLPointCloud2 (cloud_xyzrgb, temp);
  //   pcl::concatenatePointCloud(cloud_final, temp, cloud_final);
  // }
  j++;
  // pub.publish (cloud_final);
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
