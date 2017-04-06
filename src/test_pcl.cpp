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

void detect_wall(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blob){
  cout<< "NEW FRAME " << endl;
  int i = 0;
  cout << "WIDTH: " << cloud_blob->width << endl;
  cout << "HEIGHT: " << cloud_blob->height << endl;
  BOOST_FOREACH (const pcl::PointXYZRGB& pt, cloud_blob->points){//to iterate trough all the points in the filtered point cloud published by publisher
    cout << "x= " << pt.x << " y= " << pt.y << " z= " << pt.z << endl;
    i++;
    if (i>100)
      break;
  }
}


void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PCLPointCloud2* cloud_blob = new pcl::PCLPointCloud2;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl_conversions::toPCL(*input, *cloud_blob);
  pcl::fromPCLPointCloud2 (*cloud_blob, *cloud);
  std::vector<int> mapping;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);
  detect_wall(cloud);
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
