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



#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/sample_consensus/sac_model_plane.h>


using namespace::std;

ros::Publisher pub, arr_pub, voxel_pub;

int j = 0;

int maxDistance = 3;

double get_distance(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg, int counter){
/*
*
* Function Name:  get_distance
* Input:    msg -> stores point cloud information to be processed
            counter -> maintains the frame in consideration, necessary to handle clusters of same frame. 
* Output:    Publishes the minimum distance to ROS publisher
* Logic:    Iterates through all points in the filtered point cloud and publishes the min distance,
*           its angles, its leftmost points distance and angles, its rightmost points distance and angles
* Example Call:  get_distance (cloud, j)
*
*/
  double rad_to_deg = 57.2958;
  rad_to_deg = 1;

  const double pi = boost::math::constants::pi<double>();

  double minDistance[4] = {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), 0.0};
// angles of rightmost point in the cluster
  double min_angle_radx[4] = {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), 0.0};

  double min_angle_rady[4] = {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), 0.0};
// angles of leftmost point in the cluster
  double max_angle_radx[4] = {0.0, -(std::numeric_limits<double>::infinity()), 0.0};

  double max_angle_rady[4] = {0.0, -(std::numeric_limits<double>::infinity()), 0.0};
  // int i = 0;
  // Angles are calculated in radians and can convert to degree by multpying it with 180/pi 
  BOOST_FOREACH (const pcl::PointXYZRGB& pt, msg->points){//to iterate trough all the points in the filtered point cloud published by publisher
    // std::cout<<i++<<endl;
    if(hypot(pt.z, pt.x) < minDistance[0]){
      // keep updating the minimum Distant point
      minDistance[0] = hypot(pt.z, pt.x);
      // minDistance[1] = atan2(pt.z,pt.x);
      minDistance[1] = (atan2(pt.x,pt.z))*rad_to_deg;
      minDistance[2] = (atan2(pt.y, pt.z))*rad_to_deg;
    }
    // Angles are not accurately measured. SO, subtract 0.25 m from each distance.
    if(((atan2(pt.x, pt.z))*rad_to_deg) < min_angle_radx[1]){
      // keep updating the minimum angle
      min_angle_radx[0] = hypot(pt.z, pt.x);
      min_angle_radx[1] = (atan2(pt.x,pt.z))*rad_to_deg;
      min_angle_radx[2] = (atan2(pt.y, pt.z))*rad_to_deg;
    }
    
    if(((atan2(pt.x, pt.z))*rad_to_deg) > max_angle_radx[1]){
      // keep updating the maximum angle
      max_angle_radx[0] = hypot(pt.z, pt.x);
      max_angle_radx[1] = (atan2(pt.x,pt.z))*rad_to_deg;
      max_angle_radx[2] = (atan2(pt.y, pt.z))*rad_to_deg;
    }
    if(((atan2(pt.y, pt.z))*rad_to_deg) < min_angle_radx[2]){
      // keep updating the maximum angle
      min_angle_rady[0] = hypot(pt.z, pt.x);
      min_angle_rady[1] = (atan2(pt.x,pt.z))*rad_to_deg;
      min_angle_rady[2] = (atan2(pt.y, pt.z))*rad_to_deg;
    }
    if(((atan2(pt.y, pt.z))*rad_to_deg) > max_angle_radx[2]){
      // keep updating the maximum angle
      max_angle_rady[0] = hypot(pt.z, pt.x);
      max_angle_rady[1] = (atan2(pt.x,pt.z))*rad_to_deg;
      max_angle_rady[2] = (atan2(pt.y, pt.z))*rad_to_deg;
    }
  }
  if (minDistance[0]<maxDistance)
  {
    std::cout<<"-------NEW CLUSTER------"<<std::endl;
    std::cout<<"MinDistance  "<<minDistance[0]<<"  ";
    std::cout<<"MinAngleX  "<<minDistance[1]<<"  ";
    std::cout<<"MinAngleY  "<<minDistance[2]<<std::endl;
    std::cout<<"LeftDistance  "<<min_angle_radx[0]<<"  ";
    std::cout<<"LeftAngleX  "<<min_angle_radx[1]<<"  ";
    std::cout<<"LeftAngleY  "<<min_angle_radx[2]<<std::endl;
    std::cout<<"RightDistance  "<<max_angle_radx[0]<<"  ";
    std::cout<<"RightAngleX  "<<max_angle_radx[1]<<"  ";
    std::cout<<"RightAngleY  "<<max_angle_radx[2]<<std::endl;
    std_msgs::Float64MultiArray arr;

    arr.data.clear();
    bool sign1 =  min_angle_radx[1] < 0;
    bool sign2 =  max_angle_radx[1] < 0;
    double width = 0.0;
    if ((sign1 && sign2) || (!sign1 && !sign2))
    {
      width = abs(min_angle_radx[0]*sin(abs(min_angle_radx[1])) - max_angle_radx[0]*sin(abs(max_angle_radx[1])));
    }else{
      width = abs(min_angle_radx[0]*sin(abs(min_angle_radx[1])) + max_angle_radx[0]*sin(abs(max_angle_radx[1])));
    }
    std::cout<<"Width  "<< width <<std::endl;

    sign1 =  min_angle_rady[2] < 0;
    sign2 =  max_angle_rady[2] < 0;
    double height = 0.0;
    if ((sign1 && sign2) || (!sign1 && !sign2))
    {
      height = abs(min_angle_rady[0]*sin(abs(min_angle_rady[2])) - max_angle_rady[0]*sin(abs(max_angle_rady[2])));
    }else{
      height = abs(min_angle_rady[0]*sin(abs(min_angle_rady[2])) + max_angle_rady[0]*sin(abs(max_angle_rady[2])));
    }
    // std::cout<<"Height  "<< height <<std::endl;

    arr.data.push_back(counter);
    arr.data.push_back(width);
    // distance of nearest point in cluster
    arr.data.push_back(minDistance[0]);
    // angle in xy plane of nearest point in cluster
    arr.data.push_back(minDistance[1]);
    arr.data.push_back(height);
    
    arr_pub.publish(arr);
  }
  return minDistance[0];
}



/*
*
* Function Name:  cloud_cb
* Input:    input -> A point cloud to work on 
* Output:   Publishes the clustered point cloud and the distances of them.
* Logic:    first a voxelgrid filter is applied to make the cloud less dense.
*           then Sac segmentation is done using ransac model to extract the planes
*           Then using extractIndices the point cloud without the floor plane is extracted.
*           then eucledian clustering is executed to find clusters
*           the various clusters are then concatinated and then published, this also removes outliers
*           for each cluster the teh get_distance() function is called which produces useful information
*           about each of the clusters.
* Example Call: Callback function. Manual calling not required 
*
*/
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	// Leaf size == Distance between 2 pts in the point cloud.
	// maxClusterSize == Max number of points that are allowed to cluster together.
	// minClusterSize == Min number of points that should be there to form a cluster

  int minClusterSize = 100, maxClusterSize = 200, maxIterations = 150;
  double leaf_size = 0.05, distanceThreshold = 0.01, clusterTolerance = 0.05;

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

  // pcl::SampleConsensusModelNormalPlane<pcl::PointXYZRGB, pcl::Normal> sac_model;

  pcl::PointCloud<pcl::Normal>::Ptr normals_out (new pcl::PointCloud<pcl::Normal>);
  pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
  // Specify method for normal estimation
  norm_est.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
  // Specify max depth change factor
  norm_est.setMaxDepthChangeFactor(0.02f);
  // Specify smoothing area size
  norm_est.setNormalSmoothingSize(10.0f);
  // Set the input points
  norm_est.setInputCloud (points);
  // Estimate the surface normals and
  // store the result in "normals_out"
  norm_est.compute (*normals_out);

  // Create a shared plane model pointer directly
  pcl::SampleConsensusModelNormalPlane<PointXYZRGB, pcl::Normal>::Ptr model (new SampleConsensusModelNormalPlane<PointXYZ, pcl::Normal> (*cloud_filtered));
  // Set normals
  model->setInputNormals(normals);
  // Set the normal angular distance weight.
  model->setNormalDistanceWeight(0.5f);
  // Create the RANSAC object
  RandomSampleConsensus<PointXYZRGB> sac (model, 0.03);
  // perform the segmenation step
  bool result = sac.computeModel ();


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
