// ROS core
#include <ros/ros.h>
//Image message
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
//pcl::toROSMsg
#include <pcl/io/pcd_io.h>
//stl stuff
#include <string>


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

// class PointCloudToImage
// {
// public:
  
//   PointCloudToImage () : cloud_topic_("input"),image_topic_("output")
//   {
//     sub_ = nh_.subscribe (cloud_topic_, 30,
//                           &PointCloudToImage::cloud_cb, this);
//     image_pub_ = nh_.advertise<sensor_msgs::Image> (image_topic_, 30);

//     //print some info about the node
//     std::string r_ct = nh_.resolveName (cloud_topic_);
//     std::string r_it = nh_.resolveName (image_topic_);
//     ROS_INFO_STREAM("Listening for incoming data on topic " << r_ct );
//     ROS_INFO_STREAM("Publishing image on topic " << r_it );
//   }
// private:
//   ros::NodeHandle nh_;
//   sensor_msgs::Image image_; //cache the image message
//   std::string cloud_topic_; //default input
//   std::string image_topic_; //default output
//   ros::Subscriber sub_; //cloud subscriber
//   ros::Publisher image_pub_; //image message publisher
// };

ros::Publisher image_pub_; //image message publisher

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  sensor_msgs::Image image_;

  if ((input->width * input->height) == 0)
    return; //return if the cloud is not dense!
  try
  {
    pcl::toROSMsg (*input, image_); //convert the cloud
  }
  catch (std::runtime_error e)
  {
    ROS_ERROR_STREAM("Error in converting cloud to image message: "
                      << e.what());
  }
  image_pub_.publish (image_); //publish our cloud image
}

int
main (int argc, char **argv)
{
  // Initialize ROS
  ros::init (argc, argv, "convert_pointcloud_to_image");
  ros::NodeHandle nh;


  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  image_pub_ = nh.advertise<sensor_msgs::Image> ("image_out", 30);

  // Spin
  ros::spin ();
}
