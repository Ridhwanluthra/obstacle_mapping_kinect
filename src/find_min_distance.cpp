/*
*author: Abhi alias Faize. 
*email: info@aiinspired.com
******* abhijith.anjana@gmail.com
*/
/* Explanation
* Have a better understanding about the Coordinate Frame Conventions for camera :http://www.ros.org/reps/rep-0103.html
* Now consider a point inside the point cloud and imagaine that point is formed on a XY plane where the perpendicular 
* distance from the plane to the camera is Z. 
* The perpendicular drawn from the camera to the plane hits at center of the XY plane 
* Also now we have the x and y coordinate of the point which is formed on the XY plane.
* Now X is the horizontal axis and Y is the vertical axis
*                             
*
*
*
*
*                                  X
*                    _____________________________  
*                   |                             |
*                   |                     *______________Z______________________
*                   |                             |  
*                   |             C|______________________Z_____________________|
*                  Y|              |              |                             | 
*                   |                             | 
*                   |                             | 
*                   |_____________________________|  
*
*
* C is the center of the plane which is Z meter away from the center of camera and * is the point on the plane
* Now we know Z is the perpendicular distance from the point to the camera. 
* If you need to find the  actual distance d from the point to the camera, you shloud calculate the hypotenuse hypot(pt.z, pt.x)
* Angle convention
* to the left of the camera is 0 degree and to the right is 180 degree. Like you stretch both the hands towards sides (deals with X axis). left 0-----|-----180 right (could be other way around, dont have sensor to test)
* to the  bottom of the camera is 0 degree and to the top is 108 degree. 
* angle the point made horizontally min_angle_radx=atan2(pt.z,pt.x);
* angle the point made Verticlly    min_angle_rady=atan2(pt.z, pt.y);
*/

/*
*
* Project Name:   Visual perception for the visually impaired
* Author List:     
* Filename:     find_min_distance.cpp
* Functions:    callback, main 
* Global Variables: N.A.
*
*/

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <stdio.h>

using namespace::std;


/*
*
* function name: callback
* input:  msg -> a point cloud data to be worked on 
* output: finds the minimum distance in the point cloud  
* logic:   
* example call: callback function. manual calling not required.
*
*/
void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg){
  double minDistance = std::numeric_limits<double>::infinity();;
  double min_angle_radx=0.0;
  double min_angle_rady=0.0;
  double xX=0.0,yY=0.0,zZ=0.0;
  int count=0;
  // int i = 0;
  // Angles are calculated in radians and can convert to degree by multpying it with 180/pi 
  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){//to iterate trough all the points in the filtered point cloud published by publisher
    // std::cout<<i++<<endl;
    if(hypot(pt.z, pt.x) < minDistance){
      // keep updating the minimum Distant point
      minDistance=hypot(pt.z, pt.x);
      min_angle_radx=atan2(pt.z,pt.x);
      min_angle_rady=atan2(pt.z, pt.y);
      xX=pt.x;
      yY=pt.y;
      zZ=pt.z;
    }
  }
 cout<<"Distance="<<minDistance<<"\n";
 cout<<"Angle in Degree X axis="<<min_angle_radx*(180/3.14159265358979323846)<<"\n";
 cout<<"Angle in Degree Y axis="<<min_angle_rady*(180/3.14159265358979323846)<<"\n";
 cout<<"pointXcoordinate="<<xX<<"\n";
 cout<<"pointYcoordinate="<<yY<<"\n";
 cout<<"pointZcoordinate="<<zZ<<"\n";
 // sleep(3); //use sleep if you want to delay loop.
}


int main(int argc, char** argv)
{
  ros::init(argc, argv,"find_min_distance");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >("input", 1, callback);
  ros::spin();
}
