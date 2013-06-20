/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>

#include <iostream>
#include <fstream>
#include <math.h>

#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/io/pcd_io.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
PointCloudT::Ptr final(new PointCloudT);
class point
{
  public:
  point();
  ~point();
  void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
  geometry_msgs::Vector3 rpyGet(const tf::Quaternion &qua);
  ros::NodeHandle nh_;

  private:
  int count;
  ros::Subscriber laser_scan_sub_;
  tf::TransformListener odom_tf_listener_;
  double cloud_x,cloud_y,cloud_z,new_x,new_y,new_z;
  double transform_roll,transform_pitch,transform_yaw;
  PointT point2_;
  ros::Publisher point_cloud2_pub_;//publish the PointCloud and PointCloud2 message
};
point::point()
{
  ROS_INFO("Starting ScanToCloudConverter");
  laser_scan_sub_ = nh_.subscribe("scan", 1000, &point::scanCallBack,this);
  point_cloud2_pub_ = nh_.advertise<PointCloudT> ("velodyne/pointcloud2", 1);
  count=0;
}
point::~point()
{
}

void point::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{

  PointCloudT::Ptr cloud_msg=boost::shared_ptr<PointCloudT>(new PointCloudT());

  cloud_msg->points.resize(scan_msg->ranges.size());

  for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
  {
    PointT& p = cloud_msg->points[i];
    float range = scan_msg->ranges[i];
    if (range > scan_msg->range_min && range < scan_msg->range_max)
    {
      float angle = scan_msg->angle_min + i*scan_msg->angle_increment;

      p.x = range * cos(angle);
      p.y = range * sin(angle);
      p.z = 0.0;
      cloud_x=-p.z;
      cloud_y=-p.x;
      cloud_z=-p.y;
      cloud_msg->width = scan_msg->ranges.size();
      cloud_msg->height = 1;
      cloud_msg->is_dense = false; //contains nans
      cloud_msg->header = scan_msg->header;
      tf::StampedTransform transform;
    try
    {
      odom_tf_listener_.lookupTransform("odom","base_link",  ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }

    transform_roll = rpyGet(transform.getRotation()).x;
    transform_pitch = rpyGet(transform.getRotation()).y;
    double a0=transform.getRotation().x();
    double a1=transform.getRotation().y();
    double a2=transform.getRotation().z();
    double a3=transform.getRotation().w();

    transform_yaw = rpyGet(transform.getRotation()).z;


    final->header.stamp = ros::Time::now();
    final->header.frame_id = "pch_frame";  
    new_x=cloud_x*(cos(transform_yaw)*cos(transform_pitch) + sin(transform_yaw)*sin(transform_roll)*sin(transform_pitch))+cloud_y*(cos(transform_yaw)*sin(transform_roll)*sin(transform_pitch)-sin(transform_yaw)*cos(transform_pitch))+cloud_z;
    new_y=cloud_x*(sin(transform_yaw)*cos(transform_roll))+cloud_y*(cos(transform_yaw)*cos(transform_roll));
    new_z=cloud_x*(sin(transform_yaw)*sin(transform_roll)*cos(transform_pitch)-cos(transform_yaw)*sin(transform_pitch))+cloud_y*(sin(transform_yaw)*sin(transform_pitch)+cos(transform_yaw)*sin(transform_roll)*cos(transform_pitch))+cloud_z;

    point2_.x=new_x;
    point2_.y=new_y;
    point2_.z=new_z;

    final->points.push_back(point2_); 

    }

  }

  if(count==500)
  {
    ROS_INFO("111");

    point_cloud2_pub_.publish(*final);
      final->points.clear();

    count=0;

  }

    count++;

}


int main(int argc,char **argv)
{
  ros::init(argc,argv,"cloud");
  point point1;

  ros::spin();
  return 0;
}

geometry_msgs::Vector3 point::rpyGet(const tf::Quaternion &qua)
{
  double x = qua.x();
  double y = qua.y();
  double z = qua.z();
  double w = qua.w();

  geometry_msgs::Vector3 vec;
  vec.x = atan2(2*(w*x + y*z),1 - 2*(x*x + y*y));//from -pai to pai
  vec.y = asin(2*(w*y - z*x));		//from -pai/2 to pai/2
  vec.z = atan2(2*(w*z + x*y),1 - 2*(y*y + z*z));//from -pai to pai

  return vec;
}



