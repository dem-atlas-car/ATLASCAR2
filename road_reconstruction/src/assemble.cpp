// /*********************************************************************
// * Software License Agreement (BSD License)
// *
// *  Copyright (c) 2008, Willow Garage, Inc.
// *  All rights reserved.
// *
// *  Redistribution and use in source and binary forms, with or without
// *  modification, are permitted provided that the following conditions
// *  are met:
// *
// *   * Redistributions of source code must retain the above copyright
// *     notice, this list of conditions and the following disclaimer.
// *   * Redistributions in binary form must reproduce the above
// *     copyright notice, this list of conditions and the following
// *     disclaimer in the documentation and/or other materials provided
// *     with the distribution.
// *   * Neither the name of the Willow Garage nor the names of its
// *     contributors may be used to endorse or promote products derived
// *     from this software without specific prior written permission.
// *
// *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// *  POSSIBILITY OF SUCH DAMAGE.
// *********************************************************************/

// #include <cstdio>
// #include <ros/ros.h>

// // Services
// #include "laser_assembler/AssembleScans.h"

// // Messages
// #include "sensor_msgs/PointCloud.h"

// /***
//  * This a simple test app that requests a point cloud from the
//  * point_cloud_assembler every 4 seconds, and then publishes the
//  * resulting data
//  */
// namespace laser_assembler
// {

// class PeriodicSnapshotter
// {

// public:

//   PeriodicSnapshotter()
//   {
//     // Create a publisher for the clouds that we assemble
//     pub_ = n_.advertise<sensor_msgs::PointCloud> ("assembled_cloud", 1);

//     // Create the service client for calling the assembler
//     client_ = n_.serviceClient<AssembleScans>("assemble_scans");

//     // Start the timer that will trigger the processing loop (timerCallback)
//     timer_ = n_.createTimer(ros::Duration(5,0), &PeriodicSnapshotter::timerCallback, this);

//     // Need to track if we've called the timerCallback at least once
//     first_time_ = true;
//   }

//   void timerCallback(const ros::TimerEvent& e)
//   {

//     // We don't want to build a cloud the first callback, since we we
//     //   don't have a start and end time yet
//     if (first_time_)
//     {
//       first_time_ = false;
//       return;
//     }

//     // Populate our service request based on our timer callback times
//     AssembleScans srv;
//     srv.request.begin = e.last_real;
//     srv.request.end   = e.current_real;

//     // Make the service call
//     if (client_.call(srv))
//     {
//       ROS_INFO("Published Cloud with %u points", (uint32_t)(srv.response.cloud.points.size())) ;
//       pub_.publish(srv.response.cloud);
//     }
//     else
//     {
//       ROS_ERROR("Error making service call\n") ;
//     }
//   }

// private:
//   ros::NodeHandle n_;
//   ros::Publisher pub_;
//   ros::ServiceClient client_;
//   ros::Timer timer_;
//   bool first_time_;
// } ;

// }

// using namespace laser_assembler ;

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "periodic_snapshotter");
//   ros::NodeHandle n;
//   ROS_INFO("Waiting for [build_cloud] to be advertised");
//   ros::service::waitForService("build_cloud");
//   ROS_INFO("Found build_cloud! Starting the snapshotter");
//   PeriodicSnapshotter snapshotter;
//   ros::spin();
//   return 0;
// }

//------------------------------------------------------------------

/*
 * cloud_assembler.cpp
 *
 *  Created on: Nov 11, 2011
 *      Author: dimitri prosser
 */

#include <laser_geometry/laser_geometry.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <boost/algorithm/string.hpp>

using namespace pcl;

namespace cloud_assembler
{
typedef PointCloud<PointXYZ> PointCloud2;

class CloudAssembler
{
public:
  CloudAssembler();
  void cloudCallback(const sensor_msgs::PointCloud2& cloud);

private:
  ros::NodeHandle node_;

  ros::ServiceServer pause_srv_;

  ros::Publisher output_pub_;
  ros::Subscriber cloud_sub_;

  tf::TransformListener tf_;

  PointCloud2 assembled_cloud_;
  PointCloud2 clean_cloud_;
  int buffer_length_;
  int dist;
  std::deque<sensor_msgs::PointCloud2> cloud_buffer_;
  bool assemblerPaused_;

  void addToBuffer(sensor_msgs::PointCloud2 cloud);
  void assembleCloud();
  void cleanCloud();
  // bool pauseSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
};

CloudAssembler::CloudAssembler()
{
  ros::NodeHandle private_nh("~");

  private_nh.param("buffer_length", buffer_length_, 400);

  output_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/assembled_cloud", 100);

  // pause_srv_ = node_.advertiseService("/pause_assembler", &CloudAssembler::pauseSrv, this);

  cloud_sub_ = node_.subscribe("/my_cloud_in", 100, &CloudAssembler::cloudCallback, this);

  // PointCloud2 clear;
  // assembled_cloud_ = clear;

  assemblerPaused_ = false;
}

void CloudAssembler::cloudCallback(const sensor_msgs::PointCloud2& cloud)
{
  addToBuffer(cloud);
  assembleCloud();
  cleanCloud();

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(clean_cloud_, cloud_msg);

  cloud_msg.header.frame_id = cloud.header.frame_id;
  cloud_msg.header.stamp = ros::Time::now();

  output_pub_.publish(cloud_msg);
}

void CloudAssembler::addToBuffer(sensor_msgs::PointCloud2 cloud)
{
  ROS_DEBUG("Adding cloud to buffer. Current buffer length is %ld", cloud_buffer_.size());

  if (cloud_buffer_.size() >= (unsigned int)buffer_length_)
  {
    cloud_buffer_.erase(cloud_buffer_.begin());
  }

  cloud_buffer_.push_back(cloud);
}

void CloudAssembler::assembleCloud()
{
  ROS_DEBUG("Assembling.");

  unsigned int i;

  // if (assemblerPaused_)
  // {
  //   ROS_INFO("assemblerPaused_ is true");
  // }
  // if (!assemblerPaused_)
  // {
  //   ROS_DEBUG("assemblerPaused_ is false");
  // }

  std::string fixed_frame = cloud_buffer_[0].header.frame_id;

  PointCloud2 new_cloud;
  new_cloud.header.frame_id = fixed_frame;
  // int last_elem = cloud_buffer_.size();
  // pcl_conversions::toPCL(cloud_buffer_[last_elem].header.stamp, new_cloud.header.stamp);
  pcl_conversions::toPCL(ros::Time::now(), new_cloud.header.stamp);
  // new_cloud.header.stamp = ros::Time::now();

  for (i = 0; i < cloud_buffer_.size(); i++)
  {
    PointCloud2 temp_cloud;
    pcl::fromROSMsg(cloud_buffer_[i], temp_cloud);
    temp_cloud.header.frame_id = fixed_frame;
    new_cloud += temp_cloud;
  }

  // If it's paused, don't overwrite the stored cloud with a new one, just keep publishing the same cloud
  // if (!assemblerPaused_)
  // {
  //   assembled_cloud_ = new_cloud;
  // }
  // else if (assemblerPaused_)
  // {
  //   ROS_DEBUG("The Assembler will continue to publish the same cloud.");
  // }
}

void CloudAssembler::cleanCloud()
{
  tf::StampedTransform transformOdom;
  tf::TransformListener listener;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_clean(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::fromROSMsg(assembled_cloud_, *cloud_to_clean);
  *cloud_to_clean = assembled_cloud_;
  try
  {
    listener.waitForTransform("map", "base_link_imu", ros::Time(0), ros::Duration(2.0));
    listener.lookupTransform("map", "base_link_imu", ros::Time(0), transformOdom);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  // Localização da origem do ref base_link_imu
  float Xo = transformOdom.getOrigin().x();
  float Yo = transformOdom.getOrigin().y();
  float Zo = transformOdom.getOrigin().z();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());

  // Condição para os limites da bounding box de representação da pointcloud
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, Xo - 50)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, Xo + 50)));

  // build the filter
  pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
  condrem.setCondition(range_cond);
  condrem.setInputCloud(cloud_to_clean);
  condrem.setKeepOrganized(true);
  // apply filter
  condrem.filter(*cloud_filtered);

  // Depois passar aqui um voxel filter para diminuir a densidade dos pontos
  // VoxelGrid<pcl::PointXYZ> vg;
  // PointCloud<pcl::PointXYZ>::Ptr cloud_filteredVox(new PointCloud<pcl::PointXYZ>);
  // vg.setInputCloud(cloud_filtered);
  // vg.setLeafSize(0.9f, 0.9f, 0.9f);
  // vg.filter(*cloud_filteredVox);
  // converter para mensagem para ser publicada

  clean_cloud_ = *cloud_filtered;
  // pcl::toROSMsg(*cloud_filtered, clean_cloud_);
}

/*
bool CloudAssembler::pauseSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
{
  ROS_DEBUG("In service call: %s", assemblerPaused_ ? "true" : "false");

  if (!assemblerPaused_)
  {
    ROS_DEBUG("Now paused.");
    assemblerPaused_ = true;
  }
  else if (assemblerPaused_)
  {
    assemblerPaused_ = false;
    ROS_DEBUG("Unpaused.");
  }

  return true;
}
*/

};  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_assembler");
  cloud_assembler::CloudAssembler cloud_assembler;

  ros::spin();

  return 0;
}
