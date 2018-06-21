/**************************************************************************************************
 Software License Agreement (BSD License)

 Copyright (c) 2014-2015, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:

  *Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************************/
/**
\file  device_frame_publisher.h
\brief Global include file
\author Diogo Correia
\date   June, 2017
*/

#ifndef _FRAME_PUBLISHER_H_
#define _FRAME_PUBLISHER_H_

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <sys/stat.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
/*---PointCould Includes---*/
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl_conversions/pcl_conversions.h>
/*---LAR TK4 Includes---*/
#include <colormap/colormap.h>
#include "lidar_segmentation/clustering.h"
#include "lidar_segmentation/lidar_segmentation.h"
#include "lidar_segmentation/visualization_rviz.h"
/*---Boost filesystem to get parent directory---*/
#include <boost/bind.hpp>
#include "boost/filesystem.hpp"
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/progress.hpp"

/*---DEFINES---*/
typedef geometry_msgs::PolygonStamped polygonS;
typedef boost::shared_ptr<polygonS> polygonSPtr;

typedef sensor_msgs::PointCloud2::Ptr pclPtr;
typedef sensor_msgs::PointCloud2 PCL;

using namespace ros;
using namespace std;

/*---Prototipos---*/
int getFilesInDir(const string filesPath, vector<string> &outfiles, vector<string> &filesName);
Eigen::Matrix4f getTransformFromFile(string filePath);
tf::Transform getTfTransform(Eigen::MatrixX4f trans);
tf::Transform getTf(double x, double y, double z, double r, double p, double yy);
void readCalibrationFiles(string filesPath, vector<tf::Transform> &deviceFrames, vector<string> &deviceNames);
void drawMarker(ros::Publisher chatter_pub, tf::StampedTransform transform_marker);

#endif
