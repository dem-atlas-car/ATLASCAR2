/**************************************************************************************************
   Software License Agreement (BSD License)

   Copyright (c) 2014-2015, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
   All rights reserved.

   Redistribution and use in source and binary forms, with or without modification, are permitted
   provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
 * Neither the name of the University of Aveiro nor the names of its contributors may be used to
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
   \file  device_frame_publisher.cpp
   \brief Algorithm for reading calibrations files and publish the corresponding frames
   \author Diogo Correia
   \date   June, 2017
 */

#include <tf/transform_listener.h>
#include "free_space_detection/device_frame_publisher.h"

/**
@brief Reads the contenst of a folder and lists the names of all the files present
@param[in] Path with the location of the folder
@param[out] Array with the path of the all the files inside the folder
@param[out] Array with the name of the all the files inside the folder
@return int Return 1 if error 0 if sussecefull
*/
int getFilesInDir(const string filesPath, vector<string> &outfiles, vector<string> &filesName)
{
  boost::filesystem::path full_path(boost::filesystem::initial_path<boost::filesystem::path>());

  full_path = boost::filesystem::system_complete(boost::filesystem::path(filesPath));

  unsigned long file_count = 0;
  unsigned long dir_count = 0;
  unsigned long other_count = 0;
  unsigned long err_count = 0;

  if (!boost::filesystem::exists(full_path))
  {
    std::cout << "\nNot found: " << filesPath << std::endl;
    return 1;
  }

  if (boost::filesystem::is_directory(full_path))
  {
    boost::filesystem::directory_iterator end_iter;
    for (boost::filesystem::directory_iterator dir_itr(full_path); dir_itr != end_iter; ++dir_itr)
    {
      try
      {
        if (boost::filesystem::is_directory(dir_itr->status()))
        {
          ++dir_count;
        }
        else if (boost::filesystem::is_regular_file(dir_itr->status()))
        {
          ++file_count;
          string file_name = dir_itr->path().filename().string();
          int name_len = file_name.size();
          string file_type = file_name.substr(name_len - 4, 10);
          if (strcmp(file_type.c_str(), ".txt") == 0)
          {
            outfiles.push_back(filesPath + "/" + file_name);
            filesName.push_back(file_name.substr(0, name_len - 4));
          }
        }
        else
        {
          ++other_count;
        }
      }
      catch (const std::exception &ex)
      {
        ++err_count;
        std::cout << dir_itr->path().filename() << " " << ex.what() << std::endl;
      }
    }
  }
  else  // must be a file
  {
    std::cout << "\nFound: " << full_path.filename() << "\n";
  }

  return 0;
}

/**
@brief Gets a rigid body transformation from a file
@param[in] Path for the file with containing the transformation matrix
@return Eigen::Matrix4f Returns a rigid body transformation matrix
*/
Eigen::Matrix4f getTransformFromFile(string filePath)
{
  int nrows = 4;
  int ncols = 4;
  Eigen::MatrixX4f transform(nrows, ncols);

  const char *FilePath = filePath.c_str();
  ifstream fin(FilePath);

  if (fin.is_open())
  {
    for (int row = 0; row < nrows; row++)
      for (int col = 0; col < ncols; col++)
      {
        float item = 0.0;
        fin >> item;
        transform(row, col) = item;
      }
    fin.close();
  }
  /*--- DEBUG ---
    cout << transform(0,0) << " " << transform(0,1) << " " << transform(0,2) << " " << transform(0,3) << endl;
    cout << transform(1,0) << " " << transform(1,1) << " " << transform(1,2) << " " << transform(1,3) << endl;
    cout << transform(2,0) << " " << transform(2,1) << " " << transform(2,2) << " " << transform(2,3) << endl;
    cout << transform(3,0) << " " << transform(3,1) << " " << transform(3,2) << " " << transform(3,3) << endl;
    --- DEBUG ---*/
  return transform;
}

/**
@brief Converts a rigid body matrix (Eigen::MatrixX4f) to a tf::Transform
@param[in] Matrix with the rigid body transformation
@return tf::Transform Returns tf::Transform of a rigid body transformation
*/
tf::Transform getTfTransform(Eigen::MatrixX4f trans)
{
  tf::Transform t1;

  t1.setOrigin(tf::Vector3(trans(0, 3), trans(1, 3), trans(2, 3)));

  tf::Quaternion q;
  tf::Matrix3x3 ori;
  ori.setValue(trans(0, 0), trans(0, 1), trans(0, 2), trans(1, 0), trans(1, 1), trans(1, 2), trans(2, 0), trans(2, 1),
               trans(2, 2));
  ori.getRotation(q);

  t1.setRotation(q);

  return t1;
}

/**
@brief Converts an angle from degrees to radians
@param[in] Angle in degrees
@return double Returns an angle in radians
*/
double degToRad(double deg)
{
  double rad = deg * M_PI / 180;
  return rad;
}

/**
@brief Creates a tf::Transform from information about translation and rotation
@param[in] Translation allong the x coordinate
@param[in] Translation allong the y coordinate
@param[in] Translation allong the z coordinate
@param[in] Rotation allong the x axis
@param[in] Rotation allong the y axis
@param[in] Rotation allong the z axis
@return tf::Transform Returns tf::Transform of a rigid body transformation
*/
tf::Transform getTf(double x, double y, double z, double r, double p, double yy)
{
  tf::Transform t1;
  t1.setOrigin(tf::Vector3(x, y, z));
  tf::Quaternion q;
  q.setRPY(degToRad(r), degToRad(p), degToRad(yy));
  t1.setRotation(q);

  return t1;
}

/**
@brief Reads the calibration files and returns a tranformation for each file
@param[in] Path for the folder location
@param[in] Array to hold the transformation
@param[in] Array to hold the names of the respetive calibration files
@return void
*/
void readCalibrationFiles(string filesPath, vector<tf::Transform> &deviceFrames, vector<string> &deviceNames)
{
  vector<string> files;
  getFilesInDir(filesPath, files, deviceNames);

  tf::Transform ld_tf;
  bool ld_push = false;
  tf::Transform transform;

  for (int i = 0; i < files.size(); i++)
  {
    transform = getTfTransform(getTransformFromFile(files[i]));

    if (deviceNames[i] == "ldmrs")
    {
      deviceNames[i] = "ldmrs3";
      deviceFrames.push_back(transform * getTf(0, 0, 0, 0, -1.6, 0));
      ld_tf = transform;
      ld_push = true;
    }
    else
    {
      deviceFrames.push_back(transform);
    }
  }

  if (ld_push)
  {
    deviceNames.push_back("ldmrs2");
    deviceNames.push_back("ldmrs1");
    deviceNames.push_back("ldmrs0");

    deviceFrames.push_back(ld_tf * getTf(0, 0, 0, 0, -0.8, 0));
    deviceFrames.push_back(ld_tf * getTf(0, 0, 0, 0, 0.8, 0));
    deviceFrames.push_back(ld_tf * getTf(0, 0, 0, 0, 1.6, 0));
    // deviceFrames.push_back(ld_tf);
  }
}

/**
   @brief Main function to publish the reference frames for each device
   @param argc
   @param argv
   @return int
 */
void drawMarker(ros::Publisher chatter_pub, tf::StampedTransform transform_marker)
{
  tf::Vector3 origin;
  tf::Quaternion orient;
  origin = transform_marker.getOrigin();
  orient = transform_marker.getRotation();

  // Inicialização
  visualization_msgs::Marker marker;
  marker.header.frame_id = "moving_axis";
  marker.header.stamp = ros::Time::now();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;

  // Marcador
  // marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = origin.x();
  marker.pose.position.y = origin.y();
  marker.pose.position.z = origin.z();
  marker.pose.orientation.y = 0;  // orient.y();
  marker.pose.orientation.x = 1;  // orient.x();
  marker.pose.orientation.z = 0;  // orient.z();
  marker.pose.orientation.w = 1;  // orient.w();
  marker.scale.x = 15;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1;  // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  // ROS_INFO("Here!!!!!!!!!!!!!1");
  chatter_pub.publish(marker);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "device_frame_publisher");
  ros::NodeHandle nh;
  tf::TransformBroadcaster br;

  string calibFilesP;
  if (!nh.getParam("calibFilesPath", calibFilesP))
  {
    calibFilesP = ros::package::getPath("free_space_detection") + "/calibration_data";
  }
  ROS_INFO("Calibration Files Path: %s", calibFilesP.c_str());

  vector<tf::Transform> deviceFrames;
  vector<string> deviceNames;
  readCalibrationFiles(calibFilesP.c_str(), deviceFrames, deviceNames);

  tf::Transform LD_tf = deviceFrames[deviceFrames.size() - 1] * getTf(0, 0, 0, 0, -1.6, 0);

  //  deviceFrames.push_back(getTf(0, 0, 0.5, 0, 0, 0));
  //  deviceNames.push_back("velodyne");

  string ref_sensor = "lms151_E";
  if (!nh.getParam("ref_sensor", ref_sensor))
  {
    ROS_WARN("Param 'ref_sensor' not found!");
  }
  ROS_INFO("Ref sensor: %s", ref_sensor.c_str());

  tf::Transform transform_acerto;
  tf::Transform transform_final;
  tf::StampedTransform transform_novo;
  tf::TransformListener listener_novo;

  tf::TransformListener listener_marker;
  tf::StampedTransform transform_marker;
  // ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher chatter_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);

  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    // try
    // {
    // transformação entre o referencial mundo e o centro do carro
    // listener_novo.lookupTransform("car_center", "base_link_imu", ros::Time(0), transform_novo);
    // transform_acerto = transform_novo;

    // Vetor entre a origem do mundo e a origem do ref do centro do carro com a mesma rotação
    // transform_final.setOrigin(transform_novo.getOrigin());
    transform_final.setOrigin(tf::Vector3(1.772, 0, -0.168));

    transform_final.setRotation(tf::Quaternion(0, 0, 0, 1));

    // ROS_INFO("Moving axis position in car_center [%f, %f, %f]", transform_novo.getOrigin().x(),
    //          transform_novo.getOrigin().y(), transform_novo.getOrigin().z());
    // Criar um referencial novo que irá servir como referencial base de todos os sensores
    br.sendTransform(tf::StampedTransform(transform_final, ros::Time::now(), "car_center", "moving_axis"));
    // }
    // catch (tf::TransformException &ex)
    // {
    //   ROS_ERROR("%s", ex.what());
    //   ros::Duration(1.0).sleep();
    //   continue;
    // }

    // Associar ao sensor referência o referencial base
    br.sendTransform(tf::StampedTransform(LD_tf.inverse(), ros::Time::now(), "moving_axis", ref_sensor));

    // Criar os referenciais dos respetivos sensores baseado nas informações dos ficheiros de calibração
    for (int i = 0; i < deviceNames.size(); i++)
    {
      tf::Transform T = deviceFrames[i];
      string name = deviceNames[i];
      // if (name == "velodyne")
      // {
      //   br.sendTransform(tf::StampedTransform(T, ros::Time::now(), name, "map"));
      // }
      // else
      // {
      br.sendTransform(tf::StampedTransform(T, ros::Time::now(), ref_sensor, name));
      // }
    }

    try
    {
      // Trasformação que dá o centro do referencial ldmrs3 (TA MAL)
      listener_marker.lookupTransform("lms151_E", "ldmrs3", ros::Time(0), transform_marker);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    // Desenhar a seta de marcador
    // drawMarker(chatter_pub, transform_marker);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
