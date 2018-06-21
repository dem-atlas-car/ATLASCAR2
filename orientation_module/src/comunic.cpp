
// -----Includes -----
#include <algorithm>
#include <iostream>
#include <vector>
// Boost includes
#include <boost/shared_ptr.hpp>

// Ros includes
#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>  //Messages on sreen

// Data types
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

using namespace ros;

using namespace std;

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

// apagar isto -------------------------
#include <laser_geometry/laser_geometry.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>

class Comunication
{
public:
  Comunication()
  {
    sub = nh.subscribe("DadosInclin", 100, &Comunication::printData, this);
    sub_imu = nh.subscribe("imu", 1, &Comunication::PubImuData, this);
    sub_simple = nh.subscribe("cloud_simple", 1, &Comunication::PrintOri, this);
  }

  void PrintOri(const sensor_msgs::PointCloud2 &msg)
  {
    //-----writing to file -------------
    // std::ofstream myfile;
    // myfile.open("/home/tiago/catkin_ws_path/src/result_ori.txt", std::ios::out | std::ios::app);
    // myfile << pitch << "\t" << roll << "\t" << z_mean << '\n';
    // myfile.close();
    //-----writing to file--------------
  }

  void PubImuData(const sensor_msgs::ImuPtr &imu)
  {
    imu->header.frame_id = "World";
    imu_pub.publish(imu);
  }

  void printData(const std_msgs::Float32MultiArray::ConstPtr &msg)
  {
    // Criar a transformação entre chassis e estrada
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    // Receber e converter para radianos (remover o 4.8)
    // float pitch = (msg->data[0] + 4.8) * 3.1415 / 180;
    pitch = msg->data[0] * 3.1415 / 180;
    roll = msg->data[1] * 3.1415 / 180;
    z_mean = msg->data[2] / 1000;

    // pitch = -3.1415 / 10;
    // ROS_INFO("P: %f, R: %f, Z_m: %f", msg->data[0] + 4.8, msg->data[1], z_mean);
    // aqui depois colocar zmean
    transform.setOrigin(tf::Vector3(1.175, 0, z_mean));
    transform.setRotation(tf::createQuaternionFromRPY(roll, pitch, 0));

    // publicar a tranformada entre o chassis do carro e a estrada
    br.sendTransform(tf::StampedTransform(transform, Time::now(), "base_link_imu", "car_center"));
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Subscriber sub_imu;
  ros::Subscriber sub_simple;
  ros::Publisher vis_pub;
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 1);
  float pitch, roll, z_mean;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Comunication");
  // ros::NodeHandle nh;
  Comunication comunic;

  while (ros::ok())
  {
    // comunic.loop_function();
    ros::spinOnce();
  }

  return 0;
}
