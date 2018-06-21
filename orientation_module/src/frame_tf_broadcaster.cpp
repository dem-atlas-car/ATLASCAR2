
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <gps_common/GPSFix.h>
#include <nav_msgs/Odometry.h>

#include <novatel_gps_msgs/Inspva.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <swri_math_util/constants.h>
#include <swri_math_util/trig_util.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <sstream>
// Data types
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

class GroundPosition
{
public:
  GroundPosition();
  void getPose(const nav_msgs::OdometryPtr &msg);
  void HandleImu(const novatel_gps_msgs::InspvaPtr &imu_inspva);

private:
  ros::NodeHandle n;
  ros::Subscriber sub_odom;
  ros::Subscriber sub_direction;
  tf::TransformBroadcaster br;
  tf::Transform transform;

  geometry_msgs::Pose pose_in_world;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("Path_marker", 10);
  double yaw;
  double yaw_1;
  std::vector<geometry_msgs::Point> pontos;
  void TracePath(geometry_msgs::Point Coord);
  double val_init_x;
  double val_init_y;
};

GroundPosition::GroundPosition()
{
  sub_odom = n.subscribe("odom", 100, &GroundPosition::getPose, this);
  sub_direction = n.subscribe("inspva", 100, &GroundPosition::HandleImu, this);
}

void GroundPosition::HandleImu(const novatel_gps_msgs::InspvaPtr &imu_inspva)
{
  yaw = (90.0 - imu_inspva->azimuth) * swri_math_util::_deg_2_rad;

  yaw_1 = swri_math_util::WrapRadians(yaw, swri_math_util::_pi);
}

bool dd = 0;
void GroundPosition::getPose(const nav_msgs::OdometryPtr &msg)
{
  geometry_msgs::Point Coord;
  geometry_msgs::Quaternion Rot;
  // velocidade em m/s
  pose_in_world = msg->pose.pose;

  Coord = pose_in_world.position;
  Rot = pose_in_world.orientation;

  // Valor da primeira medicao
  if (dd != 1)
  {
    val_init_x = Coord.x;
    val_init_y = Coord.y;
    dd = 1;
  }
  // ROS_INFO("Val_init x,y: %f, %f", val_init_x, val_init_y);
  // ROS_INFO("X: %f, Y: %f", Coord.x, Coord.y);

  // O RVIZ nao gosta de numeros grandes por isso é preciso corrigir com um offset em relacao ao valor da primeira
  // medicao
  Coord.x = Coord.x - val_init_x;
  Coord.y = Coord.y - val_init_y;

  // transformação
  // transform.setOrigin(tf::Vector3(Coord.x, Coord.y, -0.28));
  transform.setOrigin(tf::Vector3(Coord.x, Coord.y, 0));

  // ROS_INFO("YAW: %f", yaw_1 * 180 / 3.1415);
  transform.setRotation(tf::createQuaternionFromRPY(0, 0, yaw_1));

  // publicar tensformação entre o referencial do mundo e o ground
  br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "ground"));

  // ground2map();
  TracePath(Coord);
}

void GroundPosition::TracePath(geometry_msgs::Point Coord)
{
  // float x = Coord.x;
  // float y = Coord.y;
  Coord.x = Coord.x;
  Coord.y = Coord.y;
  Coord.z = 0;
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "map";
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns = "car_path";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 1;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.1;
  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  if (pontos.size() < 500)
  {
    pontos.insert(pontos.begin(), Coord);
  }
  else
  {
    pontos.pop_back();
    pontos.insert(pontos.begin(), Coord);
  }

  line_strip.points = pontos;
  marker_pub.publish(line_strip);
}

/*
void GroundPosition::ground2map()
{
  // Antigo
  // transform_ekf.setOrigin(tf::Vector3(-(0.5 + 2.550 / 2), 0, -0.28));
  // transform_ekf.setOrigin(tf::Vector3(dist_tot / 10, 0, -0.28));
  // transform_ekf.setRotation(tf::createQuaternionFromRPY(0, 0, yaw));
  // transform.setRotation(tf::Quaternion(0, 0, 0, 1));

  try
  {
    listener.lookupTransform("map", "base_footprint_frame", ros::Time(0), transform_ekf);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("Aqui!!!");
  // get roll pitch yaw from transform_robot_pose_ekf
  double roll_1, pitch_1, yaw_1;
  tf::Quaternion q1 = transform_ekf.getRotation();
  tf::Matrix3x3(q1).getRPY(roll_1, pitch_1, yaw_1);
  // ROS_INFO("R: %f, P: %f, Y: %f", roll_1, pitch_1, yaw_1);
  tf::Transform transform;

  // transformação
  transform.setOrigin(tf::Vector3((transform_ekf.getOrigin()).x() / 100, (transform_ekf.getOrigin()).y() / 100, 0));
  // ROS_INFO("X: %f, Y: %f", transform_ekf.getOrigin().x(), transform_ekf.getOrigin().y());
  transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, yaw_1));

  // publicar tensformação entre o referencial do mundo e o ground
  br.sendTransform(tf::StampedTransform(transform, transform_ekf.stamp_, "map", "ground"));
}
*/

// Main
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ground_tf_broadcaster");
  GroundPosition ground_pos;
  ros::spin();
  return 0;
}
