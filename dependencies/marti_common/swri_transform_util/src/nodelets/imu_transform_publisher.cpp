//*******************************************
// Ficheiro criado por Pedro Bouça Nova
//

#include <string>

#include <gps_common/GPSFix.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <swri_math_util/constants.h>
#include <swri_math_util/trig_util.h>
#include <swri_roscpp/parameters.h>
#include <swri_transform_util/frames.h>
#include <swri_transform_util/transform_manager.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <novatel_gps_msgs/Gpgga.h>
#include <novatel_gps_msgs/Gpgsa.h>
#include <novatel_gps_msgs/Gprmc.h>
#include <novatel_gps_msgs/Inspva.h>
#include <novatel_gps_msgs/Insstdev.h>
#include <novatel_gps_msgs/NovatelCorrectedImuData.h>
#include <novatel_gps_msgs/NovatelGNSSPosition.h>
#include <novatel_gps_msgs/NovatelPosition.h>
#include <novatel_gps_msgs/NovatelVelocity.h>
#include <novatel_gps_msgs/Range.h>
#include <novatel_gps_msgs/Time.h>
#include <novatel_gps_msgs/Trackstat.h>

#include <sensor_msgs/Imu.h>

#include <boost/make_shared.hpp>
using namespace swri;

namespace swri_transform_util
{
class ImuTransformPublisher : public nodelet::Nodelet
{
private:
  ros::Subscriber imu_sub_;
  ros::Subscriber bestpos_;

  tf::TransformBroadcaster tf_imu;

  swri_transform_util::TransformManager tf_manager_imu_;

  std::string veh_frame_id_;
  std::string global_frame_id_;
  tf::Transform transform;

public:
  void onInit();

  void HandleImu(const novatel_gps_msgs::InspvaPtr& imu_inspva);
  void HandleImuxy(const novatel_gps_msgs::NovatelPositionPtr& bestpos);
};
//*****************************************************************
// Funções

void ImuTransformPublisher::onInit()
{
  ros::NodeHandle prive = getPrivateNodeHandle();

  swri::param(prive, "child_frame_id", veh_frame_id_, std::string("base_link_imu"));
  swri::param(prive, "parent_frame_id", global_frame_id_, std::string("map"));



  imu_sub_ = getNodeHandle().subscribe("inspva", 100, &ImuTransformPublisher::HandleImu, this);

  bestpos_ = getNodeHandle().subscribe("bestpos", 100, &ImuTransformPublisher::HandleImuxy, this);
  tf_manager_imu_.Initialize();
}

void ImuTransformPublisher::HandleImu(const novatel_gps_msgs::InspvaPtr& imu_inspva)
{
  double yaw = (90.0 - imu_inspva->azimuth) * swri_math_util::_deg_2_rad;
  // double yaw = (imu_inspva->azimuth) * swri_math_util::_deg_2_rad;
  // double yaw =  imu_inspva->azimuth * swri_math_util::_deg_2_rad;
  yaw = swri_math_util::WrapRadians(yaw, swri_math_util::_pi);

  tf::Quaternion orientation;
  // orientation.setRPY(0, 0, yaw-1.57075);
    orientation.setRPY(0, 0, yaw);
  transform.setRotation(orientation);
}

void ImuTransformPublisher::HandleImuxy(const novatel_gps_msgs::NovatelPositionPtr& bestpos)
{
  swri_transform_util::Transform to_local_xy;

  if (tf_manager_imu_.GetTransform(global_frame_id_, swri_transform_util::_wgs84_frame, ros::Time(0), to_local_xy))
  {

    tf::Vector3 position(bestpos->lon, bestpos->lat, bestpos->height);
    position = to_local_xy * position;


    transform.setOrigin(position);
    tf_imu.sendTransform(tf::StampedTransform(transform, bestpos->header.stamp, global_frame_id_, veh_frame_id_));
  }
}

}  // namespace lar_transform_util

#include <swri_nodelet/class_list_macros.h>
SWRI_NODELET_EXPORT_CLASS(swri_transform_util, ImuTransformPublisher)
