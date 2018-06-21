#include <geometry_msgs/PointStamped.h>
#include <novatel_gps_msgs/Inspva.h>
#include <novatel_gps_msgs/NovatelPosition.h>
#include <novatel_gps_msgs/NovatelVelocity.h>
#include <postgresql/libpq-fe.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <stdio.h>
#include <time.h>
#include <iostream>
#include <pqxx/pqxx>
#include <string>
#include <vector>

PGconn* conn = PQconnectdb("user=atlas password=atlascar dbname=atlas_monitoring hostaddr=193.137.172.18 port=5432");



using namespace std;
/**
\brief Function for current time
 \details This function gets the instant time from the pc
 \return time
 */
// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const string currentDateTime()
{
  time_t now = time(0);
  struct tm tstruct;
  char buf[80];
  tstruct = *localtime(&now);
  // Visit http://www.cplusplus.com/reference/clibrary/ctime/strftime/
  // for more information about date/time format
  strftime(buf, sizeof(buf), "%X", &tstruct);
  return buf;
}

/**
 \brief Function for current date
 \details This function gets the instant date from the pc
\return date
 */
const string currentDateTimedate()
{
  time_t now = time(0);
  struct tm tstruct;
  char buf[80];
  tstruct = *localtime(&now);
  // Visit http://www.cplusplus.com/reference/clibrary/ctime/strftime/
  // for more information about date/time format
  strftime(buf, sizeof(buf), "%Y-%m-%d", &tstruct);
  return buf;
}

class database_lar
{
private:
  // Bestpos - Position
  ros::Subscriber gps_position;
  // INSPVA - Imu - yaw
  ros::Subscriber imu_yaw;
  // destination
  ros::Subscriber destination_;
  // velocity
  ros::Subscriber velocity_;

  double lat_gps;
  double lon_gps;
  double alt_gps;
  double track_gps;
  double lat_dest = 0;
  double lon_dest = 0;
  double vel;
  int id = 0;

  std::string date;
  std::string time;

  std::string s_dbname;
  std::string s_user;
  std::string s_password;
  std::string s_hostaddr;
  int s_port;

public:
  void onInit();
  void getparameters(ros::NodeHandle n);
  void gpsCallback(const novatel_gps_msgs::NovatelPositionPtr& gps);
  void imuCallback(const novatel_gps_msgs::InspvaPtr& imu_inspva);
  void CleanTable();
  void CopyTable();
  void InsertData();
  void ServiceCallback(const std_msgs::Float64MultiArrayConstPtr& array);
  void velCallback(const novatel_gps_msgs::NovatelVelocityPtr& velocity);
};

void database_lar::onInit()
{
  ros::NodeHandle n;
  // Sempre que tem nova posição no gps.
  gps_position = n.subscribe("bestpos", 100, &database_lar::gpsCallback, this);
  // Sempre que tem nova orientação imu.
  imu_yaw = n.subscribe("inspva", 100, &database_lar::imuCallback, this);
  // Subscrever com a velocidade
  velocity_ = n.subscribe("bestvel", 100, &database_lar::velCallback, this);
  // Subscrever o tópico que publica o destino (mapviz)
  destination_ = n.subscribe("global_destination", 100, &database_lar::ServiceCallback, this);

  getparameters(n);
  pqxx::connection C("dbname=" + s_dbname + " user=" + s_user + " password=" + s_password + " hostaddr=" + s_hostaddr +
                     " port=" + std::to_string(s_port));
  if (C.is_open())
    std::cout << "Opened database successfully: " << C.dbname() << std::endl;
  else
    std::cout << "Can't open database" << std::endl;
};
void database_lar::getparameters(ros::NodeHandle n)
{
  if (n.getParam("dbname", s_dbname))
    ROS_INFO("Got param: %s", s_dbname.c_str());
  else
    ROS_ERROR("Failed to get param 'dbname'");

  if (n.getParam("user", s_user))
    ROS_INFO("Got param: %s", s_user.c_str());
  else
    ROS_ERROR("Failed to get param 'user'");

  if (n.getParam("password", s_password))
    ROS_INFO("Got param: %s", s_password.c_str());
  else
    ROS_ERROR("Failed to get param 'password'");

  if (n.getParam("hostaddr", s_hostaddr))
    ROS_INFO("Got param: %s", s_hostaddr.c_str());
  else
    ROS_ERROR("Failed to get param 'hostaddr'");

  if (n.getParam("port", s_port))
    ROS_INFO("Got param: %d", s_port);
  else
    ROS_ERROR("Failed to get param 'port'");
}
void database_lar::gpsCallback(const novatel_gps_msgs::NovatelPositionPtr& gps)
{
  lat_gps = gps->lat;
  lon_gps = gps->lon;
  alt_gps = gps->height;
  // std::cout << lat_gps << "   " << lon_gps << "   " << alt_gps << "   " << track_gps << "   " << currentDateTime()
            // << "   " << currentDateTimedate() << "   " << lat_dest << "  " << lon_dest << std::endl;

  database_lar::InsertData();
}

void database_lar::imuCallback(const novatel_gps_msgs::InspvaPtr& imu_inspva)
{
  track_gps = imu_inspva->azimuth;
}

void database_lar::ServiceCallback(const std_msgs::Float64MultiArrayConstPtr& array)
{
  // std::cout << "dest service" << std::endl;
  lon_dest = array->data[1];
  lat_dest = array->data[0];
}
void database_lar::velCallback(const novatel_gps_msgs::NovatelVelocityPtr& velocity)
{
  vel = velocity->horizontal_speed;
}
void database_lar::CleanTable()
{
  pqxx::connection C("dbname=" + s_dbname + " user=" + s_user + " password=" + s_password + " hostaddr=" + s_hostaddr +
                     " port=" + std::to_string(s_port));

  pqxx::work W(C);
  W.exec("DELETE FROM global_navigation");
  W.commit();
}

void database_lar::CopyTable()
{
  pqxx::connection C("dbname=" + s_dbname + " user=" + s_user + " password=" + s_password + " hostaddr=" + s_hostaddr +
                     " port=" + std::to_string(s_port));
  pqxx::work WW(C);
  WW.exec("COPY global_navigation TO '/opt/atlascar/atlascar_" + currentDateTimedate() + "_" + currentDateTime() +
          ".csv' DELIMITER ',' CSV HEADER;");
  WW.commit();
}

void database_lar::InsertData()
{
  id++;
  pqxx::connection C("dbname=" + s_dbname + " user=" + s_user + " password=" + s_password + " hostaddr=" + s_hostaddr +
                     " port=" + std::to_string(s_port));

  pqxx::work W(C);
  W.exec("INSERT INTO global_navigation (id,lat_gps,lon_gps, alt_gps,track_gps,date,time,speed,lat_dest,lon_dest) "
         "VALUES (" +
         std::to_string(id) + "," + std::to_string(lat_gps) + "," + std::to_string(lon_gps) + "," +
         std::to_string(alt_gps) + "," + std::to_string(track_gps) + "," + "'" + currentDateTimedate() + "'" + "," +
         "'" + currentDateTime() + "'" + "," + std::to_string(vel) + "," + std::to_string(lat_dest) + "," +
         std::to_string(lon_dest) + ")");
  W.commit();
}

/************************************************************************************************************************************************************************/
/************************************************************************************************************************************************************************/
/************************************************************************************************************************************************************************/
/************************************************************************************************************************************************************************/

/************************************************************************************************************************************************************************/
/************************************************************************************************************************************************************************/
/************************************************************************************************************************************************************************/
/************************************************************************************************************************************************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "database_LAR");
  database_lar run_class;
  run_class.onInit();
  run_class.CleanTable();
  ros::Rate r(20);
  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
  }
  // ros::spin();
  run_class.CopyTable();
  run_class.CleanTable();
  return 0;
}
