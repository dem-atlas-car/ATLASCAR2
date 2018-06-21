#include <global_planning/Pathh.h>
#include <gps_common/GPSFix.h>
// #include <mission_planning/Path.h>
#include <novatel_gps_msgs/NovatelPosition.h>
#include <postgresql/libpq-fe.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <stdio.h>

#include <novatel_gps_msgs/Inspva.h>

#include <algorithm>
#include <cstdlib>
#include <iostream>

#include <iterator>
#include "ros/ros.h"

#include <math.h>
#include <vector>

#include <string>

#include <utility>  //std::pair

#include <pqxx/pqxx>

#include <geometry_msgs/PointStamped.h>

#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <boost/shared_ptr.hpp>
#define d2r (M_PI / 180.0)  // degree to radians




PGconn* conn = PQconnectdb("user=atlascar2 password=atlas dbname=atlas_navegacao_global hostaddr=127.0.0.1 port=5432");
// PGconn *conn = PQconnectdb("user=pedro password=atlascar2 dbname=atlas_navegacao_global hostaddr=127.0.0.1 port=5432");

using namespace gps_common;

class global_path
{
private:
  global_planning::Pathh::Request req;
  global_planning::Pathh::Response resp;

  // Subscribers
  // Bestpos - Position
  ros::Subscriber gps_position;

  ros::Subscriber yaw_sub;

  // Destination local from
  ros::Subscriber gps_destination;
  // ServiceClient
  ros::ServiceClient client;
  ros::Subscriber fix_sub;

  bool has_path = false;
  bool arrived_destination = false;
  bool first_request;

  boost::shared_ptr<ros::Subscriber> sub;
  ros::Publisher waypoints_pub;
  ros::Publisher waypoints_step_pub;
  ros::Publisher waypoints_previous_next_pub;
  ros::Publisher destination_;
  ros::Publisher waypoints_previous_next_wsg84_pub;

  ros::Publisher odom_pub;
  std::string frame_id, child_frame_id;
  double rot_cov;

  std::vector<double> X_way;
  std::vector<double> Y_way;
  std::vector<double> matrix;
  double X_pos;
  double Y_pos;

  double X_way_previous;
  double X_way_next;

  double Y_way_previous;
  double Y_way_next;

  double Lat_way_previous = 0;
  double Lat_way_next = 0;

  double Lon_way_previous = 0;
  double Lon_way_next = 0;
  int index = 0;
  int tmp_index = 0;
  double lon_dest = 0;
  double lat_dest = 0;

  double idxx = 0;

  double YAW = 0;

  std::string s_dbname;
  std::string s_user;
  std::string s_password;
  std::string s_hostaddr;
  int s_port;

public:
  void onInit();
  void Path_onInit();
  void getparameters(ros::NodeHandle n);

  void gpsCallback(const novatel_gps_msgs::NovatelPositionPtr &bestpos);
  void yawCallback(const novatel_gps_msgs::InspvaPtr &imu_inspva);

  void ServiceCallback(const geometry_msgs::PointStampedPtr &des);
  void ProcessPath(const novatel_gps_msgs::NovatelPositionPtr &bestpos);

  void CleanAllTables();
  void InsertWaypointsDB();
  void InsertStepWaypointsDB();
  void PublishPath();

  bool ReCalculation(double dist);

  void PositionXY(const novatel_gps_msgs::NovatelPositionPtr &bestpos);
  void WaypointsToXY();
  double FindDistanceToSegment(double x1, double y1, double x2, double y2, double pointX, double pointY);
  bool ArrivedDestination();
  void GlobaltoLocal();
  void Destination();
};
/*******************************************************************************************/
/*******************************************************************************************/
void global_path::onInit()
{
  ros::NodeHandle n;
  client = n.serviceClient<global_planning::Pathh>("global_path");

  // Posição e orientação
  gps_position = n.subscribe("bestpos", 100, &global_path::gpsCallback, this);
  yaw_sub = n.subscribe("inspva", 100, &global_path::yawCallback, this);

  // Sempre recebe ordem do utilizador via mapviz -> faz pedido de rota
  gps_destination = n.subscribe("destination", 100, &global_path::ServiceCallback, this);

  // topicos para fornecer os waypoints ao mapviz
  waypoints_pub = n.advertise<std_msgs::Float64MultiArray>("/waypoints_full", 2);
  waypoints_step_pub = n.advertise<std_msgs::Float64MultiArray>("/waypoints_steps", 2);
  waypoints_previous_next_wsg84_pub = n.advertise<std_msgs::Float64MultiArray>("/waypoints_previous_next_wsg84", 2);
  waypoints_previous_next_pub = n.advertise<std_msgs::Float64MultiArray>("/waypoints_previous_next", 2);

  // topicos para publicar o destino da missao
  destination_ = n.advertise<std_msgs::Float64MultiArray>("/global_destination", 2);

  // Credenciais Base de dados local
  getparameters(n);

  // Ligação à base de dados
  pqxx::connection C("dbname=" + s_dbname + " user=" + s_user + " password=" + s_password + " hostaddr=" + s_hostaddr +
                     " port=" + std::to_string(s_port));
  if (C.is_open())
    std::cout << "Opened database successfully: " << C.dbname() << std::endl;
  else
    std::cout << "Can't open database" << std::endl;
}

/*******************************************************************************************/
void global_path::getparameters(ros::NodeHandle n)
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

/*******************************************************************************************/
void global_path::gpsCallback(const novatel_gps_msgs::NovatelPositionPtr &bestpos)
{
  // Posicao instantanea usada no serviço;
  req.gpsLat = bestpos->lat;
  req.gpsLon = bestpos->lon;

  global_path::PositionXY(bestpos);   // Posição XY instantanea
  global_path::ProcessPath(bestpos);  // Processamento do caminho
}

/*******************************************************************************************/
void global_path::yawCallback(const novatel_gps_msgs::InspvaPtr &imu_inspva)
{
  YAW = -imu_inspva->azimuth;
}

/*******************************************************************************************/
void global_path::ServiceCallback(const geometry_msgs::PointStampedPtr &dest)
{
  req.destLon = dest->point.x;
  req.destLat = dest->point.y;

  global_path::Path_onInit();
}

/*******************************************************************************************/
void global_path::Path_onInit()
{
  ROS_INFO("global_path start");
  global_path::Destination();
  global_path::CleanAllTables();
  bool espera = false;

  if (client.call(req, resp))
  {
    has_path = true;
    global_path::InsertWaypointsDB();
    global_path::InsertStepWaypointsDB();
    global_path::WaypointsToXY();
    PublishPath();
    espera = false;
  }
  else
  {
    ROS_INFO("client.call  request error");
    has_path = false;
  }
}

/*******************************************************************************************/
// Processamento do caminho ;
// Distancias entre posição e destino;
void global_path::ProcessPath(const novatel_gps_msgs::NovatelPositionPtr &bestpos)
{
  std::vector<double> distance;

  if (has_path)
  {
    for (int i = 0; i < X_way.size() - 1; i++)
    {
      distance.push_back(
          global_path::FindDistanceToSegment(X_way[i], Y_way[i], X_way[i + 1], Y_way[i + 1], X_pos, Y_pos));
    }

    double min_distance = *std::min_element(distance.begin(), distance.end());
    auto it = std::find(distance.begin(), distance.end(), min_distance);
    if (it == distance.end())
    {
    }
    else
    {
      index = std::distance(distance.begin(), it);
    }
    /******************************************************************/
    if (index < tmp_index)  // limpa a lista => se index for menor o carro deu a volta tem de recalcular
    {
      global_path::Path_onInit();  // recalcula a rota
    }
    /******************************************************************/
    for (int i = 0; i < distance.size(); i++)
    {
      if (distance[i] == min_distance)
      {
        if (sqrt((X_pos - X_way[i + 1]) * (X_pos - X_way[i + 1]) + (Y_pos - Y_way[i + 1]) * (Y_pos - Y_way[i + 1])) <
                4 &&
            i != distance.size() - 2)
        {
          X_way_previous = X_way[i + 1];
          X_way_next = X_way[i + 2];

          Y_way_previous = Y_way[i + 1];
          Y_way_next = Y_way[i + 2];

          Lat_way_previous = resp.lat[index + 1];
          Lat_way_next = resp.lat[index + 2];

          Lon_way_previous = resp.lon[index + 1];
          Lon_way_next = resp.lon[index + 2];

          idxx = index + 2;
        }
        else
        {
          X_way_previous = X_way[i];
          X_way_next = X_way[i + 1];

          Y_way_previous = Y_way[i];
          Y_way_next = Y_way[i + 1];

          Lat_way_previous = resp.lat[index];
          Lat_way_next = resp.lat[index + 1];

          Lon_way_previous = resp.lon[index];
          Lon_way_next = resp.lon[index + 1];
          idxx = index + 1;
        }
      }
    }

    std::vector<double> way_prev_next_ws84 = { Lat_way_previous, Lat_way_next, Lon_way_previous, Lon_way_next };

    std_msgs::Float64MultiArray msg_way_wsg84;
    msg_way_wsg84.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg_way_wsg84.layout.dim[0].size = 4;
    msg_way_wsg84.layout.dim[0].stride = 0;
    msg_way_wsg84.layout.dim[0].label = "x";  // or whatever name you typically use to index vec1
    msg_way_wsg84.data.clear();

    msg_way_wsg84.data.insert(msg_way_wsg84.data.end(), way_prev_next_ws84.begin(), way_prev_next_ws84.end());
    waypoints_previous_next_wsg84_pub.publish(msg_way_wsg84);

    global_path::Destination();
    global_path::GlobaltoLocal();
    global_path::ArrivedDestination();

    if (global_path::ReCalculation(min_distance))
    {
      global_path::Path_onInit();  // recalcula a rota
    }
  }
}

/*******************************************************************************************/
void global_path::PublishPath()
{
  std::vector<std::pair<long double, long double>> waypointss;

  for (int i = 0; i < resp.lat.size(); i++)
  {
    waypointss.push_back(std::make_pair(resp.lat[i], resp.lon[i]));
  }

  std_msgs::Float64MultiArray msg_;
  msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg_.layout.dim[0].size = resp.lat.size();
  msg_.layout.dim[1].size = 2;
  msg_.layout.dim[0].stride = 2 * resp.lat.size();
  msg_.layout.dim[1].stride = 2;
  msg_.layout.dim[0].label = "x";  // or whatever name you typically use to index vec1
  msg_.layout.dim[1].label = "y";
  msg_.data.clear();

  msg_.data.insert(msg_.data.end(), resp.lat.begin(), resp.lat.end());
  msg_.data.insert(msg_.data.end(), resp.lon.begin(), resp.lon.end());
  waypoints_pub.publish(msg_);

  std::vector<std::pair<long double, long double>> waypointss_steps;

  for (int i = 0; i < resp.startlat.size(); i++)
  {
    waypointss_steps.push_back(std::make_pair(resp.startlat[i], resp.startlon[i]));
    waypointss_steps.push_back(std::make_pair(resp.endlat[i], resp.endlon[i]));
  }

  std_msgs::Float64MultiArray msg_step;

  msg_step.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg_step.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg_step.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg_step.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg_step.layout.dim[0].size = resp.startlat.size();
  msg_step.layout.dim[1].size = 4;
  msg_step.layout.dim[2].size = 4;
  msg_step.layout.dim[3].size = 4;
  msg_step.layout.dim[0].stride = 4 * resp.startlat.size();
  msg_step.layout.dim[1].stride = 4;
  msg_step.layout.dim[2].stride = 4;
  msg_step.layout.dim[3].stride = 4;
  msg_step.layout.dim[0].label = "x";
  msg_step.layout.dim[1].label = "xx";
  msg_step.layout.dim[2].label = "y";
  msg_step.layout.dim[3].label = "yy";

  msg_step.data.clear();
  msg_step.data.insert(msg_step.data.end(), resp.startlat.begin(), resp.startlat.end());
  msg_step.data.insert(msg_step.data.end(), resp.endlat.begin(), resp.endlat.end());

  msg_step.data.insert(msg_step.data.end(), resp.startlon.begin(), resp.startlon.end());
  msg_step.data.insert(msg_step.data.end(), resp.endlon.begin(), resp.endlon.end());

  waypoints_step_pub.publish(msg_step);
}

/*******************************************************************************************/
// Funções relativas a base de dados
void global_path::CleanAllTables()
{
  pqxx::connection C("dbname=" + s_dbname + " user=" + s_user + " password=" + s_password + " hostaddr=" + s_hostaddr +
                     " port=" + std::to_string(s_port));

  pqxx::work W(C);
  W.exec("DELETE FROM waypoint_table");
  W.commit();

  // pqxx::work WW(C);
  // WW.exec("DELETE FROM gnss_coord_table");
  // WW.commit();

  // pqxx::work WWWW(C);
  // WWWW.exec("DELETE FROM destination_table");
  // WWWW.commit();

  pqxx::work WWWWWW(C);
  WWWWWW.exec("DELETE FROM stepswaypoint_table");
  WWWWWW.commit();

  ROS_INFO("Database is clean");
}

/*******************************************************************************************/
void global_path::InsertWaypointsDB()
{
  pqxx::connection C("dbname=" + s_dbname + " user=" + s_user + " password=" + s_password + " hostaddr=" + s_hostaddr +
                     " port=" + std::to_string(s_port));

  for (int i = 0; i < resp.lat.size(); i++)
  {
    std::string sSQL = "INSERT INTO waypoint_table (way_lat,way_lon) VALUES (" + std::to_string(resp.lat[i]) + "," +
                       std::to_string(resp.lon[i]) + ")";
    pqxx::work W(C);
    W.exec(sSQL);
    W.commit();
  }
  ROS_INFO("Waypoints are in Database");
}

/*******************************************************************************************/
void global_path::InsertStepWaypointsDB()
{
  pqxx::connection C("dbname=" + s_dbname + " user=" + s_user + " password=" + s_password + " hostaddr=" + s_hostaddr +
                     " port=" + std::to_string(s_port));

  std::string instruct;
  int ii = 0;
  for (int i = 0; i < resp.startlat.size(); i++)
  {
    if (i != 0)
    {
      instruct = "'" + resp.instruction[ii] + "'";
      ii++;
    }
    else
    {
      instruct = " 'all steps' ";
    }

    std::string sSQL = "INSERT INTO stepswaypoint_table "
                       "(start_way_lat,start_way_lon,end_way_lat,end_way_lon,instruction) VALUES (" +
                       std::to_string(resp.startlat[i]) + "," + std::to_string(resp.startlon[i]) + "," +
                       std::to_string(resp.endlat[i]) + "," + std::to_string(resp.endlon[i]) + "," + instruct + ")";

    pqxx::work W(C);
    W.exec(sSQL);
    W.commit();
  }
  ROS_INFO("Steps are in Database");
}

/*******************************************************************************************/
void global_path::GlobaltoLocal()
{
  double Xprevf =
      cos(YAW * (M_PI / 180)) * (X_way_previous - X_pos) - sin(YAW * (M_PI / 180)) * (Y_way_previous - Y_pos);

  double Yprevf =
      -sin(YAW * (M_PI / 180)) * (X_way_previous - X_pos) - cos(YAW * (M_PI / 180)) * (Y_way_previous - Y_pos);

  double Xnextf = cos(YAW * (M_PI / 180)) * (X_way_next - X_pos) - sin(YAW * (M_PI / 180)) * (Y_way_next - Y_pos);

  double Ynextf = -sin(YAW * (M_PI / 180)) * (X_way_next - X_pos) - cos(YAW * (M_PI / 180)) * (Y_way_next - Y_pos);

  // ROS_INFO("yaw : %g    Xprev: %g  Yprev:   %g Xnext: %g  Ynext:   %g   ", YAW, Xprevf, Yprevf, Xnextf, Ynextf);

  std::vector<double> way_prev_next_ws84 = {
    Xprevf,       Xnextf,   Yprevf,   Ynextf, Lat_way_previous, Lat_way_next, Lon_way_previous,
    Lon_way_next, lat_dest, lon_dest, idxx,
  };

  std_msgs::Float64MultiArray msg_way;
  msg_way.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg_way.layout.dim[0].size = 10;
  msg_way.layout.dim[0].stride = 0;
  msg_way.layout.dim[0].label = "x";
  msg_way.data.clear();
  msg_way.data.insert(msg_way.data.end(), way_prev_next_ws84.begin(), way_prev_next_ws84.end());

  waypoints_previous_next_pub.publish(msg_way);
}

/*******************************************************************************************/
//   Recalcula a rota sempre que um determinado valor seja ultrapassado.
bool global_path::ReCalculation(double dist)
{
  if (dist > 10)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/*******************************************************************************************/
void global_path::PositionXY(const novatel_gps_msgs::NovatelPositionPtr &bestpos)
{
  if (bestpos->lat == 0)
  {
    ROS_INFO("No Position.");
    return;
  }

  if (bestpos->header.stamp == ros::Time(0))
  {
    return;
  }

  std::string zone;
  LLtoUTM(bestpos->lat, bestpos->lon, X_pos, Y_pos, zone);

  nav_msgs::Odometry odom;
  odom.header.stamp = bestpos->header.stamp;

  if (frame_id.empty())
    odom.header.frame_id = bestpos->header.frame_id;
  else
    odom.header.frame_id = frame_id;

  odom.child_frame_id = child_frame_id;
}

/*******************************************************************************************/
void global_path::WaypointsToXY()
{
  for (int i = 0; i < resp.lat.size(); i++)
  {
    std::string zone;
    X_way.push_back(0);
    Y_way.push_back(0);
    LLtoUTM(resp.lat[i], resp.lon[i], X_way[i], Y_way[i], zone);
  }
}

/*******************************************************************************************/
double global_path::FindDistanceToSegment(double x1, double y1, double x2, double y2, double pointX, double pointY)
{
  double diffX = x2 - x1;
  float diffY = y2 - y1;
  if ((diffX == 0) && (diffY == 0))
  {
    diffX = pointX - x1;
    diffY = pointY - y1;
    return sqrt(diffX * diffX + diffY * diffY);
  }

  float t = ((pointX - x1) * diffX + (pointY - y1) * diffY) / (diffX * diffX + diffY * diffY);

  if (t < 0)
  {
    // point is nearest to the first point i.e x1 and y1
    diffX = pointX - x1;
    diffY = pointY - y1;
  }
  else if (t > 1)
  {
    // point is nearest to the end point i.e x2 and y2
    diffX = pointX - x2;
    diffY = pointY - y2;
  }
  else
  {
    // if perpendicular line intersect the line segment.
    diffX = pointX - (x1 + t * diffX);
    diffY = pointY - (y1 + t * diffY);
  }

  // returning shortest distance
  return sqrt(diffX * diffX + diffY * diffY);
}

/*******************************************************************************************/
bool global_path::ArrivedDestination()
{
  double X_req, Y_req;
  std::string zone;
  LLtoUTM(req.destLat, req.destLon, X_req, Y_req, zone);

  if (sqrt((X_pos - X_req) * (X_pos - X_req) + (Y_pos - Y_req) * (Y_pos - Y_req)) <
      10)  // Se está a menos de 10 metros do destino, então chegou ao seu destino
  {
    arrived_destination = true;
    has_path = false;

    ROS_INFO("ARRIVED AT YOUR DESTINATION!!");
    return true;
  }
  else
  {
    return false;
  }
}

/*******************************************************************************************/
void global_path::Destination()
{
  if (has_path)
  {
    lon_dest = req.destLon;
    lat_dest = req.destLat;
  }
  else
  {
    lon_dest = 0;
    lat_dest = 0;
  }

  std_msgs::Float64MultiArray msg_dest;
  std::vector<double> desti = { lat_dest, lon_dest };
  // set up dimensions
  msg_dest.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg_dest.layout.dim[0].size = 2;
  msg_dest.layout.dim[0].stride = 0;
  msg_dest.layout.dim[0].label = "x";
  msg_dest.data.clear();

  msg_dest.data.insert(msg_dest.data.end(), desti.begin(), desti.end());
  destination_.publish(msg_dest);
}

// /*******************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "global_planning_client");
  global_path run_class;
  run_class.onInit();

  ros::Rate r(100);
  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
