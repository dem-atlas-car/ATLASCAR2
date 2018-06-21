#include <global_planning/Pathh.h>
#include "ros/ros.h"

#include <stdio.h>

#include <stdlib.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <string>

#include <iostream>
#include <iterator>
#include <vector>

// INCLUDE CURL
#include <curl/curl.h>
#include <boost/asio.hpp>

// INCLUDE JSON
#include <jsoncpp/json/json.h>

#include <std_msgs/String.h>
#include <fstream>
#include <iostream>

#include <polyline/polylineencoder.h>



/***********************************/

/***********************************/

using namespace std;

/*****************************************************************/
/* Dar a string returnar os index de lat ou lng ou outro campo qualquer*/
std::vector<int> field_indx_vector(std::string str, std::string substr)
{
  std::vector<int> idx_vector;

  bool substring_was_found = false;
  for (std::size_t pos = 0; pos < str.size(); ++pos)
  {
    if (str.find(substr, pos) == pos)
    {
      if (!substring_was_found)
      {
        substring_was_found = true;
      }
      // std::cout << pos << ' ';
      idx_vector.push_back(pos + 5);
    }
  }

  if (!substring_was_found)
    std::cout << "String '" << substr << "' not found";

  return idx_vector;
}

/*****************************************************************/
/* BOOL para verificar a existencia do campo*/
bool field_ismember(std::string str, std::string substr)
{
  if (str.find(substr))
  {
    return true;
  }
  else
  {
    return false;
  }
}
/*****************************************************************/
/* Retirar para um vetor os valores dos camps lat e lng*/
std::vector<long double> field_values_vector(std::string str, std::vector<int> idx_values)
{
  std::vector<long double> values_field_vector;

  int sub_indx = 0;
  std::vector<char> temp_string;

  for (int i = 0; i < idx_values.size(); i++)
  {
    if (str[idx_values[i]] == '-' || isdigit(str[idx_values[i]]) || str[idx_values[i]] == '.')
    {
      while (str[idx_values[i] + sub_indx] == '-' || isdigit(str[idx_values[i] + sub_indx]) ||
             str[idx_values[i] + sub_indx] == '.')
      {
        temp_string.push_back(str[idx_values[i] + sub_indx]);

        sub_indx++;
      }

      std::string tp_string(temp_string.begin(), temp_string.end());
      std::string::size_type sz;  // alias of size_t

      long double mars = std::stod(tp_string, &sz);
      values_field_vector.push_back(mars);

      temp_string.clear();
      sub_indx = 0;
    }
  }

  return values_field_vector;
}

/*****************************************************************/
/* Retirar para um vetor os valores dos camps instraction*/
std::vector<string> field_instruction_vector(std::string str, std::vector<int> idx_values)
{
  std::vector<string> instruction_field_vector;

  int sub_indx = 0;
  std::vector<char> temp_string;

  for (int i = 0; i < idx_values.size(); i++)
  {
    if (str[idx_values[i] + 14] == 34)
    {
      do
      {
        temp_string.push_back(str[idx_values[i] + 15 + sub_indx]);

        sub_indx++;
      } while (str[idx_values[i] + sub_indx + 15] != 34);

      std::string str(temp_string.begin(), temp_string.end());

      instruction_field_vector.push_back(str);

      temp_string.clear();
      sub_indx = 0;
    }
  }

  return instruction_field_vector;
}

/*****************************************************************/
/* Retirar para um vetor os valores dos camps instraction*/
std::vector<string> field_points_vector(std::string str, std::vector<int> idx_values)
{
  std::vector<string> points_field_vector;

  int sub_indx = 0;
  std::vector<char> temp_string;

  for (int i = 0; i < idx_values.size(); i++)
  {
    if (str[idx_values[i] + 3] == 34)
    {
      do
      {
        temp_string.push_back(str[idx_values[i] + 4 + sub_indx]);

        sub_indx++;
      } while (str[idx_values[i] + sub_indx + 4] != 34);

      std::string str(temp_string.begin(), temp_string.end());

      points_field_vector.push_back(str);

      temp_string.clear();
      sub_indx = 0;
    }
  }

  return points_field_vector;
}

/*****************************************************************/
/* Retirar para um vetor os valores dos camps instraction*/
std::vector<string> field_steps_vector(std::string str, std::vector<int> idx_values, int id)
{
  std::vector<string> steps_field_vector;

  int sub_indx = 0;
  std::vector<char> temp_string;

  for (int i = 0; i < idx_values.size(); i++)
  {
    if (str[idx_values[i] + id] == 123)
    {
      do
      {
        temp_string.push_back(str[idx_values[i] + id + sub_indx]);

        sub_indx++;
      } while (str[idx_values[i] + sub_indx + id] != 125);

      if (str[idx_values[i] + sub_indx + id] == 125)
      {
        temp_string.push_back(str[idx_values[i] + id + sub_indx]);
      }

      std::string str(temp_string.begin(), temp_string.end());

      steps_field_vector.push_back(str);

      temp_string.clear();
      sub_indx = 0;
    }
  }

  return steps_field_vector;
}

/*****************************************************************/
/* Retirar para um vetor os valores dos camps lat e lng*/
long double field_values(std::string str, std::string field)
{
  long double values_field;
  std::vector<int> _ind = field_indx_vector(str, field);

  std::vector<long double> values_field_vector = field_values_vector(str, _ind);

  values_field = values_field_vector[0];

  return values_field;
}

/*****************************************************************/
std::string ReplaceAll(std::string str, const std::string &from, const std::string &to)
{
  size_t start_pos = 0;
  while ((start_pos = str.find(from, start_pos)) != std::string::npos)
  {
    str.replace(start_pos, from.length(), to);
    start_pos += to.length();  // Handles case where 'to' is a substring of 'from'
  }
  return str;
}

/*****************************************************************/
// FUNCTION TO DECODE ENCODED POLYLINE
vector<double> Decompress(string encodedPoints, int precision)
{
  int len = encodedPoints.length();
  int index = 0;
  double lat = 0;
  double lng = 0;
  vector<double> array;

  while (index < len)
  {
    int b;
    int shift = 0;
    int result = 0;
    do
    {
      b = (int)encodedPoints[index++] - 63;  // gets ascii value of the char
      // result |= (b & 0x1f) << shift;
      result |= (b & 31) << shift;
      shift += 5;
    } while (b >= 32);
    int dlat = ((result & 1) ? ~(result >> 1) : (result >> 1));
    lat += dlat;
    shift = 0;
    result = 0;

    do
    {
      b = (int)encodedPoints[index++] - 63;  // gets ascii value of the char
      result |= (b & 31) << shift;
      shift += 5;
    } while (b >= 32);
    int dlng = ((result & 1) ? ~(result >> 1) : (result >> 1));
    lng += dlng;

    array.push_back(lat * pow(10, -precision));
    array.push_back(lng * pow(10, -precision));
  }
  return array;
}

/*****************************************************************/
// WRITE CURL RESPONSE TO STRING FORMAT
size_t write_to_string(void *ptr, size_t size, size_t nmemb, std::string *data)
{
  data->append((const char *)ptr, size * nmemb);
  return size * nmemb;
}
/*****************************************************************/

/*****************************************************************/
/*****************************************************************/
/*****************************************************************/
class route_planner
{
private:
  std::string s_api_key;

public:
  void getParam(ros::NodeHandle n);
  bool return_path(global_planning::Pathh::Request &req, global_planning::Pathh::Response &resp);
};

/*****************************************************************/
void route_planner::getParam(ros::NodeHandle n)
{
  // ros::NodeHandle n;
  if (n.getParam("api_key_google_maps_api_directions", s_api_key))
    ROS_INFO("Got param: %s", s_api_key.c_str());
  else
    ROS_ERROR("Failed to get param 'api_key_google_maps_api_directions'");
}

/*****************************************************************/
bool route_planner::return_path(global_planning::Pathh::Request &req, global_planning::Pathh::Response &resp)
{
  double startLat = req.gpsLat;
  double startLon = req.gpsLon;
  double destLat = req.destLat;
  double destLon = req.destLon;

  // STEP 1 - GET JSON VIA CURL
  int http_code = 0;
  string data;

  CURL *curl;
  CURLcode res;

  // Initialize cURL
  curl = curl_easy_init();

  if (curl)
  {
    // Set the function to call when there is new data
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_to_string);

    // Set the parameter to append the new data to
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &data);

    double startLat = req.gpsLat;
    double startLon = req.gpsLon;
    double destLat = req.destLat;
    double destLon = req.destLon;

    // string htmlquery = "https://maps.googleapis.com/maps/api/directions/json?origin=" + to_string(req.gpsLat) + "," +
    //                    to_string(req.gpsLon) + "&destination=" + to_string(req.destLat) + "," +
    //                    to_string(req.destLon) +
    //                    "&key=AIzaSyDJzTcc4EOMFkfoF3-mKfN92ixV_1Wfaz8";

    string htmlquery = "https://maps.googleapis.com/maps/api/directions/json?origin=" + to_string(req.gpsLat) + "," +
                       to_string(req.gpsLon) + "&destination=" + to_string(req.destLat) + "," + to_string(req.destLon) +
                       "&key=" + s_api_key;

    // Set the URL to download
    curl_easy_setopt(curl, CURLOPT_URL, htmlquery.c_str());
    /* Follow HTTP redirection in necessary */
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

    /* Perform the request, res will get the return code (Downloads content) */
    res = curl_easy_perform(curl);
    /* Check for errors */
    if (res != CURLE_OK)
      fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));

    // Get the HTTP response code
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

    /* Always cleanup */
    curl_easy_cleanup(curl);
    curl_global_cleanup();

    Json::Value jsonData;
    Json::Reader jsonReader;

    if (http_code == 200)
    {
      ROS_INFO("Server OK, JSON file acquired ");
      // Response looks good - done using Curl now.  Try to parse the results and print them out.

      if (jsonReader.parse(data, jsonData))
      {
        // std::cout << "Successfully parsed JSON data" << std::endl;
        // std::cout << "\nJSON data received:" << std::endl;
        std::cout << jsonData.toStyledString() << std::endl;
      }
      else
      {
        return false;
      }

      Json::Value rota = jsonData["routes"];
      Json::FastWriter fastWriter;
      std::string output = fastWriter.write(rota);

      /**********************************************************************************************************************************************/
      if (field_ismember(output, "lat"))
      {
        std::vector<int> vect = field_indx_vector(output, "lat");
        std::vector<long double> values_field_vector = field_values_vector(output, vect);
      }
      else
      {
        ROS_INFO("Waypoints : latitude data not exist");
      }

      /**********************************************************************************************************************************************/
      if (field_ismember(output, "lng"))
      {
        std::vector<int> vectlng = field_indx_vector(output, "lng");

        std::vector<long double> values_field_vector2 = field_values_vector(output, vectlng);
      }
      else
      {
        ROS_INFO("Waypoints : longitude data not exist");
      }

      /**********************************************************************************************************************************************/
      if (field_ismember(output, "html_instructions"))
      {
        std::vector<int> vect_ind = field_indx_vector(output, "html_instructions");
        std::vector<string> ve = field_instruction_vector(output, vect_ind);

        for (std::vector<string>::iterator it = ve.begin(); it != ve.end(); ++it)
        {
          resp.instruction.push_back(*it);
        }
      }
      else
      {
      }

      /**********************************************************************************************************************************************/
      if (field_ismember(output, "points"))
      {
        cout << "waypoints descodificados" << endl;
        std::vector<int> points_ind = field_indx_vector(output, "points");

        std::vector<string> points = field_points_vector(output, points_ind);
        vector<pair<double, double>> waypoints;  // pair of lat,lon values
        for (std::vector<string>::iterator it = points.end() - 1; it != points.end(); ++it)
        {
          std::string te = *it;
          string barra(1, (char)92);

          std::string teste = ReplaceAll(te, std::string("\\\\"), barra);

          vector<double> data = Decompress(teste, 5);

          for (int i = 0; i < data.size(); i += 2)
          {
            waypoints.push_back(make_pair(data[i], data[i + 1]));  // pair of lat,lon values

            printf("%lf , %lf \n", data[i], data[i + 1]);

            resp.lat.push_back(data[i]);
            resp.lon.push_back(data[i + 1]);
          }
        }
        ROS_INFO("Waypoints  size : %ld", waypoints.size());
      }
      else
      {
        ROS_INFO("Waypoints not exist");
      }

      /**********************************************************************************************************************************************/
      if (field_ismember(output, "start_location"))
      {
        std::vector<int> start_step_ind = field_indx_vector(output, "start_location");
        std::vector<string> steps_start = field_steps_vector(output, start_step_ind, 11);

        std::vector<int> end_step_ind = field_indx_vector(output, "end_location");
        std::vector<string> steps_end = field_steps_vector(output, end_step_ind, 9);

        for (std::vector<string>::iterator it = steps_start.begin(); it != steps_start.end(); ++it)
        {
          std::string tp = *it;

          long double lat = field_values(tp, "lat");
          long double lon = field_values(tp, "lng");

          resp.startlat.push_back(lat);
          resp.startlon.push_back(lon);
        }

        for (std::vector<string>::iterator it = steps_end.begin(); it != steps_end.end(); ++it)
        {
          std::string tp = *it;

          long double lat = field_values(tp, "lat");
          long double lon = field_values(tp, "lng");

          resp.endlat.push_back(lat);
          resp.endlon.push_back(lon);
        }
      }
    }
  }

  ROS_INFO("route service finished");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "global_planning_server");
  ros::NodeHandle n;

  route_planner objt;
  objt.getParam(n);

  route_planner obj;
  ros::ServiceServer service = n.advertiseService("global_path", &route_planner::return_path, &obj);
  ROS_INFO("Ready to call");
  ros::spin();

  return 0;
}
