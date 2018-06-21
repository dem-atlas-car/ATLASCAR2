//---------------------------------------------------
// using namespace laser_assembler;

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "test_client");
//   ros::NodeHandle n;
//   ros::Publisher pub_cloud0 = nh_.advertise<sensor_msgs::PointCloud2>("cloud_minada0", 100);
//   ros::Publisher pub_cloud3 = n.advertise<sensor_msgs::PointCloud2>("cloud_minada3", 100);
//   ros::Rate rate(20);

//   while (ros::ok())
//   {
//     ros::service::waitForService("assemble_scans0");
//     ros::ServiceClient client = n.serviceClient<AssembleScans2>("assemble_scans0");
//     AssembleScans2 srv;
//     srv.request.begin = ros::Time(0, 0);
//     srv.request.end = ros::Time::now();

//     if (client.call(srv))
//     {
//       printf("Got cloud 0 with %lu points\n", srv.response.cloud.data.size());
//       pub_cloud0.publish(srv.response.cloud);
//     }
//     else
//       printf("Service call failed\n");

//     ros::service::waitForService("assemble_scans3");
//     ros::ServiceClient client3 = n.serviceClient<AssembleScans2>("assemble_scans3");
//     AssembleScans2 srv3;

//     srv3.request.begin = ros::Time(0, 0);
//     srv3.request.end = ros::Time::now();
//     if (client3.call(srv3))
//     {
//       printf("Got cloud 3 with %lu points\n", srv3.response.cloud.data.size());
//       pub_cloud3.publish(srv3.response.cloud);
//     }
//     else
//       printf("Service call failed\n");

//     ros::spinOnce();
//     rate.sleep();
//   }

//   return 0;
// }

//--------------------------------***-------------------------------

// #include <laser_geometry/laser_geometry.h>
// #include <pcl/PCLPointCloud2.h>
// #include <pcl/filters/conditional_removal.h>
// #include <pcl/filters/radius_outlier_removal.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <ros/ros.h>
// #include <tf/transform_listener.h>
// #include <visualization_msgs/Marker.h>
// #include <Eigen/Core>

// using namespace pcl;
// class My_Filter
// {
// public:
//   My_Filter();
//   void scanCallback3(const sensor_msgs::LaserScan::ConstPtr& scan);
//   void scanCallback2(const sensor_msgs::LaserScan::ConstPtr& scan);
//   sensor_msgs::PointCloud2 remove_extra_points(sensor_msgs::PointCloud2 cloud, float dist);

// private:
//   ros::NodeHandle node_;
//   laser_geometry::LaserProjection projector_;
//   tf::TransformListener tfListener_;
//   ros::Publisher point_cloud_publisher_;
//   ros::Subscriber scan_sub3_;
//   ros::Subscriber scan_sub2_;
//   sensor_msgs::PointCloud2 cloud;
// };

// My_Filter::My_Filter()
// {
//   scan_sub3_ = node_.subscribe<sensor_msgs::LaserScan>("/ld_rms/scan1", 100, &My_Filter::scanCallback3, this);
//   // scan_sub2_ = node_.subscribe<sensor_msgs::LaserScan>("/ld_rms/scan2", 100, &My_Filter::scanCallback2, this);
//   point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2>("/my_cloud_in", 100, false);
//   // tfListener_.setExtrapolationLimit(ros::Duration(0.1));
// }

// sensor_msgs::PointCloud2 My_Filter::remove_extra_points(sensor_msgs::PointCloud2 cloud, float dist)
// {
//   tf::StampedTransform transformOdom;
//   tf::TransformListener listener;
//   // current_time = ros::Time::now();
//   // transformar a nuvem de pontos para o tipo XYZ
//   // pcl::PointCloud<pcl::PointXYZ> cloud_to_clean;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_clean(new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::fromROSMsg(cloud, *cloud_to_clean);

//   try
//   {
//     listener.waitForTransform("map", "base_link_imu", ros::Time(0), ros::Duration(2.0));
//     listener.lookupTransform("map", "base_link_imu", ros::Time(0), transformOdom);
//   }
//   catch (tf::TransformException& ex)
//   {
//     ROS_ERROR("%s", ex.what());
//   }
//   // Localização da origem do ref base_link_imu
//   float Xo = transformOdom.getOrigin().x();
//   float Yo = transformOdom.getOrigin().y();
//   float Zo = transformOdom.getOrigin().z();

//   // ROS_INFO("X pose %f", transformOdom.getOrigin().x());
//   // ROS_INFO("Y pose %f", transformOdom.getOrigin().y());

//   // Variavel a ser retornada
//   sensor_msgs::PointCloud2 cloud2;
//   // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_clean(new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());

//   // Condição para os limites da bounding box de representação da pointcloud

//   range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
//       new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, Yo - 20)));
//   range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
//       new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, Yo + 20)));

//   // build the filter
//   pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
//   condrem.setCondition(range_cond);
//   condrem.setInputCloud(cloud_to_clean);
//   condrem.setKeepOrganized(true);
//   // apply filter
//   condrem.filter(*cloud_filtered);

//   // Filtro de Voxel para dimuinuir os pontos na pc do laser-scan
//   VoxelGrid<pcl::PointXYZ> vg;
//   PointCloud<pcl::PointXYZ>::Ptr cloud_filteredVox(new PointCloud<pcl::PointXYZ>);
//   vg.setInputCloud(cloud_filtered);
//   vg.setLeafSize(0.2f, 0.2f, 0.2f);
//   vg.filter(*cloud_filteredVox);

//   // ROS_INFO("Tamanho: %ld", (*cloud_filteredVox).size());
//   // converter para mensagem para ser publicada
//   pcl::toROSMsg(*cloud_filteredVox, cloud2);

//   return cloud2;
// }

// void My_Filter::scanCallback3(const sensor_msgs::LaserScan::ConstPtr& scan)
// {
//   // projector_.projectLaser(*scan, cloud);
//   if (!tfListener_.waitForTransform(
//           scan->header.frame_id, "map",
//           scan->header.stamp + ros::Duration().fromSec(scan->ranges.size() * scan->time_increment),
//           ros::Duration(1.0)))
//   {
//     return;
//   }

//   sensor_msgs::PointCloud2 cloud_clean;
//   projector_.transformLaserScanToPointCloud("map", *scan, cloud, tfListener_);
//   cloud_clean = remove_extra_points(cloud, 5);
//   point_cloud_publisher_.publish(cloud_clean);
// }

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "my_filter");
//   My_Filter filter;

//   ros::spin();

//   return 0;
// }
