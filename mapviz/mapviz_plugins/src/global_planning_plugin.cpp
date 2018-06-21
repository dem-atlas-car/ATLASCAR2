//*******************************************
//
// Criado por Pedro Bouça Nova

// #include <mapviz_plugins/bestpos_plugin.h>
#include <mapviz_plugins/global_planning_plugin.h>
#include <mapviz_plugins/gps_plugin.h>
// C++ standard libraries
#include <cstdio>
#include <vector>

// QT libraries
#include <QDialog>
#include <QGLWidget>
#include <QPalette>

#include <opencv2/core/core.hpp>

// ROS libraries
#include <ros/master.h>

#include <mapviz/select_topic_dialog.h>
#include <swri_image_util/geometry_util.h>
#include <swri_transform_util/transform_util.h>

#include <std_msgs/Float64MultiArray.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::GlobalPlanningPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
GlobalPlanningPlugin::GlobalPlanningPlugin() : config_widget_(new QWidget())
{
  ui_.setupUi(config_widget_);

  ui_.color->setColor(Qt::green);

  // Set background white
  QPalette p(config_widget_->palette());
  p.setColor(QPalette::Background, Qt::white);
  config_widget_->setPalette(p);

  // Set status text red
  QPalette p3(ui_.status->palette());
  p3.setColor(QPalette::Text, Qt::red);
  ui_.status->setPalette(p3);

  QObject::connect(ui_.selecttopic, SIGNAL(clicked()), this, SLOT(SelectTopic()));
  QObject::connect(ui_.topic, SIGNAL(editingFinished()), this, SLOT(TopicEdited()));
  QObject::connect(ui_.positiontolerance, SIGNAL(valueChanged(double)), this, SLOT(PositionToleranceChanged(double)));
  QObject::connect(ui_.buffersize, SIGNAL(valueChanged(int)), this, SLOT(BufferSizeChanged(int)));
  QObject::connect(ui_.drawstyle, SIGNAL(activated(QString)), this, SLOT(SetDrawStyle(QString)));
  QObject::connect(ui_.static_arrow_sizes, SIGNAL(clicked(bool)), this, SLOT(SetStaticArrowSizes(bool)));
  QObject::connect(ui_.arrow_size, SIGNAL(valueChanged(int)), this, SLOT(SetArrowSize(int)));
  connect(ui_.color, SIGNAL(colorEdited(const QColor &)), this, SLOT(SetColor(const QColor &)));
}

GlobalPlanningPlugin::~GlobalPlanningPlugin()
{
}

void GlobalPlanningPlugin::SelectTopic()
{
  ros::master::TopicInfo topic = mapviz::SelectTopicDialog::selectTopic("std_msgs/Float64MultiArray");

  if (!topic.name.empty())
  {
    ui_.topic->setText(QString::fromStdString(topic.name));
    TopicEdited();
  }
}

void GlobalPlanningPlugin::TopicEdited()
{
  std::string topic = ui_.topic->text().trimmed().toStdString();
  if (topic != topic_)
  {
    initialized_ = false;
    points_.clear();
    has_message_ = false;
    PrintWarning("No messages received.");

    waypoints_.shutdown();

    topic_ = topic;

    if (!topic.empty())
    {
      if (topic == "/waypoints_full")
      {
        waypoints_ = node_.subscribe(topic_, 1, &GlobalPlanningPlugin::WaypointsCallback, this);
      }
      if (topic == "/waypoints_steps")
      {
        waypoints_ = node_.subscribe(topic_, 1, &GlobalPlanningPlugin::WaypointsStepsCallback, this);
      }
      if (topic == "/waypoints_previous_next_wsg84")
      {
        waypoints_ = node_.subscribe(topic_, 1, &GlobalPlanningPlugin::WaypointsPrevNextCallback, this);
      }

      bestpos_sub_ = node_.subscribe("bestpos", 1, &GlobalPlanningPlugin::bestposCallback, this);
    }
  }
}

int sopas;

void GlobalPlanningPlugin::bestposCallback(const novatel_gps_msgs::NovatelPositionPtr &bestpos)
{
  // StampedPoint stamped_point;
  stamped_point_point.stamp = bestpos->header.stamp;
}

void GlobalPlanningPlugin::WaypointsCallback(const std_msgs::Float64MultiArrayConstPtr &array)
{
  if (!local_xy_util_.Initialized())
  {
    return;
  }
  if (!has_message_)
  {
    initialized_ = true;
    has_message_ = true;
  }

  sopas += 1;

  for (std::vector<double>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
  {
    // ROS_ERROR(" Callback   callback %lf", *it);
  }

  int size = array->data.size();
  int size_ = size / 2;
  // ROS_ERROR("size total  %d   lat ou lon   :::: %d", size, size_);
  // ROS_ERROR("size total  %lf   lat ou lon   :::: %lf::::::::::%lf", array->data[size_ - 1], array->data[size_],
  //           array->data[size_ + 1]);

  double lat_way[size_];
  double lon_way[size_];

  for (int i = 0; i < size_; i++)
  {
    lat_way[i] = array->data[i];

    // ROS_ERROR("array lat  %lf", lat_way[i]);
  }
  int ii = 0;
  for (int i = size_; i < size; i++)
  {
    lon_way[ii] = array->data[i];

    // ROS_ERROR("array lon  %lf", lon_way[ii]);
    ii++;
  }

  /************************************************************/

  for (int i = 0; i < size_; i++)
  // for (int i = 0; i < 1; i++)
  {
    StampedPoint stamped_point;

    if (!local_xy_util_.Initialized())
    {
      ROS_ERROR("local not initialized");
      return;
    }
    if (!has_message_)
    {
      initialized_ = true;
      has_message_ = true;
    }
    // stamped_point.stamp = ros::Time::now();
    stamped_point.stamp = stamped_point_point.stamp;
    stamped_point.source_frame = local_xy_util_.Frame();

    double x = 0;
    double y = 0;

    local_xy_util_.ToLocalXy(lat_way[i], lon_way[i], x, y);

    stamped_point.point = tf::Point(x, y, 0);
    lap_checked_ = ui_.show_laps->isChecked();

    stamped_point.orientation = tf::createQuaternionFromYaw(M_PI_2);

    if (points_.empty() || (stamped_point.point.distance(points_.back().point)) >= (position_tolerance_))
    {
      points_.push_back(stamped_point);
    }

    if (buffer_size_ > 0)
    {
      while (static_cast<int>(points_.size()) > buffer_size_)
      {
        points_.pop_front();
      }
    }

    cur_point_ = stamped_point;
    swri_transform_util::LocalXyWgs84Util local_xy_util_;
  }
}
void GlobalPlanningPlugin::WaypointsStepsCallback(const std_msgs::Float64MultiArrayConstPtr &array)
{
  if (!local_xy_util_.Initialized())
  {
    return;
  }
  if (!has_message_)
  {
    initialized_ = true;
    has_message_ = true;
  }

  int size = array->data.size();
  int size_ = size / 4;
  // ROS_ERROR("size total  %d   lat ou lon   :::: %d", size, size_);
  // ROS_ERROR("size total  %lf   lat ou lon   :::: %lf::::::::::%lf", array->data[size_ - 1], array->data[size_],
  //           array->data[size_ + 1]);

  double lat_way[size_ * 2];
  double lon_way[size_ * 2];
  int iii = 0;
  for (int i = 0; i < size_; i++)
  {
    lat_way[iii] = array->data[i];
    lat_way[iii + 1] = array->data[i + size_];

    // ROS_ERROR("array lat  step %lf", lat_way[iii]);
    iii += 2;
  }
  int ii = 0;
  for (int i = 0; i < size_; i++)
  {
    lon_way[ii] = array->data[i + 2 * size_];
    lon_way[ii + 1] = array->data[i + 3 * size_];

    // ROS_ERROR("array lon step %lf", lon_way[ii]);
    ii += 2;
  }
  for (int i = 0; i < 2 * size_; i++)
  {
    // ROS_ERROR("array lat  step %lf", lat_way[i]);
    // ROS_ERROR("array lon step %lf", lon_way[i]);
  }

  /************************************************************/

  for (int i = 0; i < size_ * 2; i++)
  // for (int i = 0; i < 1; i++)
  {
    StampedPoint stamped_point;

    if (!local_xy_util_.Initialized())
    {
      ROS_ERROR("local not initialized");
      return;
    }
    if (!has_message_)
    {
      initialized_ = true;
      has_message_ = true;
    }
    // stamped_point.stamp = ros::Time::now();
    stamped_point.stamp = stamped_point_point.stamp;
    stamped_point.source_frame = local_xy_util_.Frame();

    double x = 0;
    double y = 0;

    local_xy_util_.ToLocalXy(lat_way[i], lon_way[i], x, y);

    stamped_point.point = tf::Point(x, y, 0);
    lap_checked_ = ui_.show_laps->isChecked();

    stamped_point.orientation = tf::createQuaternionFromYaw(M_PI_2);

    if (points_.empty() || (stamped_point.point.distance(points_.back().point)) >= (position_tolerance_))
    {
      points_.push_back(stamped_point);
    }

    if (buffer_size_ > 0)
    {
      while (static_cast<int>(points_.size()) > buffer_size_)
      {
        points_.pop_front();
      }
    }

    cur_point_ = stamped_point;
    swri_transform_util::LocalXyWgs84Util local_xy_util_;
  }
}

void GlobalPlanningPlugin::WaypointsPrevNextCallback(const std_msgs::Float64MultiArrayConstPtr &array)
{
  if (!local_xy_util_.Initialized())
  {
    return;
  }
  if (!has_message_)
  {
    initialized_ = true;
    has_message_ = true;
  }

  int size = array->data.size();
  int size_ = size / 4;
  // ROS_ERROR("PREV NEXT size total  %d   lat ou lon   :::: %d", size, size_);
  // ROS_ERROR("size total  %lf   lat ou lon   :::: %lf::::::::::%lf", array->data[size_ - 1], array->data[size_],
  //           array->data[size_ + 1]);

  double lat_way[size_ * 2];
  double lon_way[size_ * 2];
  int iii = 0;
  for (int i = 0; i < size_; i++)
  {
    lat_way[iii] = array->data[i];
    lat_way[iii + 1] = array->data[i + size_];

    // ROS_ERROR("array lat  step %lf", lat_way[iii]);
    iii += 2;
  }
  int ii = 0;
  for (int i = 0; i < size_; i++)
  {
    lon_way[ii] = array->data[i + 2 * size_];
    lon_way[ii + 1] = array->data[i + 3 * size_];

    // ROS_ERROR("array lon step %lf", lon_way[ii]);
    ii += 2;
  }
  for (int i = 0; i < 2 * size_; i++)
  {
    // ROS_ERROR("PREV NEXT array lat  step %lf", lat_way[i]);
    // ROS_ERROR("PREV NEXT array lon step %lf", lon_way[i]);
  }

  /************************************************************/

  for (int i = 0; i < size_ * 2; i++)
  // for (int i = 0; i < 1; i++)
  {
    StampedPoint stamped_point;

    if (!local_xy_util_.Initialized())
    {
      ROS_ERROR("local not initialized");
      return;
    }
    if (!has_message_)
    {
      initialized_ = true;
      has_message_ = true;
    }
    // stamped_point.stamp = ros::Time::now();
    stamped_point.stamp = stamped_point_point.stamp;
    stamped_point.source_frame = local_xy_util_.Frame();

    double x = 0;
    double y = 0;

    local_xy_util_.ToLocalXy(lat_way[i], lon_way[i], x, y);

    stamped_point.point = tf::Point(x, y, 0);
    lap_checked_ = ui_.show_laps->isChecked();

    stamped_point.orientation = tf::createQuaternionFromYaw(M_PI_2);

    if (points_.empty() || (stamped_point.point.distance(points_.back().point)) >= (position_tolerance_))
    {
      points_.push_back(stamped_point);
    }

    if (buffer_size_ > 0)
    {
      while (static_cast<int>(points_.size()) > buffer_size_)
      {
        points_.pop_front();
      }
    }

    cur_point_ = stamped_point;
    swri_transform_util::LocalXyWgs84Util local_xy_util_;
  }
}
void GlobalPlanningPlugin::PrintError(const std::string &message)
{
  PrintErrorHelper(ui_.status, message);
}

void GlobalPlanningPlugin::PrintInfo(const std::string &message)
{
  PrintInfoHelper(ui_.status, message);
}

void GlobalPlanningPlugin::PrintWarning(const std::string &message)
{
  PrintWarningHelper(ui_.status, message);
}

QWidget *GlobalPlanningPlugin::GetConfigWidget(QWidget *parent)
{
  config_widget_->setParent(parent);

  return config_widget_;
}

bool GlobalPlanningPlugin::Initialize(QGLWidget *canvas)
{
  canvas_ = canvas;
  SetColor(ui_.color->color());

  return true;
}

void GlobalPlanningPlugin::Draw(double x, double y, double scale)
{
  if (DrawPoints(scale))
  {
    PrintInfo("OK");
  }
}

void GlobalPlanningPlugin::LoadConfig(const YAML::Node &node, const std::string &path)
{
  if (node["topic"])
  {
    std::string topic;
    node["topic"] >> topic;
    ui_.topic->setText(topic.c_str());
  }

  if (node["color"])
  {
    std::string color;
    node["color"] >> color;
    SetColor(QColor(color.c_str()));
    ui_.color->setColor(color_);
  }

  if (node["draw_style"])
  {
    std::string draw_style;
    node["draw_style"] >> draw_style;

    if (draw_style == "lines")
    {
      draw_style_ = LINES;
      ui_.drawstyle->setCurrentIndex(0);
    }
    else if (draw_style == "points")
    {
      draw_style_ = POINTS;
      ui_.drawstyle->setCurrentIndex(1);
    }
    else if (draw_style == "arrows")
    {
      draw_style_ = ARROWS;
      ui_.drawstyle->setCurrentIndex(2);
    }
  }

  if (node["position_tolerance"])
  {
    node["position_tolerance"] >> position_tolerance_;
    ui_.positiontolerance->setValue(position_tolerance_);
  }

  if (node["buffer_size"])
  {
    node["buffer_size"] >> buffer_size_;
    ui_.buffersize->setValue(buffer_size_);
  }

  if (node["show_laps"])
  {
    bool show_laps = false;
    node["show_laps"] >> show_laps;
    ui_.show_laps->setChecked(show_laps);
  }

  if (node["static_arrow_sizes"])
  {
    bool static_arrow_sizes = node["static_arrow_sizes"].as<bool>();
    ui_.static_arrow_sizes->setChecked(static_arrow_sizes);
    SetStaticArrowSizes(static_arrow_sizes);
  }

  if (node["arrow_size"])
  {
    ui_.arrow_size->setValue(node["arrow_size"].as<int>());
  }

  TopicEdited();
}

void GlobalPlanningPlugin::SaveConfig(YAML::Emitter &emitter, const std::string &path)
{
  std::string topic = ui_.topic->text().toStdString();
  emitter << YAML::Key << "topic" << YAML::Value << topic;

  emitter << YAML::Key << "color" << YAML::Value << ui_.color->color().name().toStdString();

  std::string draw_style = ui_.drawstyle->currentText().toStdString();
  emitter << YAML::Key << "draw_style" << YAML::Value << draw_style;

  emitter << YAML::Key << "position_tolerance" << YAML::Value << position_tolerance_;

  if (!lap_checked_)
  {
    emitter << YAML::Key << "buffer_size" << YAML::Value << buffer_size_;
  }
  else
  {
    emitter << YAML::Key << "buffer_size" << YAML::Value << buffer_holder_;
  }

  bool show_laps = ui_.show_laps->isChecked();
  emitter << YAML::Key << "show_laps" << YAML::Value << show_laps;

  emitter << YAML::Key << "static_arrow_sizes" << YAML::Value << ui_.static_arrow_sizes->isChecked();

  emitter << YAML::Key << "arrow_size" << YAML::Value << ui_.arrow_size->value();
}
}
