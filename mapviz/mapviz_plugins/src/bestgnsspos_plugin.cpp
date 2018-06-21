//*******************************************
//
// Criado por Pedro Bouça Nova

#include <mapviz_plugins/bestgnsspos_plugin.h>
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

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::BestgnssposPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
BestgnssposPlugin::BestgnssposPlugin() : config_widget_(new QWidget())
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
  connect(ui_.color, SIGNAL(colorEdited(const QColor&)), this, SLOT(SetColor(const QColor&)));
}

BestgnssposPlugin::~BestgnssposPlugin()
{
}

void BestgnssposPlugin::SelectTopic()
{
  ros::master::TopicInfo topic = mapviz::SelectTopicDialog::selectTopic("novatel_gps_msgs/NovatelGNSSPosition");

  if (!topic.name.empty())
  {
    ui_.topic->setText(QString::fromStdString(topic.name));
    TopicEdited();
  }
}

void BestgnssposPlugin::TopicEdited()
{
  std::string topic = ui_.topic->text().trimmed().toStdString();
  if (topic != topic_)
  {
    initialized_ = false;
    points_.clear();
    has_message_ = false;
    PrintWarning("No messages received.");

    bestpos_sub_.shutdown();

    topic_ = topic;
    if (!topic.empty())
    {
      bestpos_sub_ = node_.subscribe(topic_, 1, &BestgnssposPlugin::BestgnssposCallback, this);

      inspva_sub_ = nh.subscribe("inspva", 1, &BestgnssposPlugin::InspvaCallback, this);

      ROS_INFO("Subscribing to %s", topic_.c_str());
    }
  }
}

void BestgnssposPlugin::BestgnssposCallback(const novatel_gps_msgs::NovatelGNSSPositionPtr& bestgnsspos)
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

  // StampedPoint stamped_point;
  stamped_point.stamp = bestgnsspos->header.stamp;
  stamped_point.source_frame = local_xy_util_.Frame();
  double x;
  double y;
  local_xy_util_.ToLocalXy(bestgnsspos->lat, bestgnsspos->lon, x, y);

  stamped_point.point = tf::Point(x, y, bestgnsspos->height);
  lap_checked_ = ui_.show_laps->isChecked();
  // The GPS "track" is in degrees, but createQuaternionFromYaw expects
  // radians.
  // Furthermore, the track rotates in the opposite direction and is also
  // offset by 90 degrees, so all of that has to be compensated for.
  //   stamped_point.orientation = tf::createQuaternionFromYaw((-gps->track * (M_PI / 180.0)) + M_PI_2);

  //   if (points_.empty() || (stamped_point.point.distance(points_.back().point)) >= (position_tolerance_))
  //   {
  //     points_.push_back(stamped_point);
  //   }

  //   if (buffer_size_ > 0)
  //   {
  //     while (static_cast<int>(points_.size()) > buffer_size_)
  //     {
  //       points_.pop_front();
  //     }
  //   }

  //   cur_point_ = stamped_point;
}

void BestgnssposPlugin::InspvaCallback(const novatel_gps_msgs::InspvaPtr& imu_inspva)
{
  stamped_point.orientation = tf::createQuaternionFromYaw((-imu_inspva->azimuth * (M_PI / 180.0)) + M_PI_2);

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
}

void BestgnssposPlugin::PrintError(const std::string& message)
{
  PrintErrorHelper(ui_.status, message);
}

void BestgnssposPlugin::PrintInfo(const std::string& message)
{
  PrintInfoHelper(ui_.status, message);
}

void BestgnssposPlugin::PrintWarning(const std::string& message)
{
  PrintWarningHelper(ui_.status, message);
}

QWidget* BestgnssposPlugin::GetConfigWidget(QWidget* parent)
{
  config_widget_->setParent(parent);

  return config_widget_;
}

bool BestgnssposPlugin::Initialize(QGLWidget* canvas)
{
  canvas_ = canvas;
  SetColor(ui_.color->color());

  return true;
}

void BestgnssposPlugin::Draw(double x, double y, double scale)
{
  if (DrawPoints(scale))
  {
    PrintInfo("OK");
  }
}

void BestgnssposPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
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

void BestgnssposPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
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
