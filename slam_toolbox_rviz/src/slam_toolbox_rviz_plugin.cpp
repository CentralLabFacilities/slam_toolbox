/*
 * slam_toolbox
 * Copyright (c) 2018, Simbe Robotics, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Steven Macenski */

// Header
#include "slam_toolbox_rviz_plugin.h"
// QT
#include <QPushButton>
#include <QCheckBox>
#include <QLineEdit>
#include <QComboBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QtGui>
#include <QLabel>
#include <QFrame>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(slam_toolbox::SlamToolboxPlugin, rviz::Panel)

namespace slam_toolbox
{

/*****************************************************************************/
SlamToolboxPlugin::SlamToolboxPlugin(QWidget* parent):
    rviz::Panel(parent),
    _thread(NULL),
    _match_type(PROCESS_FIRST_NODE_CMT)
/*****************************************************************************/    
{
  ros::NodeHandle nh;
  bool paused_measure = false, interactive = false;
  nh.getParam("/slam_toolbox/paused_new_measurements", paused_measure);
  nh.getParam("/slam_toolbox/interactive_mode", interactive);
  _serialize = nh.serviceClient<slam_toolbox_msgs::SerializePoseGraph>("/slam_toolbox/serialize_map");
  _load_map = nh.serviceClient<slam_toolbox_msgs::DeserializePoseGraph>("/slam_toolbox/deserialize_map");
  _clearChanges = nh.serviceClient<slam_toolbox_msgs::Clear>("/slam_toolbox/clear_changes");
  _saveChanges = nh.serviceClient<slam_toolbox_msgs::LoopClosure>("/slam_toolbox/manual_loop_closure");
  _saveMap = nh.serviceClient<slam_toolbox_msgs::SaveMap>("/slam_toolbox/save_map");
  _clearQueue = nh.serviceClient<slam_toolbox_msgs::ClearQueue>("/slam_toolbox/clear_queue");
  _interactive = nh.serviceClient<slam_toolbox_msgs::ToggleInteractive>("/slam_toolbox/toggle_interactive_mode");
  _pause_measurements = nh.serviceClient<slam_toolbox_msgs::Pause>("/slam_toolbox/pause_new_measurements");
  _load_submap_for_merging = nh.serviceClient<slam_toolbox_msgs::AddSubmap>("/map_merging/add_submap");
  _merge = nh.serviceClient<slam_toolbox_msgs::MergeMaps>("/map_merging/merge_submaps");

  _vbox = new QVBoxLayout();
  _hbox1 = new QHBoxLayout();
  _hbox2 = new QHBoxLayout();
  _hbox3 = new QHBoxLayout();
  _hbox4 = new QHBoxLayout();
  _hbox5 = new QHBoxLayout();
  _hbox6 = new QHBoxLayout();
  _hbox7 = new QHBoxLayout();
  _hbox8 = new QHBoxLayout();
  _hbox9 = new QHBoxLayout();
  _hbox10 = new QHBoxLayout();



  _btn_clear = new QPushButton(this);
  _btn_clear->setText("Clear Changes");
  connect(_btn_clear, SIGNAL(clicked()), this, SLOT(ClearChanges()));
  _btn_save_changes = new QPushButton(this);
  _btn_save_changes->setText("Save Changes");
  connect(_btn_save_changes, SIGNAL(clicked()), this, SLOT(SaveChanges()));
  _btn_save_map = new QPushButton(this);
  _btn_save_map->setText("Save Map");
  connect(_btn_save_map, SIGNAL(clicked()), this, SLOT(SaveMap()));
  _btn_clear_measurement = new QPushButton(this);
  _btn_clear_measurement->setText("Clear Measurement Queue");
  connect(_btn_clear_measurement, SIGNAL(clicked()), this, SLOT(ClearQueue()));
  _btn_add_submap = new QPushButton(this);
  _btn_add_submap->setText("Add Submap");
  connect(_btn_add_submap, SIGNAL(clicked()), this, SLOT(LoadSubmap()));
  _btn_generate_map = new QPushButton(this);
  _btn_generate_map->setText("Generate Map");
  connect(_btn_generate_map, SIGNAL(clicked()), this, SLOT(GenerateMap()));
  _btn_serialize = new QPushButton(this);
  _btn_serialize->setText("Serialize Map");
  connect(_btn_serialize, SIGNAL(clicked()), this, SLOT(SerializeMap()));
  _btn_deserialize = new QPushButton(this);
  _btn_deserialize->setText("Deserialize Map");
  connect(_btn_deserialize, SIGNAL(clicked()), this, SLOT(DeserializeMap()));

  _is_interactive = new QCheckBox();
  _is_interactive->setChecked(interactive);
  connect(_is_interactive, SIGNAL(stateChanged(int)), this, SLOT(InteractiveCb(int)));
  _accept_measure = new QCheckBox();
  _accept_measure->setChecked(!paused_measure);
  connect(_accept_measure, SIGNAL(stateChanged(int)), this, SLOT(PauseMeasurementsCb(int)));
  _start_at_dock = new QRadioButton(tr("Start At Dock"));
  _start_at_dock->setChecked(true);
  _start_at_est = new QRadioButton(tr("Start At Pose Est."));
  _start_at_odom = new QRadioButton(tr("Start At Curr. Odom"));
  _localize = new QRadioButton(tr("Localize"));

  connect(_start_at_dock, SIGNAL(clicked()), this, SLOT(FirstNodeMatchCb()));
  connect(_start_at_est, SIGNAL(clicked()), this, SLOT(PoseEstMatchCb()));
  connect(_start_at_odom, SIGNAL(clicked()), this, SLOT(CurEstMatchCb()));
  connect(_localize, SIGNAL(clicked()), this, SLOT(LocalizeCb()));

  _map_name = new QLineEdit();
  _submap_name = new QLineEdit();
  _posegraph_filename = new QLineEdit();
  _initpose_x = new QDoubleSpinBox();
  _initpose_y = new QDoubleSpinBox();
  _initpose_theta = new QDoubleSpinBox();

  _btn_clear->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _btn_save_changes->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _btn_save_map->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _btn_clear_measurement->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _btn_add_submap->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _btn_generate_map->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _btn_serialize->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _btn_deserialize->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _is_interactive->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _accept_measure->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _map_name->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _submap_name->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _posegraph_filename->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  _hbox1->addWidget(_is_interactive);
  _hbox1->addWidget(new QLabel("Interactive Mode"));
  _hbox1->addWidget(_accept_measure);
  _hbox1->addWidget(new QLabel("Accept New Scans"));

  _hbox2->addWidget(_btn_clear);
  _hbox2->addWidget(_btn_save_changes);

  _hbox3->addWidget(_btn_save_map);
  _hbox3->addWidget(_map_name);

  _hbox4->addWidget(_btn_clear_measurement);

  _hbox5->addWidget(_btn_add_submap);
  _hbox5->addWidget(_submap_name);

  _hbox6->addWidget(_btn_generate_map);

  _hbox7->addWidget(new QLabel("filename:"));
  _hbox7->addWidget(_posegraph_filename);

  _hbox8->addWidget(_btn_deserialize);
  _hbox8->addWidget(_btn_serialize);

  _hbox9->addWidget(_start_at_dock);
  _hbox9->addWidget(_start_at_est);
  _hbox9->addWidget(_start_at_odom);
  _hbox9->addWidget(_localize);
  _hbox9->addStretch(1);

  _hbox10->addWidget(new QLabel("X"));
  _hbox10->addWidget(_initpose_x);
  _hbox10->addWidget(new QLabel("Y"));
  _hbox10->addWidget(_initpose_y);
  _hbox10->addWidget(new QLabel("θ"));
  _hbox10->addWidget(_initpose_theta);

  _vbox->addWidget(new QLabel("Create Map Tool"));
  _vbox->addLayout(_hbox1);
  _vbox->addLayout(_hbox2);
  _vbox->addLayout(_hbox3);
  _vbox->addLayout(_hbox7);
  _vbox->addLayout(_hbox8);
  _vbox->addLayout(_hbox9);
  _vbox->addLayout(_hbox10);
  _vbox->addLayout(_hbox4);

  QFrame* line = new QFrame();
  line->setFrameShape(QFrame::HLine);
  line->setFrameShadow(QFrame::Sunken);
  _vbox->addWidget(line);
  
  _vbox->addWidget(new QLabel("Merge Map Tool"));
  _vbox->addLayout(_hbox5);
  _vbox->addLayout(_hbox6);

  setLayout(_vbox);

  _thread = new std::thread(&SlamToolboxPlugin::updateCheckStateIfExternalChange, this);
}

/*****************************************************************************/
SlamToolboxPlugin::~SlamToolboxPlugin()
/*****************************************************************************/
{
  if (_thread)
  {
    delete _thread;
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::SerializeMap()
/*****************************************************************************/
{
  slam_toolbox_msgs::SerializePoseGraph msg;
  msg.request.filename = _posegraph_filename->text().toStdString();
  if (!_serialize.call(msg))
  {
    ROS_WARN("SlamToolbox: Failed to serialize pose graph to file, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::DeserializeMap()
/*****************************************************************************/
{
  typedef slam_toolbox_msgs::DeserializePoseGraph::Request procType;

  slam_toolbox_msgs::DeserializePoseGraph msg;
  msg.request.filename = _posegraph_filename->text().toStdString();
  if (_match_type == PROCESS_FIRST_NODE_CMT)
  {
    msg.request.match_type = procType::START_AT_FIRST_NODE;
  }
  else if (_match_type == PROCESS_NEAR_REGION_CMT)
  {
    msg.request.match_type = procType::START_AT_GIVEN_POSE;
    msg.request.initial_pose.x = _initpose_x->value();
    msg.request.initial_pose.y = _initpose_y->value();
    msg.request.initial_pose.theta = _initpose_theta->value();
  }
  else if (_match_type == LOCALIZE_CMT)
  {
    msg.request.match_type = procType::LOCALIZE_AT_POSE;
     msg.request.initial_pose.x = _initpose_x->value();
    msg.request.initial_pose.y = _initpose_y->value();
    msg.request.initial_pose.theta = _initpose_theta->value();
  }
  else
  {
    ROS_WARN("No match type selected, cannot send request.");
    return;
  }
  if (!_load_map.call(msg))
  {
     ROS_WARN("SlamToolbox: Failed to deserialize mapper object "
      "from file, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::LoadSubmap()
/*****************************************************************************/
{
  slam_toolbox_msgs::AddSubmap msg;
  msg.request.filename = _submap_name->text().toStdString();
  if (!_load_submap_for_merging.call(msg))
  {
    ROS_WARN("MergeMaps: Failed to load pose graph from file, is service running?");
  }
}
/*****************************************************************************/
void SlamToolboxPlugin::GenerateMap()
/*****************************************************************************/
{
  slam_toolbox_msgs::MergeMaps msg;
  if (!_merge.call(msg))
  {
    ROS_WARN("MergeMaps: Failed to merge maps, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::ClearChanges()
/*****************************************************************************/
{
  slam_toolbox_msgs::Clear msg;
  if (!_clearChanges.call(msg))
  {
    ROS_WARN("SlamToolbox: Failed to clear changes, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::SaveChanges()
/*****************************************************************************/
{
  slam_toolbox_msgs::LoopClosure msg;

  if (!_saveChanges.call(msg))
  {
    ROS_WARN("SlamToolbox: Failed to save changes, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::SaveMap()
/*****************************************************************************/
{
  slam_toolbox_msgs::SaveMap msg;
  msg.request.name.data = _map_name->text().toStdString();
  if (!_saveMap.call(msg))
  {
    ROS_WARN("SlamToolbox: Failed to save map as %s, is service running?",
              msg.request.name.data.c_str());
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::ClearQueue()
/*****************************************************************************/
{
  slam_toolbox_msgs::ClearQueue msg;
  if (!_clearQueue.call(msg))
  {
    ROS_WARN("Failed to clear queue, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::InteractiveCb(int state)
/*****************************************************************************/
{
  slam_toolbox_msgs::ToggleInteractive msg;
  if (!_interactive.call(msg))
  {
    ROS_WARN("SlamToolbox: Failed to toggle interactive mode, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::PauseMeasurementsCb(int state)
/*****************************************************************************/
{
  slam_toolbox_msgs::Pause msg;
  if (!_pause_measurements.call(msg))
  {
    ROS_WARN("SlamToolbox: Failed to toggle pause measurements, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::FirstNodeMatchCb()
/*****************************************************************************/
{
  if (_start_at_dock->isChecked() == Qt::Unchecked)
  {
    return;
  }
  else
  {
    _match_type = PROCESS_FIRST_NODE_CMT;
    ROS_INFO("Processing at first node selected.");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::PoseEstMatchCb()
/*****************************************************************************/
{
  if (_start_at_est->isChecked() == Qt::Unchecked)
  {
    return;
  }
  else
  {
    _match_type = PROCESS_NEAR_REGION_CMT;
    ROS_INFO("Processing at current pose estimate selected.");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::CurEstMatchCb()
/*****************************************************************************/
{
  if (_start_at_odom->isChecked() == Qt::Unchecked)
  {
    return;
  }
  else
  {
    _match_type = PROCESS_CMT;
    ROS_INFO("Processing at current odometry selected.");
  }
}


/*****************************************************************************/
void SlamToolboxPlugin::LocalizeCb()
/*****************************************************************************/
{
  if (_localize->isChecked() == Qt::Unchecked)
  {
    return;
  }
  else
  {
    _match_type = LOCALIZE_CMT;
    ROS_INFO("Processing localization selected.");
  }
}


/*****************************************************************************/
void SlamToolboxPlugin::updateCheckStateIfExternalChange()
/*****************************************************************************/
{
  ros::Rate r(1); //1 hz
  ros::NodeHandle nh;
  bool paused_measure = false, interactive = false;
  while (ros::ok())
  {
    nh.getParam("/slam_toolbox/paused_new_measurements", paused_measure);
    nh.getParam("/slam_toolbox/interactive_mode", interactive);

    bool oldState = _is_interactive->blockSignals(true);
    _is_interactive->setChecked(interactive);
    _is_interactive->blockSignals(oldState);

    oldState = _accept_measure->blockSignals(true);
    _accept_measure->setChecked(!paused_measure);
    _accept_measure->blockSignals(oldState);

    r.sleep();
  }
}

} // end namespace
