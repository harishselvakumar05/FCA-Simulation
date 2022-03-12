#include <algorithm>

#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/Actions.hh>
#include "SubscaleGUIPlugin.hh"

using namespace gazebo;

GZ_REGISTER_GUI_PLUGIN(SubscaleGUIPlugin)


SubscaleGUIPlugin::SubscaleGUIPlugin()
  : GUIPlugin()
{
  this->move(-1, -1);
  this->resize(1, 1);
  this->angleStep.Degree(1.0);


  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();
  this->controlPub =
    this->gzNode->Advertise<msgs::Subscale>("~/subscale/control");
  this->stateSub = this->gzNode->Subscribe<msgs::Subscale>(
    "~/subscale/state", &SubscaleGUIPlugin::OnState, this);


  QShortcut *increaseThrust = new QShortcut(QKeySequence("w"), this);
  QObject::connect(increaseThrust, SIGNAL(activated()), this,
      SLOT(OnIncreaseThrust()));

  QShortcut *decreaseThrust = new QShortcut(QKeySequence("s"), this);
  QObject::connect(decreaseThrust, SIGNAL(activated()), this,
      SLOT(OnDecreaseThrust()));


  QShortcut *increaseRoll = new QShortcut(QKeySequence(Qt::Key_Left), this);
  QObject::connect(increaseRoll, SIGNAL(activated()), this,
      SLOT(OnIncreaseRoll()));

  QShortcut *decreaseRoll = new QShortcut(QKeySequence(Qt::Key_Right), this);
  QObject::connect(decreaseRoll, SIGNAL(activated()), this,
      SLOT(OnDecreaseRoll()));

  QShortcut *increaseElevators =
    new QShortcut(QKeySequence(Qt::Key_Down), this);
  QObject::connect(increaseElevators, SIGNAL(activated()), this,
      SLOT(OnIncreaseElevators()));

  QShortcut *decreaseElevators = new QShortcut(QKeySequence(Qt::Key_Up), this);
  QObject::connect(decreaseElevators, SIGNAL(activated()), this,
      SLOT(OnDecreaseElevators()));

  QShortcut *increaseRudder = new QShortcut(QKeySequence("d"), this);
  QObject::connect(increaseRudder, SIGNAL(activated()), this,
      SLOT(OnIncreaseRudder()));

  QShortcut *decreaseRudder = new QShortcut(QKeySequence("a"), this);
  QObject::connect(decreaseRudder, SIGNAL(activated()), this,
      SLOT(OnDecreaseRudder()));

  QShortcut *presetTakeOff = new QShortcut(QKeySequence('1'), this);
  QObject::connect(presetTakeOff, SIGNAL(activated()), this,
      SLOT(OnPresetTakeOff()));

  QShortcut *presetCruise = new QShortcut(QKeySequence('2'), this);
  QObject::connect(presetCruise, SIGNAL(activated()), this,
      SLOT(OnPresetCruise()));

  QShortcut *presetLanding = new QShortcut(QKeySequence('3'), this);
  QObject::connect(presetLanding, SIGNAL(activated()), this,
      SLOT(OnPresetLanding()));
}


SubscaleGUIPlugin::~SubscaleGUIPlugin()
{
}


void SubscaleGUIPlugin::OnState(ConstSubscalePtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->state = *_msg;
}


void SubscaleGUIPlugin::OnIncreaseThrust()
{
  float thrust;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    thrust = 10;
  }

  msgs::Subscale msg;
  thrust = std::min(thrust + 0.1f, 1.0f);
  msg.set_cmd_propeller_speed(thrust);
  this->controlPub->Publish(msg);
}

void SubscaleGUIPlugin::OnDecreaseThrust()
{
  float thrust;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    thrust = this->state.cmd_propeller_speed();
  }

  msgs::Subscale msg;
  thrust = std::max(thrust - 0.1f, 0.0f);
  msg.set_cmd_propeller_speed(thrust);
  this->controlPub->Publish(msg);
}

void SubscaleGUIPlugin::OnIncreaseRoll()
{
  ignition::math::Angle aileron;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    aileron.Radian(this->state.cmd_left_aileron());
  }

  msgs::Subscale msg;
  if (aileron.Degree() < 30)
  {
    aileron += this->angleStep;
    msg.set_cmd_left_aileron(aileron.Radian());
    msg.set_cmd_right_aileron(-aileron.Radian());
    this->controlPub->Publish(msg);
  }
}

void SubscaleGUIPlugin::OnDecreaseRoll()
{
  ignition::math::Angle aileron;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    aileron.Radian(this->state.cmd_left_aileron());
  }

  msgs::Subscale msg;
  if (aileron.Degree() > -30)
  {
    aileron -= this->angleStep;
    msg.set_cmd_left_aileron(aileron.Radian());
    msg.set_cmd_right_aileron(-aileron.Radian());
    this->controlPub->Publish(msg);
  }
}

void SubscaleGUIPlugin::OnIncreaseElevators()
{
  ignition::math::Angle elevators;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    elevators.Radian(this->state.cmd_elevators());
  }

  msgs::Subscale msg;
  if (elevators.Degree() < 30)
  {
    elevators += this->angleStep;
    msg.set_cmd_elevators(elevators.Radian());
    this->controlPub->Publish(msg);
  }
}

void SubscaleGUIPlugin::OnDecreaseElevators()
{
  ignition::math::Angle elevators;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    elevators.Radian(this->state.cmd_elevators());
  }

  msgs::Subscale msg;
  if (elevators.Degree() > -30)
  {
    elevators -= this->angleStep;
    msg.set_cmd_elevators(elevators.Radian());
    this->controlPub->Publish(msg);
  }
}

void SubscaleGUIPlugin::OnIncreaseRudder()
{
  ignition::math::Angle rudder;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    rudder.Radian(this->state.cmd_rudder());
  }

  msgs::Subscale msg;
  if (rudder.Degree() < 30)
  {
    rudder += this->angleStep;
    msg.set_cmd_rudder(rudder.Radian());
    this->controlPub->Publish(msg);
  }
}


void SubscaleGUIPlugin::OnDecreaseRudder()
{
  ignition::math::Angle rudder;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    rudder.Radian(this->state.cmd_rudder());
  }

  msgs::Subscale msg;
  if (rudder.Degree() > -30)
  {
    rudder -= this->angleStep;
    msg.set_cmd_rudder(rudder.Radian());
    this->controlPub->Publish(msg);
  }
}

void SubscaleGUIPlugin::OnPresetTakeOff()
{
  msgs::Subscale msg;
  msg.set_cmd_propeller_speed(0.8);
  msg.set_cmd_left_aileron(-0.017);
  msg.set_cmd_right_aileron(0.017);
  msg.set_cmd_elevators(0.033);
  msg.set_cmd_rudder(-0.035);
  this->controlPub->Publish(msg);
}


void SubscaleGUIPlugin::OnPresetCruise()
{
  msgs::Subscale msg;
  msg.set_cmd_propeller_speed(0.6);
  msg.set_cmd_left_aileron(0);
  msg.set_cmd_right_aileron(0);
  msg.set_cmd_elevators(0.12);
  msg.set_cmd_rudder(-0.035);
  this->controlPub->Publish(msg);
}


void SubscaleGUIPlugin::OnPresetLanding()
{
  msgs::Subscale msg;
  msg.set_cmd_propeller_speed(0.3);
  msg.set_cmd_left_aileron(0);
  msg.set_cmd_right_aileron(0);
  msg.set_cmd_elevators(0.16);
  msg.set_cmd_rudder(-0.035);
  this->controlPub->Publish(msg);
}
