
#include <functional>
#include <string>
#include <sdf/sdf.hh>
#include <ignition/common/Profiler.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include "plugins/SubscalePlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(SubscalePlugin)

////////////////////////////////////////////////////////////////////////////////
SubscalePlugin::SubscalePlugin()
{
  this->cmds.fill(0.0f);

  // PID default parameters.
  this->propellerPID.Init(50.0, 0.1, 1, 0.0, 0.0, 20000.0, -20000.0);
  this->propellerPID.SetCmd(0.0);

  for (auto &pid : this->controlSurfacesPID)
  {
    pid.Init(50.0, 0.1, 1, 0.0, 0.0, 20.0, -20.0);
    pid.SetCmd(0.0);
  }
}

/////////////////////////////////////////////////
SubscalePlugin::~SubscalePlugin()
{
  this->updateConnection.reset();
}

/////////////////////////////////////////////////
bool SubscalePlugin::FindJoint(const std::string &_sdfParam, sdf::ElementPtr _sdf,
    physics::JointPtr &_joint)
{
  // Read the required plugin parameters.
  if (!_sdf->HasElement(_sdfParam))
  {
    gzerr << "Unable to find the <" << _sdfParam << "> parameter." << std::endl;
    return false;
  }

  std::string jointName = _sdf->Get<std::string>(_sdfParam);
  _joint = this->model->GetJoint(jointName);
  if (!_joint)
  {
    gzerr << "Failed to find joint [" << jointName
          << "] aborting plugin load." << std::endl;
    return false;
  }
  return true;
}

/////////////////////////////////////////////////
void SubscalePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "SubscalePlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "SubscalePlugin _sdf pointer is NULL");
  this->model = _model;

  // Read the required parameter for the propeller max RPMs.
  if (!_sdf->HasElement("propeller_max_rpm"))
  {
    gzerr << "Unable to find the <propeller_max_rpm> parameter." << std::endl;
    return;
  }
  this->propellerMaxRpm = _sdf->Get<int32_t>("propeller_max_rpm");
  if (this->propellerMaxRpm == 0)
  {
    gzerr << "Maximum propeller RPMs cannot be 0" << std::endl;
    return;
  }

  // Read the required joint name parameters.
  std::vector<std::string> requiredParams = {"left_aileron",
    "right_aileron", "elevators", "rudder", "propeller"};

  for (size_t i = 0; i < requiredParams.size(); ++i)
  {
    if (!this->FindJoint(requiredParams[i], _sdf, this->joints[i]))
      return;
  }

  // Overload the PID parameters if they are available.
  if (_sdf->HasElement("propeller_p_gain"))
    this->propellerPID.SetPGain(_sdf->Get<double>("propeller_p_gain"));

  if (_sdf->HasElement("propeller_i_gain"))
    this->propellerPID.SetIGain(_sdf->Get<double>("propeller_i_gain"));

  if (_sdf->HasElement("propeller_d_gain"))
    this->propellerPID.SetDGain(_sdf->Get<double>("propeller_d_gain"));

  if (_sdf->HasElement("surfaces_p_gain"))
  {
    for (auto &pid : this->controlSurfacesPID)
      pid.SetPGain(_sdf->Get<double>("surfaces_p_gain"));
  }

  if (_sdf->HasElement("surfaces_i_gain"))
  {
    for (auto &pid : this->controlSurfacesPID)
      pid.SetIGain(_sdf->Get<double>("surfaces_i_gain"));
  }

  if (_sdf->HasElement("surfaces_d_gain"))
  {
    for (auto &pid : this->controlSurfacesPID)
      pid.SetDGain(_sdf->Get<double>("surfaces_d_gain"));
  }

  // Controller time control.
  this->lastControllerUpdateTime = this->model->GetWorld()->SimTime();

  // Listen to the update event. This event is broadcast every simulation
  // iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&SubscalePlugin::Update, this, std::placeholders::_1));

  // Initialize transport.
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  std::string prefix = "~/" + this->model->GetName() + "/";
  this->statePub = this->node->Advertise<msgs::Subscale>(prefix + "state");
  this->controlSub = this->node->Subscribe(prefix + "control",
    &SubscalePlugin::OnControl, this);

  gzlog << "Subscale ready to fly. The force will be with you" << std::endl;
}

/////////////////////////////////////////////////
void SubscalePlugin::Update(const common::UpdateInfo &/*_info*/)
{
  IGN_PROFILE("SubscalePlugin::OnUpdate");
  std::lock_guard<std::mutex> lock(this->mutex);

  gazebo::common::Time curTime = this->model->GetWorld()->SimTime();

  if (curTime > this->lastControllerUpdateTime)
  {
    // Update the control surfaces and publish the new state.
    IGN_PROFILE_BEGIN("Update");
    this->UpdatePIDs((curTime - this->lastControllerUpdateTime).Double());
    IGN_PROFILE_END();
    IGN_PROFILE_BEGIN("Publish");
    this->PublishState();
    IGN_PROFILE_END();

    this->lastControllerUpdateTime = curTime;
  }
}

/////////////////////////////////////////////////
void SubscalePlugin::OnControl(ConstSubscalePtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  if (_msg->has_cmd_propeller_speed() &&
      std::abs(_msg->cmd_propeller_speed()) <= 1)
  {
    this->cmds[kPropeller] = _msg->cmd_propeller_speed();
  }
  if (_msg->has_cmd_left_aileron())
    this->cmds[kLeftAileron] = _msg->cmd_left_aileron();

  if (_msg->has_cmd_right_aileron())
    this->cmds[kRightAileron] = _msg->cmd_right_aileron();

  if (_msg->has_cmd_elevators())
    this->cmds[kElevators] = _msg->cmd_elevators();
  if (_msg->has_cmd_rudder())
    this->cmds[kRudder] = _msg->cmd_rudder();
}

/////////////////////////////////////////////////
void SubscalePlugin::UpdatePIDs(double _dt)
{
  // Velocity PID for the propeller.
  double vel = this->joints[kPropeller]->GetVelocity(0);
  double maxVel = this->propellerMaxRpm*2.0*M_PI/60.0;
  double target = maxVel * this->cmds[kPropeller];
  double error = vel - target;
  double force = this->propellerPID.Update(error, _dt);
  this->joints[kPropeller]->SetForce(0, force);

  // Position PID for the control surfaces.
  for (size_t i = 0; i < this->controlSurfacesPID.size(); ++i)
  {
    double pos = this->joints[i]->Position(0);
    error = pos - this->cmds[i];
    force = this->controlSurfacesPID[i].Update(error, _dt);
    this->joints[i]->SetForce(0, force);
  }
}

/////////////////////////////////////////////////
void SubscalePlugin::PublishState()
{
  // Read the current state.
  double propellerRpms = this->joints[kPropeller]->GetVelocity(0)
    /(2.0*M_PI)*60.0;
  float propellerSpeed = propellerRpms / this->propellerMaxRpm;
  float leftAileron = this->joints[kLeftAileron]->Position(0);
  float rightAileron = this->joints[kRightAileron]->Position(0);
  float elevators = this->joints[kElevators]->Position(0);
  float rudder = this->joints[kRudder]->Position(0);

  msgs::Subscale msg;
  // Set the observed state.
  msg.set_propeller_speed(propellerSpeed);
  msg.set_left_aileron(leftAileron);
  msg.set_right_aileron(rightAileron);
  msg.set_elevators(elevators);
  msg.set_rudder(rudder);

  // Set the target state.
  msg.set_cmd_propeller_speed(this->cmds[kPropeller]);
  msg.set_cmd_left_aileron(this->cmds[kLeftAileron]);
  msg.set_cmd_right_aileron(this->cmds[kRightAileron]);
  msg.set_cmd_elevators(this->cmds[kElevators]);
  msg.set_cmd_rudder(this->cmds[kRudder]);

  this->statePub->Publish(msg);
}
