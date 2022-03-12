#include <functional>
#include <string>
#include <sdf/sdf.hh>
#include <ignition/common/Profiler.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include "SubscalePlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(SubscalePlugin)

SubscalePlugin::SubscalePlugin()
{
  this->cmds.fill(0.0f);


}


SubscalePlugin::~SubscalePlugin()
{
  this->updateConnection.reset();
}


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

void SubscalePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "SubscalePlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "SubscalePlugin _sdf pointer is NULL");
  this->model = _model;


  // Read the required joint name parameters.
  std::vector<std::string> requiredParams = {"left_aileron",
    "right_aileron", "rudder"};

  for (size_t i = 0; i < requiredParams.size(); ++i)
  {
    if (!this->FindJoint(requiredParams[i], _sdf, this->joints[i]))
      return;
  }


  this->lastControllerUpdateTime = this->model->GetWorld()->SimTime();


  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&SubscalePlugin::Update, this, std::placeholders::_1));


  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  std::string prefix = "~/" + this->model->GetName() + "/";
  this->statePub = this->node->Advertise<msgs::Cessna>(prefix + "state");
  this->controlSub = this->node->Subscribe(prefix + "control",
    &CessnaPlugin::OnControl, this);
}
void CessnaPlugin::OnControl(ConstCessnaPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  if (_msg->has_cmd_left_aileron())
    this->cmds[kLeftAileron] = _msg->cmd_left_aileron();
  if (_msg->has_cmd_right_aileron())
    this->cmds[kRightAileron] = _msg->cmd_right_aileron();
  if (_msg->has_cmd_elevators())
    this->cmds[kElevators] = _msg->cmd_elevators();
  if (_msg->has_cmd_rudder())
    this->cmds[kRudder] = _msg->cmd_rudder();
}
void SubscalePlugin::Update(const common::UpdateInfo &/*_info*/)
{
  IGN_PROFILE("SubscalePlugin::OnUpdate");
  std::lock_guard<std::mutex> lock(this->mutex);

  gazebo::common::Time curTime = this->model->GetWorld()->SimTime();

  if (curTime > this->lastControllerUpdateTime)
  {
    // Update the control surfaces and publish the new state.
    IGN_PROFILE_BEGIN("Update");
    IGN_PROFILE_END();
    IGN_PROFILE_BEGIN("Publish");
    this->PublishState();
    IGN_PROFILE_END();

    this->lastControllerUpdateTime = curTime;
  }
}

void SubscalePlugin::PublishState()
{

  float leftAileron = this->joints[kLeftAileron]->Position(0);
  float rightAileron = this->joints[kRightAileron]->Position(0);
  float elevators = this->joints[kElevators]->Position(0);
  float rudder = this->joints[kRudder]->Position(0);
  
    msgs::Cessna msg;
  // Set the observed state.
  msg.set_propeller_speed(propellerSpeed);
  msg.set_left_aileron(leftAileron);
  msg.set_left_flap(leftFlap);
  msg.set_right_aileron(rightAileron);
  msg.set_right_flap(rightFlap);
  msg.set_elevators(elevators);
  msg.set_rudder(rudder);

  // Set the target state.
  msg.set_cmd_propeller_speed(this->cmds[kPropeller]);
  msg.set_cmd_left_aileron(this->cmds[kLeftAileron]);
  msg.set_cmd_left_flap(this->cmds[kLeftFlap]);
  msg.set_cmd_right_aileron(this->cmds[kRightAileron]);
  msg.set_cmd_right_flap(this->cmds[kRightFlap]);
  msg.set_cmd_elevators(this->cmds[kElevators]);
  msg.set_cmd_rudder(this->cmds[kRudder]);

  this->statePub->Publish(msg);

}
