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


  gzlog << "Subscale ready to fly. The force will be with you" << std::endl;
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

}
