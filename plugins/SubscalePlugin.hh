#ifndef GAZEBO_PLUGINS_SubscalePLUGIN_HH_
#define GAZEBO_PLUGINS_SubscalePLUGIN_HH_

#include <array>
#include <mutex>
#include <string>
#include <sdf/sdf.hh>
#include <ignition/transport/Node.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>

namespace gazebo
{
  class GZ_PLUGIN_VISIBLE SubscalePlugin : public ModelPlugin
  {
    public: SubscalePlugin();

    public: ~SubscalePlugin();

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    private: bool FindJoint(const std::string &_sdfParam,
        sdf::ElementPtr _sdf, physics::JointPtr &_joint);

    private: void Update(const common::UpdateInfo &_info);



    private: void PublishState();

    private: static const unsigned int kLeftAileron  = 0;
    private: static const unsigned int kLeftFlap     = 1;
    private: static const unsigned int kRightAileron = 2;
    private: static const unsigned int kRightFlap    = 3;
    private: static const unsigned int kElevators    = 4;
    private: static const unsigned int kRudder       = 5;


    private: event::ConnectionPtr updateConnection;

    private: transport::NodePtr node;

    private: transport::SubscriberPtr controlSub;

    private: transport::PublisherPtr statePub;

    private: physics::ModelPtr model;

    private: std::array<physics::JointPtr, 7> joints;



    /// \brief Next command to be applied to the propeller and control surfaces.
    private: std::array<float, 7> cmds;



    /// \brief keep track of controller update sim-time.
    private: gazebo::common::Time lastControllerUpdateTime;

    /// \brief Controller update mutex.
    private: std::mutex mutex;

    /// \brief Ignition node used for using Gazebo communications.
    private: ignition::transport::Node nodeIgn;

    private: ignition::transport::Node::Publisher statePubIgn;
  };
}
#endif
