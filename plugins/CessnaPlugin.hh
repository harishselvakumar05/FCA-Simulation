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
  /// <propeller>         Name of the joint controlling the propeller spin.
  /// <propeller_max_rpm> Maximum angular speed in rpm.
  /// <left_aileron>      Name of the joint controlling the left aileron.
  /// <left_flap>         Name of the joint controlling the left flap.
  /// <right_aileron>     Name of the joint controlling the right aileron.
  /// <right_flap>        Name of the joint controlling the right flap.
  /// <elevators>         Name of the joint controlling the rear elevators.
  /// <rudder>            Name of the joint controlling the rudder.
  ///
  /// The following parameters are optional:
  /// <propeller_p_gain> P gain for the PID that controls the propeller's speed.
  /// <propeller_i_gain> I gain for the PID that controls the propeller's speed.
  /// <propeller_d_gain> D gain for the PID that controls the propeller's speed.
  /// <surfaces_p_gain> P gain for the PID that controls the position of the
  ///                   control surfaces.
  /// <surfaces_i_gain> I gain for the PID that controls the position of the
  ///                   control surfaces.
  /// <surfaces_d_gain> D gain for the PID that controls the position of the
  ///                   control surfaces.
  ///
  /// The plugin will be subscribed to the following topic:
  /// "~/<model_name>/control" The expected value is a Subscale message.
  ///
  /// The plugin will advertise the following topic with the current state:
  /// "~/<model_name>/state"
  class GZ_PLUGIN_VISIBLE SubscalePlugin : public ModelPlugin
  {
    public: SubscalePlugin();

    public: ~SubscalePlugin();

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    private: bool FindJoint(const std::string &_sdfParam,
        sdf::ElementPtr _sdf, physics::JointPtr &_joint);

    private: void Update(const common::UpdateInfo &_info);


    private: void UpdatePIDs(double _dt);

    private: void PublishState();

    private: static const unsigned int kLeftAileron  = 0;
    private: static const unsigned int kLeftFlap     = 1;
    private: static const unsigned int kRightAileron = 2;
    private: static const unsigned int kRightFlap    = 3;
    private: static const unsigned int kElevators    = 4;
    private: static const unsigned int kRudder       = 5;
    private: static const unsigned int kPropeller    = 6;

    private: event::ConnectionPtr updateConnection;

    /// \brief Node used for using Gazebo communications.
    private: transport::NodePtr node;

    /// \brief Subscriber pointer.
    private: transport::SubscriberPtr controlSub;

    /// \brief Publisher pointer.
    private: transport::PublisherPtr statePub;

    /// \brief Pointer to the model;
    private: physics::ModelPtr model;

    /// \brief Control surfaces joints.
    private: std::array<physics::JointPtr, 7> joints;

    /// \brief Max propeller RPM.
    private: int32_t propellerMaxRpm = 2500;

    /// \brief Next command to be applied to the propeller and control surfaces.
    private: std::array<float, 7> cmds;

    /// \brief Velocity PID for the propeller.
    private: common::PID propellerPID;

    /// \brief Position PID for the control surfaces.
    private: std::array<common::PID, 6> controlSurfacesPID;

    /// \brief keep track of controller update sim-time.
    private: gazebo::common::Time lastControllerUpdateTime;

    /// \brief Controller update mutex.
    private: std::mutex mutex;

    // Place ignition::transport objects at the end of this file to
    // guarantee they are destructed first.

    /// \brief Ignition node used for using Gazebo communications.
    private: ignition::transport::Node nodeIgn;

    /// \brief Ignition Publisher.
    private: ignition::transport::Node::Publisher statePubIgn;
  };
}
#endif
