
#ifndef GAZEBO_PLUGINS_SUBSCALEGUIPLUGIN_HH_
#define GAZEBO_PLUGINS_SUBSCALEGUIPLUGIN_HH_

#include <mutex>

#include <ignition/math/Angle.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>


namespace gazebo
{
  /// Keyboard controls:
  /// w         Increase thrust (+10 %)
  /// s         Decrease thrust (-10 %)
  /// d         Increase rudder angle (+1 degree)
  /// a         Decrease rudder angle (-1 degree)
  /// Left-Key  Left roll (+1 degree)
  /// Right-Key Right roll (+1 degree)
  /// Up-Key    Pitch down (+1 degree)
  /// Down-Key  Pitch up (+1 degree)
  /// 1         Preset for take-off
  /// 2         Preset for cruise
  /// 3         Preset for landing
  class GZ_PLUGIN_VISIBLE SubscaleGUIPlugin : public GUIPlugin
  {
    Q_OBJECT

    public: SubscaleGUIPlugin();

    public: virtual ~SubscaleGUIPlugin();

    private: void OnState(ConstSubscalePtr &_msg);

    private slots: void OnIncreaseThrust();

    private slots: void OnDecreaseThrust();

    private slots: void OnIncreaseFlaps();

    private slots: void OnDecreaseFlaps();

    private slots: void OnIncreaseRoll();

    private slots: void OnDecreaseRoll();

    private slots: void OnIncreaseElevators();

    private slots: void OnDecreaseElevators();

    private slots: void OnIncreaseRudder();

    private slots: void OnDecreaseRudder();

    private slots: void OnPresetTakeOff();

    private slots: void OnPresetCruise();

    private slots: void OnPresetLanding();

    private: sdf::ElementPtr sdf;

    private: transport::NodePtr gzNode;

    private: transport::PublisherPtr controlPub;

    private: transport::SubscriberPtr stateSub;

    private: ignition::math::Angle angleStep;

    private: msgs::Subscale state;

    private: std::mutex mutex;
  };
}
#endif
