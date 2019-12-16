#ifndef GAZEBO_PLUGINS_DOUBLE_PENDULUM_PLUGIN
#define GAZEBO_PLUGINS_DOUBLE_PENDULUM_PLUGIN


#include <functional>
#include <memory>
#include <thread>

#include <simulations_messages/Vector3f.h>
#include <simulations_messages/DoublePendulumStates.h>

#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo_plugins/PubQueue.h>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <atomic>
namespace gazebo
{
class DoublePendulumPlugin : public ModelPlugin
{
public:
  DoublePendulumPlugin()
      : ModelPlugin()
  {
  }
  ~DoublePendulumPlugin();
  

  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

private:

  void InitRos();            
  void InitRosNode();        
  void InitRosConection();

  void CallbackTrigger();
  void ForceCallback(const simulations_messages::Vector3f::ConstPtr& msg);
  void StatePubliher();
  void OnUpdate();
  physics::LinkPtr GetLink(const std::string & link_name);

  simulations_messages::Pose Pose3dToPoseMsg(const ignition::math::Pose3d pose3d);
  simulations_messages::Vector3f Vector3toVector3Msg(const ignition::math::Vector3<double> &vector3);
  simulations_messages::CartState GetCartState();
  simulations_messages::FirstPole GetFirstPoleState();
  simulations_messages::SecondPole GetSecondPoleState();
  simulations_messages::DoublePendulumStates GetStates();

  ignition::math::Vector3<double> force_;
  std::map<std::string, physics::LinkPtr> links_;
  physics::ModelPtr model_;        
  std::thread callback_thread_;
  std::thread pub_thread_;
  event::ConnectionPtr update_connection_;

  ros::Publisher ros_pub_;
  ros::Subscriber ros_sub_;
  PubMultiQueue pmq_;
  PubQueue<simulations_messages::DoublePendulumStates>::Ptr pub_queue_;
  ros::CallbackQueue callback_queue_;
  std::unique_ptr<ros::NodeHandle> ros_node_;
  transport::NodePtr node_;

};

GZ_REGISTER_MODEL_PLUGIN(DoublePendulumPlugin)
} // namespace gazebo

#endif