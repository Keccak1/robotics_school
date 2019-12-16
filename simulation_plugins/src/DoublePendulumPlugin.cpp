#include "gazebo_plugins/DoublePendulumPlugin.h"

namespace gazebo
{

DoublePendulumPlugin::~DoublePendulumPlugin()
{

    if (this->callback_thread_.joinable())
    {
        this->callback_thread_.join();
    }
    if (this->pub_thread_.joinable())
    {
        this->pub_thread_.join();
    }
}

void DoublePendulumPlugin::Load(physics::ModelPtr model, sdf::ElementPtr element_ptr)
{
    this->model_ = model;
    InitRos();
    ROS_INFO_STREAM("Double Pendulum Plugin Loaded !");
}

void DoublePendulumPlugin::InitRos()
{
    InitRosNode();
    InitRosConection();
}
void DoublePendulumPlugin::InitRosConection()
{
    ros::SubscribeOptions ros_sub_options = ros::SubscribeOptions::create<simulations_messages::Vector3f>("/double_pendulum_force",
                                                                                             1,
                                                                                             boost::bind(&DoublePendulumPlugin::ForceCallback, this, _1),
                                                                                             ros::VoidPtr(), &this->callback_queue_);
    this->ros_sub_ = this->ros_node_->subscribe(ros_sub_options);
    this->callback_thread_ = std::thread(std::bind(&DoublePendulumPlugin::CallbackTrigger, this));

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&DoublePendulumPlugin::OnUpdate, this));

    this->pmq_.startServiceThread();
    this->pub_queue_ = this->pmq_.addPub<simulations_messages::DoublePendulumStates>();
    this->ros_pub_ = this->ros_node_->advertise<simulations_messages::DoublePendulumStates>("/double_pendulum_states", 100);
    this->pub_thread_ = std::thread(&DoublePendulumPlugin::StatePubliher, this);
}

simulations_messages::Pose DoublePendulumPlugin::Pose3dToPoseMsg(const ignition::math::Pose3d pose3d)
{
    simulations_messages::Pose link_position;
    link_position.position = Vector3toVector3Msg(pose3d.Pos());
    return link_position;
}

simulations_messages::Vector3f Vector3dtoVector3Msg(const ignition::math::Vector3<double> &vector3)
{
    simulations_messages::Vector3f vector;
    vector.x = vector3.X();
    vector.y = vector3.Y();
    vector.z = vector3.Z();
    return vector;
}


simulations_messages::CartState DoublePendulumPlugin::GetCartState()
{
    simulations_messages::CartState cart_state;

    cart_state.pose.position.x = this->model_->GetLink("Cart")->RelativePose().Pos().X();
    cart_state.pose.position.y = this->model_->GetLink("Cart")->RelativePose().Pos().Y();
    cart_state.pose.position.z = this->model_->GetLink("Cart")->RelativePose().Pos().Z();

    cart_state.linear_vel.x = this->model_->GetLink("Cart")->RelativeLinearVel().X();
    cart_state.linear_vel.y = this->model_->GetLink("Cart")->RelativeLinearVel().Y();
    cart_state.linear_vel.z = this->model_->GetLink("Cart")->RelativeLinearVel().Z();

    cart_state.linear_accel.x = this->model_->GetLink("Cart")->RelativeLinearAccel().X();
    cart_state.linear_accel.y = this->model_->GetLink("Cart")->RelativeLinearAccel().Y();
    cart_state.linear_accel.z = this->model_->GetLink("Cart")->RelativeLinearAccel().Z();

    return cart_state;
}
simulations_messages::FirstPole DoublePendulumPlugin::GetFirstPoleState()
{
    simulations_messages::FirstPole first_pole;

    first_pole.pose.position.x = this->model_->GetLink("FirstPole")->RelativePose().Pos().X();
    first_pole.pose.position.y = this->model_->GetLink("FirstPole")->RelativePose().Pos().Y();
    first_pole.pose.position.z = this->model_->GetLink("FirstPole")->RelativePose().Pos().Z();

    first_pole.pose.rotation.x = this->model_->GetLink("FirstPole")->RelativePose().Rot().X();
    first_pole.pose.rotation.y = this->model_->GetLink("FirstPole")->RelativePose().Rot().Y();
    first_pole.pose.rotation.z = this->model_->GetLink("FirstPole")->RelativePose().Rot().Z();
    first_pole.pose.rotation.z = this->model_->GetLink("FirstPole")->RelativePose().Rot().W();

    first_pole.angular_vel.x = this->model_->GetLink("FirstPole")->RelativeAngularVel().X();
    first_pole.angular_vel.y = this->model_->GetLink("FirstPole")->RelativeAngularVel().Y();
    first_pole.angular_vel.z = this->model_->GetLink("FirstPole")->RelativeAngularVel().Z();

    first_pole.angular_accel.x = this->model_->GetLink("FirstPole")->RelativeAngularAccel().X();
    first_pole.angular_accel.y = this->model_->GetLink("FirstPole")->RelativeAngularAccel().Y();
    first_pole.angular_accel.z = this->model_->GetLink("FirstPole")->RelativeAngularAccel().Z();

    return first_pole;
}
simulations_messages::SecondPole DoublePendulumPlugin ::GetSecondPoleState()
{
    simulations_messages::SecondPole second_pole;

    second_pole.pose.position.x = this->model_->GetLink("SecondPole")->RelativePose().Pos().X();
    second_pole.pose.position.y = this->model_->GetLink("SecondPole")->RelativePose().Pos().Y();
    second_pole.pose.position.z = this->model_->GetLink("SecondPole")->RelativePose().Pos().Z();

    second_pole.pose.rotation.x = this->model_->GetLink("SecondPole")->RelativePose().Rot().X();
    second_pole.pose.rotation.y = this->model_->GetLink("SecondPole")->RelativePose().Rot().Y();
    second_pole.pose.rotation.z = this->model_->GetLink("SecondPole")->RelativePose().Rot().Z();
    second_pole.pose.rotation.z = this->model_->GetLink("SecondPole")->RelativePose().Rot().W();
    
    second_pole.angular_vel.x = this->model_->GetLink("SecondPole")->RelativeAngularVel().X();
    second_pole.angular_vel.y = this->model_->GetLink("SecondPole")->RelativeAngularVel().Y();
    second_pole.angular_vel.z = this->model_->GetLink("SecondPole")->RelativeAngularVel().Z();
    
    second_pole.angular_accel.x = this->model_->GetLink("SecondPole")->RelativeAngularAccel().X();
    second_pole.angular_accel.y = this->model_->GetLink("SecondPole")->RelativeAngularAccel().Y();
    second_pole.angular_accel.z = this->model_->GetLink("SecondPole")->RelativeAngularAccel().Z();

    return second_pole;
}
simulations_messages::DoublePendulumStates DoublePendulumPlugin::GetStates()
{
    simulations_messages::DoublePendulumStates states;
    states.cart_state = GetCartState();
    states.first_pole = GetFirstPoleState();
    states.second_pole = GetSecondPoleState();
    return states;
}

void DoublePendulumPlugin::OnUpdate()
{
    this->model_->GetLink("Cart")->AddForce(this->force_);
}

ignition::math::Vector3<double> ForceMsgToVector3(const simulations_messages::Vector3f::ConstPtr &msg)
{
    return ignition::math::Vector3<double>(msg->x, msg->y, msg->z);
}

void DoublePendulumPlugin::ForceCallback(const simulations_messages::Vector3f::ConstPtr &msg)
{
    this->force_ = ForceMsgToVector3(msg);
}

void DoublePendulumPlugin::StatePubliher()
{
    while (this->ros_node_->ok())
    {
        ros::Rate loop_rate(10);
        simulations_messages::DoublePendulumStates states = GetStates();
        this->pub_queue_->push(states, this->ros_pub_);
        loop_rate.sleep();
    }
}

void DoublePendulumPlugin::CallbackTrigger()
{
    static const double timeout = 0.01;
    while (this->ros_node_->ok())
    {
        this->callback_queue_.callAvailable(ros::WallDuration(timeout));
    }
}

void DoublePendulumPlugin::InitRosNode()
{
    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "double_pendulum",
                  ros::init_options::NoSigintHandler);
    }

    this->ros_node_ = std::make_unique<ros::NodeHandle>();
}
} // namespace gazebo