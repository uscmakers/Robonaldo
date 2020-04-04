#ifndef _ROBONALDO_PLUGIN_HH_
#define _ROBONALDO_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  /// \brief A plugin to control the Robonaldo robot.
  class RobonaldoPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: RobonaldoPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Just output a message for now
      std::cerr << "\nThe robonaldo plugin is attached to model[" <<
        _model->GetName() << "]\n";

      // Safety check
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, Robonaldo plugin not loaded\n";
        return;
      }

      // Store the model pointer for convenience.
      this->model = _model;

      // Get the first joint. We are making an assumption about the model
      // having one joint that is the rotational joint.
      this->joint0 = _model->GetJoints()[0];
      this->joint1 = _model->GetJoints()[1];

      // Add another joint

      // Setup a P-controller, with a gain of 0.1.
      this->pid = common::PID(0.1, 0, 0);

      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetVelocityPID(
          this->joint0->GetScopedName(), this->pid);

      this->model->GetJointController()->SetVelocityPID(
          this->joint1->GetScopedName(), this->pid);

      // Set the joint's target velocity. This target velocity is just
      // for demonstration purposes.
      this->model->GetJointController()->SetVelocityTarget(
          this->joint0->GetScopedName(), 0.0);
      this->model->GetJointController()->SetVelocityTarget(
          this->joint1->GetScopedName(), 0.0);
      
      // Create the node
      this->node = transport::NodePtr(new transport::Node());
      #if GAZEBO_MAJOR_VERSION < 8
      this->node->Init(this->model->GetWorld()->GetName());
      #else
      this->node->Init(this->model->GetWorld()->Name());
      #endif

      // Create a topic name
      std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";
      std::cout << "topic Name is " << topicName << std::endl;

      // Subscribe to the topic, and register a callback
      this->sub = this->node->Subscribe(topicName,
         &RobonaldoPlugin::OnMsg, this);
    }

    // \brief Set the velocity of the Velodyne
    /// \param[in] _vel New target velocity
    public: void SetVelocity(const double &_vel)
    {
      // Set the joint's target velocity.
      this->model->GetJointController()->SetVelocityTarget(
          this->joint0->GetScopedName(), _vel);
    }

    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    private: void OnMsg(ConstVector3dPtr &_msg)
    {
      std::cout << "OnMsg" << std::endl;
      this->SetVelocity(_msg->x());
    }

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint0;
    private: physics::JointPtr joint1;

    /// \brief A PID controller for the joint.
    private: common::PID pid;

    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(RobonaldoPlugin)
}
#endif