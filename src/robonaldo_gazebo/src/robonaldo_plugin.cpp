#ifndef _ROBONALDO_PLUGIN_HH_
#define _ROBONALDO_PLUGIN_HH_
#define _USE_MATH_DEFINES

#include <cmath>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/SensorManager.hh>
#include "gazebo/sensors/ImuSensor.hh" //yeet
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "robonaldo_msgs/motor_speeds.h"
#include "robonaldo_msgs/imu_values.h"

#include <memory>

const float max_speed = (5330.0f / 12.75f) * (M_PI * 0.2017776f) / 60.0f; //units: m/s 5330 is max speed of motor in rpm, div by 12.75 rot per sec

namespace gazebo
{
/// \brief A plugin to control the Robonaldo robot.
class RobonaldoPlugin : public ModelPlugin
{
  /// \brief Constructor
public:
  RobonaldoPlugin() {}

  /// \brief The load function is called by Gazebo when the plugin is
  /// inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is
  /// attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.

public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
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
    this->left_wheel = _model->GetJoints()[0];
    this->right_wheel = _model->GetJoints()[1];

    // Use same names as imu and magnetometer names in model.sdf file to get the imu and sensor data from model
    sensors::SensorManager *pMgr = sensors::SensorManager::Instance() ;
    this->imu = std::dynamic_pointer_cast<sensors::ImuSensor>(pMgr->GetSensor("imu"));
    this->magnetometer = pMgr->GetSensor("magnetometer");
    // this->magnetometer = std::dynamic_pointer_cast<sensors::MagnetometerSensor>(pMgr->GetSensor("magnetometer"));

    // SensorPlugin for updating and publishing sensor data
    // RobonaldoSensors sensorsPlugin;
    // sensorsPlugin.Load(this->magnetometer, this->imu, _sdf);
    // Add another joint

    // Setup a P-controller, with a gain of 0.1.
    this->pid = common::PID(0.5, 0, 0);

    // Apply the P-controller to the joint.
    this->model->GetJointController()->SetVelocityPID(
      this->left_wheel->GetScopedName(), this->pid);

    this->model->GetJointController()->SetVelocityPID(
      this->right_wheel->GetScopedName(), this->pid);


    // Set the joint's target velocity. This target velocity is just
    // for demonstration purposes.
    this->model->GetJointController()->SetVelocityTarget(
      this->left_wheel->GetScopedName(), 0.0);
    this->model->GetJointController()->SetVelocityTarget(
      this->right_wheel->GetScopedName(), 0.0);

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "robonaldo_gazebo",
                ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("robonaldo_gazebo"));

    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<robonaldo_msgs::motor_speeds>(
        "/motor_control",
        1,
        boost::bind(&RobonaldoPlugin::OnRosMsg, this, _1),
        ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);

    // Spin up the queue helper thread.
    this->rosQueueThread =
      std::thread(std::bind(&RobonaldoPlugin::QueueThread, this));
  }

  // \brief Set the velocity of the Velodyne
  /// \param[in] _vel New target velocity
public:
  void SetVelocity(const float &left_vel, const float &right_vel)
  {
    // Set the joint's target velocity.
    this->model->GetJointController()->SetVelocityTarget(
      this->left_wheel->GetScopedName(), left_vel);
    // Set the joint's target velocity.
    this->model->GetJointController()->SetVelocityTarget(
      this->right_wheel->GetScopedName(), right_vel);
  }

  /// \brief Handle an incoming message from ROS
  /// \param[in] _msg A float value that is used to set the velocity
  /// of the Velodyne.
public:
  void OnRosMsg(const robonaldo_msgs::motor_speeds::ConstPtr &_msg)
  {
    this->SetVelocity(max_speed * _msg->left_speed, max_speed * _msg->right_speed);
  }

  /// \brief ROS helper function that processes messages
private:
  void QueueThread()
  {
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }
private:
  void PublisherThread()
  {
    // do i need the following line? it is giving errors and i dontio understandio
    // anjali do be confusedio
    // imu_pub = rosNode->advertise<robonaldo_msgs::imu_values>("imu", 1000);
    while (this->rosNode->ok())
    {
      robonaldo_msgs::imu_values msg;

      //Pose returns a pose3d object
      msg.mx = this->magnetometer->Pose().Pos()[0];
      msg.my = this->magnetometer->Pose().Pos()[1];
      msg.mz = this->magnetometer->Pose().Pos()[2];

      //LinearAcceleration(const bool _noiseFree) returns a vector3d
      msg.ax = this->imu->LinearAcceleration(true)[0];
      msg.ay = this->imu->LinearAcceleration(true)[1];
      msg.az = this->imu->LinearAcceleration(true)[2];

      //AngularVelocity(const bool _noiseFree) returns a vector3d
      msg.gx = this->imu->AngularVelocity(true)[0];
      msg.gy = this->imu->AngularVelocity(true)[1];
      msg.gz = this->imu->AngularVelocity(true)[2];

      std::cout << "Imu sensor values:" << std::endl
                << "\tMagnetometer: x=" << msg.mx << ", y=" << msg.my << ", z=" << msg.mz << std::endl
                << "\tLinearAccel:  x=" << msg.ax << ", y=" << msg.ay << ", z=" << msg.az << std::endl
                << "\tAngularVeloc: x=" << msg.gx << ", y=" << msg.gy << ", z=" << msg.gz << std::endl;
      imu_pub.publish(msg);

    }
  }

  /// \brief Pointer to the model.
private:
  physics::ModelPtr model;

  /// \brief Pointer to the joint.
private:
  physics::JointPtr left_wheel;
private:
  physics::JointPtr right_wheel;

private:
  sensors::ImuSensorPtr imu;
private:
  sensors::SensorPtr magnetometer;

  /// \brief A PID controller for the joint.
private:
  common::PID pid;

  /// \brief A node use for ROS transport
private:
  std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief A ROS subscriber
private:
  ros::Subscriber rosSub;

  //ROS publisher
private:
  ros::Publisher imu_pub;


  /// \brief A ROS callbackqueue that helps process messages
private:
  ros::CallbackQueue rosQueue;

  /// \brief A thread the keeps running the rosQueue
private:
  std::thread rosQueueThread;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(RobonaldoPlugin)
}
#endif