#ifndef _ROBONALDO_SENSORS_HH_
#define _ROBONALDO_SENSORS_HH_
#define _USE_MATH_DEFINES

#include <cmath>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/plugins/ImuSensorPlugin.hh>



#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "robonaldo_msgs/imu_values.h"

namespace gazebo {
    class RobonaldoSensors : public ImuSensorPlugin{
        //default constructor
        public: RobonaldoSensors(){}
        public: virtual void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf){
            //construct magnetometer 
            magnetometer = _parent;

            // Initialize ros, if it has not already bee initialized.
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "imu",
                    ros::init_options::NoSigintHandler);
            }

            // Create our ROS node. This acts in a similar manner to
            // the Gazebo node
            ros::NodeHandle* n = new ros::NodeHandle("imu"); 
            this->rosNode.reset(n);

            imu_pub = n->advertise<robonaldo_msgs::imu_values>("imu", 1000);
        }
        protected: virtual void OnUpdate(sensors::ImuSensorPtr _sensor){
            //put sensor xyz and magnetometer xyz in ros message 
            robonaldo_msgs::imu_values msg;

            //Pose returns a pose3d object
            msg.mx = magnetometer->Pose().Pos()[0];   
            msg.my = magnetometer->Pose().Pos()[1];
            msg.mz = magnetometer->Pose().Pos()[2];
            
            //LinearAcceleration(const bool _noiseFree) returns a vector3d
            msg.ax = _sensor->LinearAcceleration(true)[0];  
            msg.ay = _sensor->LinearAcceleration(true)[1];  
            msg.az = _sensor->LinearAcceleration(true)[2];

            //AngularVelocity(const bool _noiseFree) returns a vector3d
            msg.gx = _sensor->AngularVelocity(true)[0];
            msg.gy = _sensor->AngularVelocity(true)[1];
            msg.gz = _sensor->AngularVelocity(true)[2];
            

            imu_pub.publish(msg);

        }
        
        /// \brief Pointer to the magnetometer
        private: sensors::SensorPtr magnetometer;

        /// \brief A ROS publisher
        private: ros::Publisher imu_pub;

        /// \brief A node use for ROS transport
        private: std::unique_ptr<ros::NodeHandle> rosNode;

    };
    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    // GZ_REGISTER_SENSOR_PLUGIN(RobonaldoSensors)

}
#endif