#ifndef _ROBONALDO_SENSORS_HH_
#define _ROBONALDO_SENSORS_HH_
#define _USE_MATH_DEFINES

#include <cmath>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/plugins/ImuSensorPlugin.hh>

#include <iostream>

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
        public: virtual void Load(sensors::SensorPtr mag, sensors::ImuSensorPtr imu, sdf::ElementPtr _sdf){
            //construct sensors 
            magnetometer_ = mag;
            imu_ = imu;

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
        protected: virtual void OnUpdate(){
            //put sensor xyz and magnetometer xyz in ros message 
            robonaldo_msgs::imu_values msg;

            //Pose returns a pose3d object
            msg.mx = magnetometer_->Pose().Pos()[0];   
            msg.my = magnetometer_->Pose().Pos()[1];
            msg.mz = magnetometer_->Pose().Pos()[2];
            
            //LinearAcceleration(const bool _noiseFree) returns a vector3d
            msg.ax = imu_->LinearAcceleration(true)[0];  
            msg.ay = imu_->LinearAcceleration(true)[1];  
            msg.az = imu_->LinearAcceleration(true)[2];

            //AngularVelocity(const bool _noiseFree) returns a vector3d
            msg.gx = imu_->AngularVelocity(true)[0];
            msg.gy = imu_->AngularVelocity(true)[1];
            msg.gz = imu_->AngularVelocity(true)[2];
            
            std::cout << "Sensor values:" << std::endl
                    << "\tMagnetometer: x=" << msg.mx << ", y=" << msg.my << ", z=" << msg.mz << std::endl
                    << "\tLinearAccel:  x=" << msg.ax << ", y=" << msg.ay << ", z=" << msg.az << std::endl
                    << "\tAngularVeloc: x=" << msg.gx << ", y=" << msg.gy << ", z=" << msg.gz << std::endl;
            imu_pub.publish(msg);

        }
        
        /// \brief Pointer to the magnetometer
        private: sensors::SensorPtr magnetometer_;
        private: sensors::ImuSensorPtr imu_;
        /// \brief A ROS publisher
        private: ros::Publisher imu_pub;

        /// \brief A node use for ROS transport
        private: std::unique_ptr<ros::NodeHandle> rosNode;

    };
    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    // GZ_REGISTER_SENSOR_PLUGIN(RobonaldoSensors)

}
#endif