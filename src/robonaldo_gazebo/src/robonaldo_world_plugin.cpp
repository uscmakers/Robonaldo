#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "ros/ros.h"


namespace gazebo {

class RobonaldoWorldPlugin : public WorldPlugin {
	public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/) {
		if (_parent->ModelCount() == 0) {
		    std::cerr << "Invalid model count, Robonaldo World plugin not loaded\n";
        return;
		}

		this->world = _parent;
		this->robo = world->ModelByName("main_robonaldo");
		this->ball = world->ModelByName("ball");

		// TODO: Get location configs to figure out ball location

		// TODO: Publish to ROS node
		// Initialize ros, if it has not already bee initialized.
		if (!ros::isInitialized())
		{
		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, "gazebo_client",  // TODO: change this node name
		    ros::init_options::NoSigintHandler);
		}

		// Create our ROS node. This acts in a similar manner to the Gazebo node.
		


	}
	private: physics::WorldPtr world;
	private: physics::ModelPtr robo;
	private: physics::ModelPtr ball;

};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(RobonaldoWorldPlugin)
}