// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "model/ModelInterface.h"
#include "model/RBDLModel.h"
#include "simulation/Sai2Simulation.h"
#include "redis/RedisClient.h"
#include <dynamics3d.h>
#include "timer/LoopTimer.h"
#include "force_sensor/ForceSensorSim.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;

const string world_file = "../resources/01-gravity_compensation/world.urdf";
const string robot_file = "../resources/01-gravity_compensation/4pbot_floating.urdf";
const string robot_name = "4PBOT";
const string camera_name = "camera_fixed";

// redis keys:
// - write:
const std::string JOINT_ANGLES_KEY  = "sai2::bracing2d::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::bracing2d::sensors::dq";
const std::string SIM_TIMESTAMP_KEY = "sai2::bracing2d::simulation::timestamp";

unsigned long long sim_counter = 0;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	HiredisServerInfo info;
	info.hostname_ = "127.0.0.1";
	info.port_ = 6379;
	info.timeout_ = { 1, 500000 }; // 1.5 seconds
	auto redis_client = CDatabaseRedisClient();
	redis_client.serverIs(info);

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, Simulation::urdf, false);

	sim->setCollisionRestitution(0);
	sim->setCoeffFrictionStatic(0.9);

	// load robots
	auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false);

	sim->setJointPosition(robot_name, 3, -90.0/180.0*M_PI);
	sim->setJointPosition(robot_name, 6, -90.0/180.0*M_PI);

	// create simulated force sensors
	Eigen::Affine3d fsensor1_location = Eigen::Affine3d::Identity();
	fsensor1_location.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
	Eigen::Affine3d fsensor2_location = Eigen::Affine3d::Identity();
	fsensor2_location.translation() = Eigen::Vector3d(0.0, 0.0, 1.0);
	auto fsensor1 = new ForceSensorSim(robot_name, "link0", fsensor1_location, robot);
	auto fsensor2 = new ForceSensorSim(robot_name, "link4", fsensor2_location, robot);


	Eigen::Vector3d world_gravity = sim->_world->getGravity().eigen();
	double g = -world_gravity(2);

	// create a loop timer
	double sim_freq = 5000;  // set the simulation frequency. Ideally 10kHz
	LoopTimer timer;
	timer.setLoopFrequency(sim_freq);   // 10 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	Eigen::VectorXd robot_torques(robot->dof());
	robot_torques(4) = -g;
	robot_torques(5) = -g;
	// robot_torques(3) = 7.5*g;
	// robot_torques(4) = 4*g;
	// robot_torques(5) = 1.5*g;
	// robot_torques(3) = 4.5*g;
	// robot_torques(4) = 2*g;
	// robot_torques(5) = 0.5*g;

	Eigen::Vector3d f1, f2, m1, m2;
	Eigen::Vector3d F1, F2;

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();

		sim->setJointTorques(robot_name, robot_torques);

		// update simulation by 1ms
		sim->integrate(1/sim_freq);

		// update kinematic models
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);

		robot->updateModel();

		// read force sensor values
		fsensor1->update(sim);
		fsensor1->getForce(f1);
		fsensor1->getMoment(m1);
		fsensor2->update(sim);
		fsensor2->getForce(f2);
		fsensor2->getMoment(m2);
		F1 << f1(1), f1(2), m1(0);
		F2 << f2(1), f2(2), m2(0);

		// write joint kinematics to redis
		redis_client.setEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q);
		redis_client.setEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);

		redis_client.setCommandIs(SIM_TIMESTAMP_KEY,std::to_string(timer.elapsedTime()));

		if(sim_counter % 500 == 0)
		{
			std::cout << "Force 1 : " << F1.transpose() << std::endl;
			std::cout << "Force 2 : " << F2.transpose() << std::endl;
			std::cout << "\n\n" << std::endl;
		}

		sim_counter++;

	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
