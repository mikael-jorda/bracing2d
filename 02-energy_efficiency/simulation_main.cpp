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

const string world_file = "../resources/02-energy_efficiency/world.urdf";
const string robot_file = "../resources/02-energy_efficiency/4pbot_fixed.urdf";
const string robot_name = "4PBOT";

// redis keys:
// - write:
const std::string JOINT_ANGLES_KEY  = "sai2::bracing2d::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::bracing2d::sensors::dq";
const std::string SIM_TIMESTAMP_KEY = "sai2::bracing2d::simulation::timestamp";
// - read
const std::string CONTROLLER_RUNNING = "sai2::bracing2d::controller_running";
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::bracing2d::actuators::fgc";

unsigned long long sim_counter = 0;
double controller_running = 0.0;

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

	sim->setJointPosition(robot_name, 0, -60.0/180.0*M_PI);
	sim->setJointPosition(robot_name, 1, -30.0/180.0*M_PI);
	sim->setJointPosition(robot_name, 2, -30.0/180.0*M_PI);

	// create simulated force sensors
	Eigen::Affine3d fsensor_location = Eigen::Affine3d::Identity();
	fsensor_location.translation() = Eigen::Vector3d(0.0, 0.0, 1.0);
	auto fsensor = new ForceSensorSim(robot_name, "link4", fsensor_location, robot);


	Eigen::Vector3d world_gravity = sim->_world->getGravity().eigen();
	double g = -world_gravity(2);
	Eigen::VectorXd gravity_compensation;

	// create a loop timer
	double sim_freq = 5000;  // set the simulation frequency. Ideally 10kHz
	LoopTimer timer;
	timer.setLoopFrequency(sim_freq);   // 10 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	Eigen::VectorXd robot_torques(robot->dof());

	Eigen::Vector3d f, m;
	Eigen::Vector3d F;

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();

		redis_client.getCommandIs(CONTROLLER_RUNNING, controller_running);
		if(controller_running != 0)
		{
			redis_client.getEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, robot_torques);
		}
		else
		{
			robot->gravityVector(gravity_compensation, world_gravity);
			robot_torques = gravity_compensation;
		}


		sim->setJointTorques(robot_name, robot_torques);

		// update simulation by 1ms
		sim->integrate(1/sim_freq);

		// update kinematic models
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);

		robot->updateModel();

		// read force sensor values
		fsensor->update(sim);
		fsensor->getForce(f);
		fsensor->getMoment(m);
		F << f(1), f(2), m(0);

		// write joint kinematics to redis
		redis_client.setEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q);
		redis_client.setEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);

		redis_client.setCommandIs(SIM_TIMESTAMP_KEY,std::to_string(timer.elapsedTime()));

		// if(sim_counter % 500 == 0)
		// {
		// 	std::cout << "Force : " << F.transpose() << std::endl;
		// 	std::cout << "\n\n" << std::endl;
		// }

		sim_counter++;

	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
