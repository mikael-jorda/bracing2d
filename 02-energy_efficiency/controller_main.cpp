// This example application runs a controller for the IIWA

#include "model/ModelInterface.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void stop(int){runloop = false;}

using namespace std;

const string robot_file = "../resources/02-energy_efficiency/4pbot_fixed.urdf";

// redis keys:
// - write:
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::bracing2d::actuators::fgc";
const std::string CONTROLLER_RUNNING = "sai2::bracing2d::controller_running";
// - read:
const std::string JOINT_ANGLES_KEY  = "sai2::bracing2d::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::bracing2d::sensors::dq";

unsigned long long controller_counter = 0;

bool gpjs = true;


 void sighandler(int sig)
 { runloop = false; }

int main() {
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

	// load robots
	auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false);

	// read from Redis
	redis_client.getEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q);
	redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);

	////////////////////////////////////////////////
	///    Prepare the different controllers   /////
	////////////////////////////////////////////////
	robot->updateModel();

	int dof = robot->dof();
	Eigen::VectorXd command_torques = Eigen::VectorXd::Zero(dof);
	Eigen::VectorXd gravity_compensation;

	// joint task
	Eigen::VectorXd joint_task_torques = Eigen::VectorXd::Zero(dof);
	double kp_joint = 0.0;
	double kv_joint = 10.0;

	// position task
	const string link_name = "link4";
	const Eigen::Vector3d pos_in_link = Eigen::Vector3d(0.0, 0.0, 1.0);
	Eigen::MatrixXd J3d, Jv2d, Jw2d, J2d, Lambda, N, Jbar;
	J3d = Eigen::MatrixXd::Zero(6, dof);
	Jw2d = Eigen::MatrixXd::Zero(1, dof);
	Jv2d = Eigen::MatrixXd::Zero(2, dof);
	J2d = Eigen::MatrixXd::Zero(3, dof);
	Lambda = Eigen::MatrixXd(2,2);
	Jbar = Eigen::MatrixXd(dof,2);
	N = Eigen::MatrixXd(dof,dof);

	Eigen::Vector2d pos_task_force;
	Eigen::VectorXd pos_task_torques;

	double kp_pos = 100.0;
	double kv_pos = 20.0;

	Eigen::Vector3d pos3d;
	Eigen::Vector2d x, xdot;
	Eigen::Vector2d xd = Eigen::Vector2d(3.0, 1.0);

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	Eigen::Vector3d sensed_force = Eigen::Vector3d::Zero();

	cout << "Controller starting" << endl;
	redis_client.setCommandIs(CONTROLLER_RUNNING, "1");

	// gpjs = false;

	// while window is open:
	while (runloop) {

		// wait for next scheduled loop
		timer.waitForNextLoop();

		// read from Redis
		redis_client.getEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q);
		redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);

		// update the model 20 times slower
		if(controller_counter%20 == 0)
		{
			robot->updateModel();

			robot->J_0(J3d, link_name, pos_in_link);
			Jv2d = J3d.block(1,0,2,dof);
			robot->operationalSpaceMatrices(Lambda, Jbar, N, Jv2d);
		}

		////////////////////////////// Compute joint torques
		double time = controller_counter/control_freq;

		robot->gravityVector(gravity_compensation);

		//----------- pos task
		robot->position(pos3d, link_name, pos_in_link);
		x = pos3d.tail(2);
		xdot = Jv2d*robot->_dq;

		pos_task_force = Lambda*(-kp_pos*(x - xd) - kv_pos*xdot);
		if(!gpjs)
		{
			pos_task_force += Jbar.transpose() * gravity_compensation;
		}
		pos_task_torques = Jv2d.transpose() * pos_task_force;

		//----- Joint nullspace damping
		joint_task_torques = robot->_M*( - kv_joint*robot->_dq);

		//------ Final torques
		command_torques = pos_task_torques + N.transpose()*joint_task_torques;
		if(gpjs)
		{
			command_torques += gravity_compensation;
		}
		// command_torques.setZero();

		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		if(controller_counter % 500 == 0)
		{
			cout << command_torques.transpose() << endl;
		}

		controller_counter++;

	}

	redis_client.setCommandIs(CONTROLLER_RUNNING, "0");
    command_torques.setZero();
    redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


    return 0;
}
