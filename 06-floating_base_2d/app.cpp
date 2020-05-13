#include <iostream>
#include <string>
#include <thread>
#include <math.h>
#include <fstream>

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"

#include "force_sensor/ForceSensorSim.h"
#include "timer/LoopTimer.h"
#include "filters/ButterworthFilter.h"
#include "uiforce/UIForceWidget.h"

#include "MomentumObserver.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;
using namespace Eigen;

const string world_file = "./resources/world.urdf";
const string robot_file = "./resources/4rbot_floating.urdf";
const string robot_name = "4RBOT";

const string camera_name = "camera_fixed";

// simulation loop
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget);
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);
void logger(Sai2Model::Sai2Model* robot);

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for ui widget click
bool fRobotLinkSelect = false;
Eigen::Vector3d ui_force;
Eigen::VectorXd ui_force_command_torques;

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;

VectorXd command_torques;
Vector2d xd;
Vector2d f_sensor;
// VectorXd disturbance_torques;

RedisClient redis_client;

int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, false);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setCollisionRestitution(0);
	sim->setCoeffFrictionStatic(0.0);

	// cout << sim->getCoeffFrictionStatic("Floor_bis");
	// sim->setCoeffFrictionStatic("Floor", 0.00);
	// sim->setCoeffFrictionStatic("Floor_bis", 0.6);
	// sim->setCoeffFrictionStatic(robot_name, "link2", 0.6);
	// sim->setCoeffFrictionStatic(robot_name, "link4", 0.00);

	// load robots
	Affine3d T_world_robot = sim->getRobotBaseTransform(robot_name);
	auto robot = new Sai2Model::Sai2Model(robot_file, false, T_world_robot);

	// set initial condition
	robot->_q <<  0.0,
				  22.0/180.0*M_PI,
				 -60.0/180.0*M_PI,
				  80.0/180.0*M_PI,
				 -80.0/180.0*M_PI;
	sim->setJointPositions(robot_name, robot->_q);
	robot->updateModel();

	// init click force widget 
	auto ui_force_widget = new UIForceWidget(robot_name, robot, graphics);
	ui_force_widget->setEnable(false);

	ui_force_widget->_spring_k = 50.0;
	ui_force_widget->_max_force = 100.0;

	int dof = robot->dof();
	ui_force.setZero();
	ui_force_command_torques.setZero(dof);

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

	double last_cursorx, last_cursory;

    // set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// start the simulation thread first
	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim);

	// next start the control thread
	thread ctrl_thread(control, robot, sim, ui_force_widget);

	// start the logging thread
	thread logging_thread(logger, robot);

    // while window is open:
    while (!glfwWindowShouldClose(window)) {
		// update kinematic models
		// robot->updateModel();

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->render(camera_name, width, height);
		glfwSwapBuffers(window);
		glFinish();

	    // poll for events
	    glfwPollEvents();

		// detect click to the link
		ui_force_widget->setEnable(fRobotLinkSelect);
		if (fRobotLinkSelect)
		{
			double cursorx, cursory;
			int wwidth_scr, wheight_scr;
			int wwidth_pix, wheight_pix;
			std::string ret_link_name;
			Eigen::Vector3d ret_pos;

			// get current cursor position
			glfwGetCursorPos(window, &cursorx, &cursory);

			glfwGetWindowSize(window, &wwidth_scr, &wheight_scr);
			glfwGetFramebufferSize(window, &wwidth_pix, &wheight_pix);

			int viewx = floor(cursorx / wwidth_scr * wwidth_pix);
			int viewy = floor(cursory / wheight_scr * wheight_pix);

			if(!ui_force_widget->setInteractionParams(camera_name, viewx, wheight_pix - viewy, wwidth_pix, wheight_pix))
			{
				fRobotLinkSelect = false;
			}
		}

		// move scene camera as required
    	// graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
    	Eigen::Vector3d cam_depth_axis;
    	cam_depth_axis = camera_lookat - camera_pos;
    	cam_depth_axis.normalize();
    	Eigen::Vector3d cam_up_axis;
    	// cam_up_axis = camera_vertical;
    	// cam_up_axis.normalize();
    	cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
	    Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
    	cam_roll_axis.normalize();
    	Eigen::Vector3d cam_lookat_axis = camera_lookat;
    	cam_lookat_axis.normalize();
    	if (fTransXp) {
	    	camera_pos = camera_pos + 0.05*cam_roll_axis;
	    	camera_lookat = camera_lookat + 0.05*cam_roll_axis;
	    }
	    if (fTransXn) {
	    	camera_pos = camera_pos - 0.05*cam_roll_axis;
	    	camera_lookat = camera_lookat - 0.05*cam_roll_axis;
	    }
	    if (fTransYp) {
	    	// camera_pos = camera_pos + 0.05*cam_lookat_axis;
	    	camera_pos = camera_pos + 0.05*cam_up_axis;
	    	camera_lookat = camera_lookat + 0.05*cam_up_axis;
	    }
	    if (fTransYn) {
	    	// camera_pos = camera_pos - 0.05*cam_lookat_axis;
	    	camera_pos = camera_pos - 0.05*cam_up_axis;
	    	camera_lookat = camera_lookat - 0.05*cam_up_axis;
	    }
	    if (fTransZp) {
	    	camera_pos = camera_pos + 0.1*cam_depth_axis;
	    	camera_lookat = camera_lookat + 0.1*cam_depth_axis;
	    }	    
	    if (fTransZn) {
	    	camera_pos = camera_pos - 0.1*cam_depth_axis;
	    	camera_lookat = camera_lookat - 0.1*cam_depth_axis;
	    }
	    if (fRotPanTilt) {
	    	// get current cursor position
	    	double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
	    }
	    graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
	    glfwGetCursorPos(window, &last_cursorx, &last_cursory);
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();
	ctrl_thread.join();
	logging_thread.join();

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget) {
	robot->updateModel();

	int dof = robot->dof();
	command_torques = Eigen::VectorXd::Zero(dof);
	// disturbance_torques.setZero(dof);
	Eigen::VectorXd gravity_compensation;

	// joint task
	Eigen::VectorXd joint_task_torques = Eigen::VectorXd::Zero(dof);
	VectorXd q_desired = robot->_q;
	double kp_joint = 50.0;
	double kv_joint = 14.0;

	double alpha = 0;

	// shoulder task
	double shoulder_desired = robot->_q(0);
	double kp_shoulder = 100.0;
	double kv_shoulder = 20.0;
	double shoulder_force = 0;
	VectorXd shoulder_torques = VectorXd::Zero(dof);
	MatrixXd J_shoulder = MatrixXd::Zero(1,dof);
	J_shoulder(0) = 1.0;
	MatrixXd N_shoulder, Lambda_shoulder, Jbar_shoulder;
	N_shoulder.setIdentity(dof, dof);
	Jbar_shoulder.setZero(dof,1);
	Lambda_shoulder.setZero(1,1);

	// bracing task
	VectorXd bracing_task_torques = VectorXd::Zero(dof);
	MatrixXd N_bracing = MatrixXd::Identity(dof,dof);
	MatrixXd J_bracing = MatrixXd::Zero(1,dof);
	MatrixXd Jbar_bracing = MatrixXd::Zero(dof,1);
	MatrixXd Lambda_bracing = MatrixXd::Zero(1,1);

	// position task
	const string link_name = "link4";
	const Eigen::Vector3d pos_in_link = Eigen::Vector3d(0.0, 1.0, 0.0);
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
	Eigen::Vector3d x_init;
	robot->position(x_init, link_name, pos_in_link);
	xd = x_init.tail(2);

	double amplitude = 0.2;
	double freq_motion = 0.4;

	// momentum observer
	auto tau_observer = new MomentumObserver(robot, 0.001);
	double gain = 10.0;
	tau_observer->setGain(gain * MatrixXd::Identity(dof,dof));
	VectorXd tau_observed = VectorXd::Zero(dof);

	VectorXd effective_gravity = VectorXd::Zero(dof);

	// filter for commanded_torques
	auto filter_tau_cmd = new ButterworthFilter(dof, 0.05);

	// task contact torques
	VectorXd task_contact_torques = VectorXd::Zero(dof);
	f_sensor.setZero();

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	unsigned long long controller_counter = 0;

	Eigen::Vector3d sensed_force = Eigen::Vector3d::Zero();

	bool bracing = false;
	bool gpjs = true;
	// gpjs = false;

	redis_client.createReadCallback(0);
	redis_client.addDoubleToReadCallback(0, "alpha_key", alpha);

	redis_client.createWriteCallback(0);
	redis_client.addDoubleToWriteCallback(0, "alpha_key", alpha);

	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// update time
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;

		redis_client.executeReadCallback(0);

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();

		// task_contact_torques = Jv2d.transpose() * f_sensor + ui_force_command_torques;
		task_contact_torques = Jv2d.transpose() * f_sensor;
		tau_observer->update(command_torques, task_contact_torques);
		tau_observed = tau_observer->getDisturbanceTorqueEstimate();
		tau_observed(3) = 0;
		tau_observed(4) = 0;

		robot->J_0(J3d, link_name, pos_in_link);
		Jv2d = J3d.block(1,0,2,dof);
		robot->operationalSpaceMatrices(Lambda, Jbar, N, Jv2d);

		J_shoulder = J_shoulder * N;
		robot->operationalSpaceMatrices(Lambda_shoulder, Jbar_shoulder, N_shoulder, J_shoulder, N);

		N_bracing = N_shoulder;
		if(bracing)
		{
			J_bracing = tau_observed.transpose() * N_shoulder;
			robot->operationalSpaceMatrices(Lambda_bracing, Jbar_bracing, N_bracing, J_bracing, N_shoulder);
		}

		// -------------------------------------------
		////////////////////////////// Compute joint torques
		double time = controller_counter/control_freq;

		robot->gravityVector(gravity_compensation);

		//----------- pos task
		Matrix2d Sigma_pos = Matrix2d::Zero(2,2);
		Matrix2d Sigma_force = Matrix2d::Zero(2,2);
		Sigma_pos(0,0) = 1;
		Sigma_force(1,1) = 1;


		xd(0) = x_init(1) + amplitude * sin(2 * M_PI * freq_motion * curr_time);
		Vector2d dxd = Vector2d::Zero();
		dxd(0) = 2 * M_PI * freq_motion * amplitude * cos(2 * M_PI * freq_motion * curr_time);
		Vector2d ddxd = Vector2d::Zero();
		ddxd(0) = -2 * M_PI * freq_motion * 2 * M_PI * freq_motion * amplitude * sin(2 * M_PI * freq_motion * curr_time);

		robot->position(pos3d, link_name, pos_in_link);
		x = pos3d.tail(2);
		xdot = Jv2d*robot->_dq;

		pos_task_force.setZero();
		pos_task_force = Lambda * Sigma_pos * (ddxd - kp_pos*(x - xd) - kv_pos*(xdot - dxd));
		// pos_task_force = Lambda * (ddxd - kp_pos*(x - xd) - kv_pos*(xdot - dxd));
		// pos_task_force = Sigma_pos *(ddxd - kp_pos*(x - xd) - kv_pos*(xdot - dxd));

		// force task
		Vector2d desired_force = Vector2d(0, -10.0);
		pos_task_force += Sigma_force * desired_force;

		pos_task_force += Jbar.transpose() * gravity_compensation;
		// if(!gpjs)
		// {
			// gravity_compensation.setZero();
		// }
		pos_task_force += Jbar.transpose() * tau_observed;
		pos_task_torques = Jv2d.transpose() * pos_task_force;

		// shoulder task
		shoulder_force = Lambda_shoulder(0,0) * (- kp_shoulder * (robot->_q(0) - shoulder_desired) - kv_shoulder * robot->_dq(0));
		// shoulder_force = Lambda_shoulder(0,0) * ( - kv_shoulder * robot->_dq(0));
		shoulder_force += (Jbar_shoulder.transpose() * gravity_compensation)(0);
		shoulder_force += (Jbar_shoulder.transpose() * tau_observed)(0);

		shoulder_torques = J_shoulder.transpose() * shoulder_force;

		//----- Joint nullspace damping
		if(!gpjs && !bracing)
		{
			q_desired(1) -= 0.0001;
		}
		joint_task_torques = robot->_M*( - kp_joint * (robot->_q - q_desired) - kv_joint*robot->_dq);
		joint_task_torques += gravity_compensation;
		joint_task_torques += tau_observed;
		joint_task_torques = N_bracing.transpose() * joint_task_torques;

		MatrixXd Pr = tau_observed * Jbar_bracing.transpose();
		VectorXd Prg = Pr * gravity_compensation;

		if(controller_counter % 10 == 0)
		{
			if(bracing)
			{
				alpha = 1 - (gravity_compensation + Jv2d.transpose() * Sigma_force * desired_force).dot(Prg)/Prg.squaredNorm();
				// alpha = 1 - (gravity_compensation).dot(Prg)/Prg.squaredNorm();
				if(alpha > 1)
				{
					alpha = 1;
				}
				if(alpha < -1)
				{
					alpha = -1;
				}
			}
		}
		bracing_task_torques = alpha * gravity_compensation;
		bracing_task_torques = (N_shoulder.transpose() - N_bracing.transpose()) * bracing_task_torques;


		// VectorXd G_fixed = gravity_compensation;
		// // cout << Jv2d.transpose() * Sigma_force * desired_force << endl;
		// // G_fixed += Jv2d.transpose() * Sigma_force * desired_force;
		// VectorXd G_alpha = (N_shoulder.transpose() - N_bracing.transpose()) * gravity_compensation;
		// double grad_alpha = G_alpha.dot(G_fixed) + alpha * G_fixed.dot(G_fixed);
		// double hess_alpha = G_fixed.norm();

		// if(bracing)
		// {
		// 	if(controller_counter % 10 == 0)
		// 	{

		// 		// if(G_fixed.norm() > 0.01)
		// 		// {
		// 			// alpha = -(double) G_fixed.dot(G_alpha) / G_fixed.norm();
		// 		// }
		// 		// alpha -= 0.01 * grad_alpha/abs(grad_alpha);
		// 		alpha -= 0.001 * grad_alpha/hess_alpha;
		// 		if(alpha < -1)
		// 		{
		// 			alpha = -1;
		// 		}
		// 		if(alpha > 1)
		// 		{
		// 			alpha = 1;
		// 		}
		// 		// cout << alpha << endl;
		// 		// cout << grad_alpha << endl;
		// 		// cout << hess_alpha << endl;
		// 		// cout << endl;
		// 	}
		// 	bracing_task_torques = alpha * gravity_compensation;
		// 	bracing_task_torques = (N_shoulder.transpose() - N_bracing.transpose()) * bracing_task_torques;
		// }

		// if(!gpjs)
		// {
		// 	if(controller_counter % 10 == 0)
		// 	{
		// 		VectorXd G_joint = N_shoulder.transpose() * tau_observed;
		// 		VectorXd G_task = Jv2d.transpose() * Jbar.transpose() * (gravity_compensation + tau_observed);
		// 		G_task += Jv2d.transpose() * Sigma_force * desired_force;
		// 		G_task += J_shoulder.transpose() * Jbar_shoulder.transpose() * gravity_compensation;
		// 		G_task += J_shoulder.transpose() * Jbar_shoulder.transpose() * tau_observed;

		// 		double grad_alpha = G_task.dot(G_joint) + alpha * G_joint.dot(G_joint);
		// 		double hess_alpha = G_joint.norm();
		// 		// if(G_joint.norm() > 0.01)
		// 		// {
		// 			// alpha = -(double) G_joint.dot(G_task) / G_joint.norm();
		// 		// }
		// 		// alpha -= 0.01 * grad_alpha/abs(grad_alpha);
		// 		alpha -= 0.001 * grad_alpha/hess_alpha;
		// 		if(alpha < -1)
		// 		{
		// 			alpha = -1;
		// 		}
		// 		if(alpha > 1)
		// 		{
		// 			alpha = 1;
		// 		}
		// 		// cout << alpha << endl;
		// 		// cout << grad_alpha << endl;
		// 		// cout << hess_alpha << endl;
		// 		// cout << endl;
		// 	}
		// 	joint_task_torques += alpha * tau_observed;
		// }




		//------ Final torques
		command_torques = pos_task_torques + shoulder_torques + bracing_task_torques + joint_task_torques;
		// command_torques = pos_task_torques + N.transpose()*joint_task_torques + gravity_compensation;
		// command_torques = filter_tau_cmd->update(command_torques);
		// if(gpjs)
		// {
		// 	command_torques += gravity_compensation;
		// }
		// command_torques.setZero();

		// -------------------------------------------
		// disturbance_torques << 10,10,10,10;
		// 
		// get ui force and torques
		if(ui_force_widget->getState() == UIForceWidget::UIForceWidgetState::Active)
		{
			ui_force_widget->getUIForce(ui_force);
			ui_force_widget->getUIJointTorques(ui_force_command_torques);
		}
		else
		{
			ui_force.setZero();
			ui_force_command_torques.setZero(dof);
		}

		// 
		// 
		VectorXd impact_torques = VectorXd::Zero(dof);
		// if(controller_counter % 3000 > 995 && controller_counter % 3000 < 1005)
		// {
		// 	impact_torques(0) = 100.0;
		// }
		// if(controller_counter % 3000 > 1995 && controller_counter % 3000 < 2005)
		// {
		// 	impact_torques(0) = -100.0;
		// }

		sim->setJointTorques(robot_name, command_torques + ui_force_command_torques + impact_torques);
		

		// MatrixXd C = MatrixXd::Zero(dof,dof);
		// robot->factorizedChristoffelMatrix(C);
		// VectorXd coriolis = VectorXd::Zero(dof);
		// robot->coriolisForce(coriolis);
		// VectorXd diff_coriolis = C * robot->_dq - coriolis;

		// cout << diff_coriolis.norm() << endl;

		// cout << "tau dist sim : " << disturbance_torques.transpose() << endl;
		// cout << "tau dist obs : " << tau_observed.transpose() << endl;
		// cout << "difference : " << (tau_observed - disturbance_torques).norm() << endl;
		// cout << endl;

		// -------------------------------------------
		if(controller_counter % 500 == 0)
		{
			Vector2d grav_2d = Vector2d(0.0, -9.81);
			// cout << "J : \n" << J3d << endl;
			// cout << "x init : " << x_init.transpose() << endl;
			// cout << "xerr : " << (x - xd).transpose() << endl;
			// cout << "x : " << x.transpose() << endl;
			// cout << "xd : " << xd.transpose() << endl;
			// cout << "lambda : " << Lambda << endl;
			// cout << "pos task force : " << pos_task_force.transpose() << endl;
			// cout << command_torques.transpose() << endl;
			// cout << 180.0/M_PI*robot->_q.transpose() << endl;
			cout << "alpha : " << alpha << endl;
			cout << "command torques norm 1 : " << command_torques.lpNorm<1>() << endl;
			cout << "command torques norm 2 : " << command_torques.norm() << endl;
			// cout << "p task : " << (Jbar.transpose() * gravity_compensation).transpose() << endl;
			// cout << "mg task : " << (Lambda * grav_2d).transpose() << endl;
			// cout << "(I-N^T) tau_obs : " << ((MatrixXd::Identity(dof,dof) - N.transpose()) * tau_observed).transpose() << endl;
			cout << endl;
			// cout << controller_counter << endl;
		}
		if(controller_counter == 10000)
		{
			gpjs = false;
		}
		if(tau_observed.norm() > 10.0 && !gpjs)
		{
			bracing = true;
		}

		redis_client.executeWriteCallback(0);

		controller_counter++;

		// -------------------------------------------
		// update last time
		last_time = curr_time;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Control Loop run time  : " << end_time << " seconds\n";
    std::cout << "Control Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Control Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

}

//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {
	fSimulationRunning = true;

	int dof = robot->dof();

	// create a force sensor
	const string link_name = "link4";
	const Vector3d pos_in_link = Vector3d(0, 1.0, 0);
	Affine3d transform_in_link = Affine3d::Identity();
	transform_in_link.translation() = pos_in_link;
	auto fsensor = new ForceSensorSim(robot_name, link_name, transform_in_link, robot);

	Vector3d f_contact = Vector3d::Zero();
	// Vector3d m_contact = Vector3d::Zero();
	// MatrixXd Jv_contact = MatrixXd::Zero(3,dof);
	// MatrixXd Jw_contact = MatrixXd::Zero(3,dof);

	// VectorXd fm_contact = VectorXd::Zero(6);
	// MatrixXd J_contact = MatrixXd::Zero(6,dof);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(2000); 
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// integrate forward
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time; 
		sim->integrate(loop_dt);

		// sim->showContactInfo();

		fsensor->update(sim);
		fsensor->getForce(f_contact);
		f_sensor = -f_contact.tail(2);
		// fsensor->getMoment(m_contact);
		// fsensor->getForceMoment(fm_contact);

		// f_contact.head(2) *= -1;

		// robot->J_0WorldFrame(J_contact, link_name, pos_in_link);
		// robot->JvWorldFrame(Jv_contact, link_name, pos_in_link);
		// robot->Jw(Jw_contact, link_name);
		// disturbance_torques = Jv_contact.transpose() * f_contact + Jw_contact.transpose() * m_contact;
		// disturbance_torques = J_contact.transpose() * fm_contact;

		// cout << f_contact.transpose() << endl;
		// cout << (Jw_contact.transpose() * m_contact).transpose() << endl;
		// cout << endl;
		// cout << (disturbance_torques - J_contact.transpose()*fm_contact).norm() << endl;


		// if (!fTimerDidSleep) {
		// 	cout << "Warning: timer underflow! dt: " << loop_dt << "\n";
		// }

		// update last time
		last_time = curr_time;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
    std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
}


//------------------------------------------------------------------------------
void logger(Sai2Model::Sai2Model* robot)
{
	std::ofstream data_file;

	data_file.open("../../06-floating_base_2d/data_logging/data/data.txt");


	data_file << "Timestep \t x_error[2] \t command torques[5] \t f contact[2]\n";

	Eigen::Vector3d ee_pos;
	const string link_name = "link4";
	const Eigen::Vector3d pos_in_link = Eigen::Vector3d(0.0, 1.0, 0.0);

	// robot->position(ee_pos, link_name, pos_in_link);
	// Eigen::Vector2d xd = ee_pos.tail(2);

	// create a loop timer
	LoopTimer timer;
	timer.setLoopFrequency(1000);   // 1 KHz
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	while(fSimulationRunning)
	{
		fTimerDidSleep = timer.waitForNextLoop();
		double curr_time = timer.elapsedTime();

		robot->position(ee_pos, link_name, pos_in_link);

		data_file << curr_time << '\t';
		data_file << (xd - ee_pos.tail(2)).transpose() << '\t';
		data_file << command_torques.transpose() << '\t';
		data_file << f_sensor.transpose() << '\n';
	}

	data_file.close();
}

//------------------------------------------------------------------------------
GLFWwindow* glfwInitialize() {
		/*------- Set up visualization -------*/
    // set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
	int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - CS327a HW2", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	return window;
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
    switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			glfwSetWindowShouldClose(window,GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransXp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		case GLFW_KEY_A:
			fTransZp = set;
			break;
		case GLFW_KEY_Z:
			fTransZn = set;
			break;
		default:
			break;
    }
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
		switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			fRobotLinkSelect = set;
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}