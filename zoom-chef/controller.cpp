// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

// regular panda + gripper
// const string robot_file = "./resources/panda_arm_hand.urdf";
// panda + mobile base + gripper
const string robot_file = "./resources/mmp_panda.urdf";
const string spatula_file = "./resources/spatula.urdf";
// states
#define JOINT_CONTROLLER      0
#define POSORI_CONTROLLER     1
// tasks
#define IDLE                  0
#define SPATULA_PRE_POS       1
#define SPATULA_GRASP_POS	  2
#define SLIDE		  		  3
#define LIFT_SPATULA		  4
#define DROP_FOOD			  5
#define RELAX_WRIST			  6
#define FLEX_WRIST			  7
// gripper states
#define OPEN                  0
#define CLOSED                1
// stations
#define STATION_1             1
#define STATION_2             2

// print
#define VERBOSE				  0

int state = JOINT_CONTROLLER;
int task = SPATULA_PRE_POS;
int station = STATION_2;
int gripper_state = OPEN;
// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
std::string SPATULA_POSITION_KEY;
std::string SPATULA_ORIENTATION_KEY;
std::string SPATULA_JOINT_ANGLES_KEY;
// - write
std::string JOINT_TORQUES_COMMANDED_KEY;

// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

unsigned long long controller_counter = 0;

const bool inertia_regularization = true;

int main() {

	JOINT_ANGLES_KEY = "sai2::cs225a::project::sensors::q";
	JOINT_VELOCITIES_KEY = "sai2::cs225a::project::sensors::dq";
	JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::fgc";
	// KEY FOR SPATULA POSITION
	SPATULA_POSITION_KEY = "sai2::cs225a::spatula::sensors::r_spatula";
	SPATULA_ORIENTATION_KEY = "sai2::cs225a::spatula::sensors::q_spatula";
	SPATULA_JOINT_ANGLES_KEY = "sai2::cs225a::spatula::sensors::spatula_q";

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	// cout << initial_q << endl << endl;
	robot->updateModel();
	// from world urdf
	Vector3d base_origin;
	base_origin << 0.0, -0.05, 0.3514;

	Vector3d spatula_handle_pre_grasp_local;
	spatula_handle_pre_grasp_local << -0.35, 0, 0.1;
	// spatula_handle_pre_grasp_local << -0.4, 0, 0.27;

	Vector3d spatula_handle_grasp_local;
	spatula_handle_grasp_local << -0.25, 0, 0.04;
	// spatula_handle_grasp_local << -0.25, 0, 0.184;

	Vector3d base_offset;
	base_offset << 0.0, 0.0, 0.1757;
	// base_offset << 0.0, -0.05, 0.3514;

	Matrix3d handle_rot_local;
	handle_rot_local << -0.3553997, -0.3516974,  0.8660254,
  						-0.7033947,  0.7107995,  0.0000000,
  						-0.6155704, -0.6091577, -0.5000000; 

  	double finger_rest_pos = 0.02;
	double finger_closed_pos = -0.01;
 
	auto spatula = new Sai2Model::Sai2Model(spatula_file, false);
	Vector3d r_spatula = Vector3d::Zero();
	Matrix3d q_spatula = Matrix3d::Zero();
	VectorXd spatula_q(6);
	r_spatula = redis_client.getEigenMatrixJSON(SPATULA_POSITION_KEY);	
	q_spatula = redis_client.getEigenMatrixJSON(SPATULA_ORIENTATION_KEY);	
	spatula_q = redis_client.getEigenMatrixJSON(SPATULA_JOINT_ANGLES_KEY);	

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// pose task
	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0,0,0.07);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

#ifdef USING_OTG
	posori_task->_use_interpolation_flag = true;
#else
	posori_task->_use_velocity_saturation_flag = true;
#endif
	
	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_kp_pos = 200.0;
	posori_task->_kv_pos = 20.0;
	posori_task->_kp_ori = 200.0;
	posori_task->_kv_ori = 20.0;

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);

#ifdef USING_OTG
	joint_task->_use_interpolation_flag = true;
#else
	joint_task->_use_velocity_saturation_flag = true;
#endif

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 200.0;
	joint_task->_kv = 40.0;

	VectorXd q_init_desired = initial_q;
	joint_task->_desired_position = q_init_desired;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	Vector3d slide;
	slide << 0.0, 0.25, 0.0;
	Matrix3d slide_ori;
	double slide_angle = -5 * M_PI / 180.0;
	slide_ori << 	1.0000000, 0.0000000,  		 0.0000000,
   					0.0000000, cos(slide_angle), -sin(slide_angle),
   					0.0000000, sin(slide_angle), cos(slide_angle);

	Vector3d lift_height;
	lift_height << 0.0, 0.0, 0.25;

	Vector3d drop_food;
	drop_food << 0.0, 0.0, -0.25+0.05;

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		r_spatula = redis_client.getEigenMatrixJSON(SPATULA_POSITION_KEY);
		q_spatula = redis_client.getEigenMatrixJSON(SPATULA_ORIENTATION_KEY);

		// update model
		robot->updateModel();
		spatula->updateModel();
		
		VectorXd q_curr_desired(12);
		q_curr_desired = robot->_q;

		Vector3d ee_pos;
		robot->positionInWorld(ee_pos, control_link, control_point);
		Matrix3d ee_rot;
		robot->rotationInWorld(ee_rot, control_link);


		if(state == JOINT_CONTROLLER)
		{
			// update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
			if(station == STATION_1) {
				q_curr_desired(0) = -0.3514;
			}
	
			if(station == STATION_2)
			{
				q_curr_desired(0) = 0.3514;
			}

			if(gripper_state == OPEN)
			{
				q_curr_desired(10) = finger_rest_pos;
				q_curr_desired(11) = -finger_rest_pos;
			}
			if(gripper_state == CLOSED)
			{
				q_curr_desired(10) = finger_closed_pos;
				q_curr_desired(11) = -finger_closed_pos;
			}
			if(task == RELAX_WRIST) {
				q_curr_desired(9) = -M_PI/3;
			}
			// if (task == RELAX_WRIST) {
			// 	q_curr_desired(7) -= M_PI/3;
			// }
			joint_task->_desired_position = q_curr_desired;
			// compute torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques;

			if( (robot->_q - q_curr_desired).norm() < 0.05 )
			{
				if (task == SPATULA_PRE_POS) {
					state = POSORI_CONTROLLER;
				}
				if (task == SPATULA_GRASP_POS) {
					state = POSORI_CONTROLLER;
					task = SLIDE;
				}
				if (station == STATION_1 && task == LIFT_SPATULA) {
					state = POSORI_CONTROLLER;
					cout << "Dropping food on grill..." << endl << endl;
					task = DROP_FOOD;
					posori_task->reInitializeTask();
					posori_task->_desired_position += drop_food;
				}
				if (task == SLIDE) 
				{
					cout << "Sliding..." << endl << endl;
					posori_task->reInitializeTask();
					posori_task->_use_velocity_saturation_flag = true;
					posori_task->_linear_saturation_velocity = 0.1;
					posori_task->_desired_position += slide;
					posori_task->_desired_orientation *= slide_ori ; 
				}
			}
		}

		else if(state == POSORI_CONTROLLER)
		{
			// update task model and set hierarchy
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			// N_prec = posori_task->_N;
			// joint_task->kp = 250.0;
			
			if(gripper_state == OPEN)
			{
				q_curr_desired(10) = finger_rest_pos;
				q_curr_desired(11) = -finger_rest_pos;
			}
			if(gripper_state == CLOSED)
			{
				q_curr_desired(10) = finger_closed_pos;
				q_curr_desired(11) = -finger_closed_pos;
			}
			joint_task->_desired_position = q_curr_desired;
			joint_task->updateTaskModel(posori_task->_N);

			if (task == SPATULA_PRE_POS)
			{
				// cout << "Pre" << endl << endl;
				// need to maintain the finger position while moving to the spatula
				posori_task->reInitializeTask();
				// want this to be spatula position + local vector * local to world rotation
				posori_task->_desired_position = r_spatula + q_spatula.transpose() * spatula_handle_pre_grasp_local - base_offset;
				// go to spatula position
				posori_task->_desired_orientation = q_spatula.transpose() * handle_rot_local;
			} else if (task == SPATULA_GRASP_POS) {
				// cout << "Grasp" << endl << endl;
				// need to maintain the finger position while moving to the spatula
				posori_task->reInitializeTask();
				// want this to be spatula position + local vector * local to world rotation
				posori_task->_desired_position = r_spatula + q_spatula.transpose() * spatula_handle_grasp_local - base_offset;
				// go to spatula position
				posori_task->_desired_orientation = q_spatula.transpose() * handle_rot_local;
			// } else if (task == SLIDE) {
			} else if (task == DROP_FOOD) {
				// this state should be entered after closing the gripper
				// goal is to simply lift the spatula in the z-direction while keeping the same current orientation
				// posori_task->reInitializeTask();
	
			}
			
			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques;
			
			// if we have reached the desired position and orientation
			if(posori_task->goalPositionReached(0.01) && posori_task->goalOrientationReached(0.05))
			{
				// if we have moved into the pre-grasp position (essentially you position slightly away from the spatula to not contact it)
				// else if we have moved to a position with the spatula handle between the jaws of the gripper
				if (task == SPATULA_PRE_POS)
				{
					cout << "Moving to Grasp Position" << endl << endl;
					// move inwards to the grasp position
					task = SPATULA_GRASP_POS;					
				} else if (task == SPATULA_GRASP_POS) {
					cout << "Closing Gripper..." << endl << endl;
					// if we are in the grasp position, go to a joint task and close the jaws
					state = JOINT_CONTROLLER;
					gripper_state = CLOSED;
				} else if (task == SLIDE) {
					// state = POSORI_CONTROLLER;
					cout << "Lifting..." << endl << endl;
					posori_task->reInitializeTask();
					task =  LIFT_SPATULA;
					posori_task->_desired_position +=lift_height;
				} else if (task == LIFT_SPATULA) {
					state = JOINT_CONTROLLER;
					cout << "Changing station..." << endl << endl;
					joint_task->reInitializeTask();
					station = STATION_1;
				}
				// } else if (task == DROP_FOOD) {
				// 	state = JOINT_CONTROLLER;
				// }
				else if (task == DROP_FOOD){
					cout << "\t(Relaxing wrist...)" << endl << endl;
					task = RELAX_WRIST;
					// posori_task->reInitializeTask();
					state = JOINT_CONTROLLER;
				}

			}
		}

		if(controller_counter % 5000 == 0 && VERBOSE)
		{
			cout << "gripper state = " << gripper_state << endl;
			cout << "ee_pos = " << ee_pos.transpose() << endl;
			cout << "ee_rot = " << endl << ee_rot << endl;
		}
		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
