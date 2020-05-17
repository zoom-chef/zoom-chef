#ifndef _KEYS_H
#define _KEYS_H

// simulation keys
constexpr const char *SIM_JOINT_ANGLES_KEY = "sai2::examples::sensors::q";
constexpr const char *SIM_JOINT_VELOCITIES_KEY = "sai2::examples::sensors::dq";
constexpr const char *SIM_JOINT_TORQUES_COMMANDED_KEY = "sai2::examples::actuators::fgc";

// used when on real hardware
constexpr const char *HW_JOINT_ANGLES_KEY = "sai2::FrankaPanda::Bonnie::sensors::q";
constexpr const char *HW_JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Bonnie::sensors::dq";
constexpr const char *HW_JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::Bonnie::actuators::fgc";
constexpr const char *MASSMATRIX_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::massmatrix";
constexpr const char *ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::robot_gravity";
constexpr const char *CORIOLIS_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::coriolis";

// useful logging variables
constexpr const char *CURRENT_EE_POS_KEY = "sai2::examples::current_ee_pos";
constexpr const char *CURRENT_EE_VEL_KEY = "sai2::examples::current_ee_vel";

// controller initialization
constexpr const char *CONTROL_STATE_KEY = "sai2::examples::control_state";
constexpr const char *CONTROL_STATE_INITIALIZING = "initializing";
constexpr const char *CONTROL_STATE_INITIALIZED = "initialized";
constexpr const char *CONTROL_STATE_READY = "ready";

// posori task parameters
constexpr const char *DESIRED_POS_KEY = "sai2::examples::desired_position";
constexpr const char *DESIRED_ORI_KEY = "sai2::examples::desired_orientation";
constexpr const char *DESIRED_VEL_KEY = "sai2::examples::desired_velocity";
constexpr const char *KP_POS_KEY = "sai2::examples::kp_pos";
constexpr const char *KV_POS_KEY = "sai2::examples::kv_pos";
constexpr const char *USE_VEL_SAT_POSORI_KEY = "sai2::examples::use_posori_velocity_saturation";
constexpr const char *VEL_SAT_POSORI_KEY = "sai2::examples::posori_velocity_saturation";

// joint task parameters
constexpr const char *DESIRED_JOINT_POS_KEY = "sai2::examples::desired_joint_position";
constexpr const char *KP_JOINT_KEY = "sai2::examples::kp_joint";
constexpr const char *KV_JOINT_KEY = "sai2::examples::kv_joint";

// robot file
constexpr const char *ROBOT_FILE = "resources/panda_arm_hand.urdf";

// state-related keys
constexpr const char *PRIMITIVE_KEY = "sai2::examples::primitive";
constexpr const char *PRIMITIVE_JOINT_TASK = "primitive_joint_task";
constexpr const char *PRIMITIVE_POSORI_TASK = "primitive_posori_task";
constexpr const char *PRIMITIVE_TRAJECTORY_TASK = "primitive_trajectory_task";
constexpr const char *PRIMITIVE_FLOATING_TASK = "primitive_floating_task";

#endif
