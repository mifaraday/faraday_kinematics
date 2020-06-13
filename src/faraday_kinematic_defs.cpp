#include "faraday_kinematics/faraday_kinematic_defs.h"
#include <cmath>
#include <iostream>


const Eigen::Vector3d faraday_kinematics::faraday_B1_base( 
				FARADAY_A_BASE*std::cos(FARADAY_THETA1_BASE),
				FARADAY_A_BASE*std::sin(FARADAY_THETA1_BASE),
				0);

const Eigen::Vector3d faraday_kinematics::faraday_B2_base(
				FARADAY_A_BASE*std::cos(FARADAY_THETA2_BASE),
				-FARADAY_A_BASE*std::sin(FARADAY_THETA2_BASE),
				0);

const Eigen::Vector3d faraday_kinematics::faraday_B3_base(
				-FARADAY_A_BASE*std::cos(FARADAY_THETA2_BASE),
				-FARADAY_A_BASE*std::sin(FARADAY_THETA2_BASE),
				0);

const Eigen::Vector3d faraday_kinematics::faraday_B4_base(
				-FARADAY_A_BASE*std::cos(FARADAY_THETA1_BASE),
				FARADAY_A_BASE*std::sin(FARADAY_THETA1_BASE),
				0);

const Eigen::Vector3d faraday_kinematics::faraday_S1_move(
				FARADAY_R_MOVE*std::cos(FARADAY_THETA3_MOVE),
				FARADAY_R_MOVE*std::sin(FARADAY_THETA3_MOVE),
				0);

const Eigen::Vector3d faraday_kinematics::faraday_S2_move(
				FARADAY_R_MOVE*std::cos(FARADAY_THETA3_MOVE),
				-FARADAY_R_MOVE*std::sin(FARADAY_THETA3_MOVE),
				0);

const Eigen::Vector3d faraday_kinematics::faraday_S3_move(
				-FARADAY_R_MOVE*std::cos(FARADAY_THETA3_MOVE),
				-FARADAY_R_MOVE*std::sin(FARADAY_THETA3_MOVE),
				0);

const Eigen::Vector3d faraday_kinematics::faraday_S4_move(
				-FARADAY_R_MOVE*std::cos(FARADAY_THETA3_MOVE),
				FARADAY_R_MOVE*std::sin(FARADAY_THETA3_MOVE),
				0);

const Eigen::Vector3d faraday_kinematics::faraday_S5_move(
				0,
				0,
				-FARADAY_D_MOVE);

