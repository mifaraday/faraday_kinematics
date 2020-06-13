#ifndef __FARADAY_KINEMATIC_DEFS_H__
#define __FARADAY_KINEMATIC_DEFS_H__

#include <eigen3/Eigen/Dense>

/*本文件描述机构的各个尺寸，长度单位默认为毫米，
*角度单位为弧度
*
*/

//定平台上的尺寸
#define FARADAY_THETA1_BASE 0.3298672286 //param 0
#define FARADAY_THETA2_BASE 0.6379178416 //param 1

#define FARADAY_A_BASE 240.99  //param 2

//|UiSi|的长度
#define FARADAY_C 246.0  //param3

//动平台上的尺寸
#define FARADAY_THETA3_MOVE 0.5403539364 //param 3

#define FARADAY_R_MOVE 58.31 //param 4
#define FARADAY_D_MOVE 76.0   //param 5

namespace faraday_kinematics
{
	//B1-4在定系中的坐标
	extern const Eigen::Vector3d faraday_B1_base;
	extern const Eigen::Vector3d faraday_B2_base;
	extern const Eigen::Vector3d faraday_B3_base;
	extern const Eigen::Vector3d faraday_B4_base;

	//S1-5在动系中的坐标
	extern const Eigen::Vector3d faraday_S1_move;
	extern const Eigen::Vector3d faraday_S2_move;
	extern const Eigen::Vector3d faraday_S3_move;
	extern const Eigen::Vector3d faraday_S4_move;
	extern const Eigen::Vector3d faraday_S5_move;
}

// void debugParams();

#endif //__FARADAY_KINEMATIC_DEFS_H__