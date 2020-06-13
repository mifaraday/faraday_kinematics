#ifndef __FARADAY_KINEMATICS_H__
#define __FARADAY_KINEMATICS_H__

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>		//Eigen

#include <ceres/ceres.h>
// #include <chrono>

namespace faraday_kinematics
{
	double default_initial_cartesian_value[5]={0.0,0.0,180.0,0.0,0.0};

	bool inverse_kinematics(const double cartesian_mm[],double actuator_mm[]);
	bool forward_kinematics(const double actuator_mm[],double cartesian_mm[],
							double last_cartesian_mm[]=default_initial_cartesian_value);


struct IntermediatePoints
{
	const static int N_ARMS=4;
	const static int N_DIRS=3;
	typedef Eigen::Matrix<double,N_DIRS,N_ARMS> ArmMatrixed;

	 // [  arm1x   arm2x   arm3x   arm4x  ]
     // [  arm1y   arm2y   arm3y   arm4y  ]
     // [  arm1z   arm2z   arm3z   arm4z  ]

	ArmMatrixed A;	//position of U1,U2,U3,U4

	ArmMatrixed B;	//position of S1,S2,S3,S4

	Eigen::Vector3d C;	//position of center limb 4 points

	//Alignment support
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

bool calcIntermediatePoints(const double actuator_mm[],const double cartesian_mm[],
							IntermediatePoints& pts);

}

#endif //__FARADAY_KINEMATICS_H__