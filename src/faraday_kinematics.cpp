#include "faraday_kinematics/faraday_kinematics.h"
#include "faraday_kinematics/faraday_kinematic_defs.h"

#include <cmath>
#include <iostream>

#define PIOVER180 0.01745329251994329576923690768489

//cartesian to actuator - inverse kinematics
bool faraday_kinematics::inverse_kinematics(const double cartesian_mm[],
											double actuator_mm[])
{
	double A[4],B[4],C[4];

	double b[4];

	//EulerAngles to RotationMatrix  绕动系yaw pitch roll
	// Eigen::Vector3d EulerAngles(cartesian_mm[5],cartesian_mm[4],cartesian_mm[3]);    //6 DOF
    Eigen::Vector3d EulerAngles(0.0, cartesian_mm[4], cartesian_mm[3]);     //5 DOF
	Eigen::Matrix3d R;
	R=Eigen::AngleAxisd(EulerAngles[0],Eigen::Vector3d::UnitZ())
	 *Eigen::AngleAxisd(EulerAngles[1],Eigen::Vector3d::UnitY())
	 *Eigen::AngleAxisd(EulerAngles[2],Eigen::Vector3d::UnitX());

	 //变换矩阵中的平移部分
	Eigen::Vector3d P(cartesian_mm[0],cartesian_mm[1],cartesian_mm[2]);

	//计算动平台各点在固定坐标系中的坐标
	const Eigen::Vector3d faraday_S1_base=R*faraday_S1_move+P;
	const Eigen::Vector3d faraday_S2_base=R*faraday_S2_move+P;
	const Eigen::Vector3d faraday_S3_base=R*faraday_S3_move+P;
	const Eigen::Vector3d faraday_S4_base=R*faraday_S4_move+P;
	const Eigen::Vector3d faraday_S5_base=R*faraday_S5_move+P;

	for(int i=0;i<4;++i)
		A[i]=1;

	B[0]=-2*faraday_S1_base[2];
	B[1]=-2*faraday_S2_base[2];
	B[2]=-2*faraday_S3_base[2];
	B[3]=-2*faraday_S4_base[2];

	C[0]=std::pow(faraday_S1_base[2],2.0)-std::pow(FARADAY_C,2.0)
		+std::pow((faraday_B1_base[1]-faraday_S1_base[1]),2.0)
		+std::pow((faraday_B1_base[0]-faraday_S1_base[0]),2.0);

	C[1]=std::pow(faraday_S2_base[2],2.0)-std::pow(FARADAY_C,2.0)
		+std::pow((faraday_B2_base[1]-faraday_S2_base[1]),2.0)
		+std::pow((faraday_B2_base[0]-faraday_S2_base[0]),2.0);

	C[2]=std::pow(faraday_S3_base[2],2.0)-std::pow(FARADAY_C,2.0)
		+std::pow((faraday_B3_base[1]-faraday_S3_base[1]),2.0)
		+std::pow((faraday_B3_base[0]-faraday_S3_base[0]),2.0);

	C[3]=std::pow(faraday_S4_base[2],2.0)-std::pow(FARADAY_C,2.0)
		+std::pow((faraday_B4_base[1]-faraday_S4_base[1]),2.0)
		+std::pow((faraday_B4_base[0]-faraday_S4_base[0]),2.0);

	for(int i=0;i<4;++i)
		b[i]=(-B[i]-std::sqrt(std::pow(B[i],2.0)-4*A[i]*C[i]))/2;

	for(int i=0;i<4;++i)
		actuator_mm[i]=b[i];
	
	actuator_mm[4]=faraday_S5_base[0];
	actuator_mm[5]=faraday_S5_base[1];
    actuator_mm[6]=faraday_S5_base[2];


	for(int i = 0; i < 7; ++i)
  	{
    	if (std::isnan(cartesian_mm[i])) return false;
 	}

  	return true;
}


//exploit numerical algorithm to get kinematic solution

using std::cos;
using std::sin;
using std::sqrt;
using std::pow;

struct F1
{
    explicit F1(double b1):_b1(b1)
    {
        _S1x=faraday_kinematics::faraday_S1_move[0];
        _S1y=faraday_kinematics::faraday_S1_move[1];
        _U1x=faraday_kinematics::faraday_B1_base[0];
        _U1y=faraday_kinematics::faraday_B1_base[1];
        _c=FARADAY_C;
    }
    template <typename T>
    bool operator()(const T* const p1, const T* const p2,
                    const T* const p3, const T* const gamma,
                    const T* const beta,T* residual) const
    {
        residual[0]=sqrt(pow(cos(beta[0])*_S1x+sin(beta[0])*sin(gamma[0])*_S1y+p1[0]-_U1x,2)+
                    pow(cos(gamma[0])*_S1y+p2[0]-_U1y,2)+
                    pow(-sin(beta[0])*_S1x+cos(beta[0])*sin(gamma[0])*_S1y+p3[0]-_b1,2))-_c;

        return true;
    }
    double _S1x,_S1y,_U1x,_U1y,_c;
    const double _b1;
};

struct F2
{
    explicit F2(double b2):_b2(b2)
    {
        _S2x=faraday_kinematics::faraday_S2_move[0];
        _S2y=faraday_kinematics::faraday_S2_move[1];
        _U2x=faraday_kinematics::faraday_B2_base[0];
        _U2y=faraday_kinematics::faraday_B2_base[1];
        _c=FARADAY_C;
    }
    template <typename T>
    bool operator()(const T* const p1, const T* const p2,
                    const T* const p3, const T* const gamma,
                    const T* const beta,T* residual) const
    {
        residual[0]=sqrt(pow(cos(beta[0])*_S2x+sin(beta[0])*sin(gamma[0])*_S2y+p1[0]-_U2x,2)+
                    pow(cos(gamma[0])*_S2y+p2[0]-_U2y,2)+
                    pow(-sin(beta[0])*_S2x+cos(beta[0])*sin(gamma[0])*_S2y+p3[0]-_b2,2))-_c;

        return true;
    }
    double _S2x,_S2y,_U2x,_U2y,_c;
    const double _b2;
};

struct F3
{
    explicit F3(double b3):_b3(b3)
    {
        _S3x=faraday_kinematics::faraday_S3_move[0];
        _S3y=faraday_kinematics::faraday_S3_move[1];
        _U3x=faraday_kinematics::faraday_B3_base[0];
        _U3y=faraday_kinematics::faraday_B3_base[1];
        _c=FARADAY_C;
    }
    template <typename T>
    bool operator()(const T* const p1, const T* const p2,
                    const T* const p3, const T* const gamma,
                    const T* const beta,T* residual) const
    {
        residual[0]=sqrt(pow(cos(beta[0])*_S3x+sin(beta[0])*sin(gamma[0])*_S3y+p1[0]-_U3x,2)+
                    pow(cos(gamma[0])*_S3y+p2[0]-_U3y,2)+
                    pow(-sin(beta[0])*_S3x+cos(beta[0])*sin(gamma[0])*_S3y+p3[0]-_b3,2))-_c;

        return true;
    }
    double _S3x,_S3y,_U3x,_U3y,_c;
    const double _b3;
};

struct F4
{
    explicit F4(double b4):_b4(b4)
    {
        _S4x=faraday_kinematics::faraday_S4_move[0];
        _S4y=faraday_kinematics::faraday_S4_move[1];
        _U4x=faraday_kinematics::faraday_B4_base[0];
        _U4y=faraday_kinematics::faraday_B4_base[1];
        _c=FARADAY_C;
    }
    template <typename T>
    bool operator()(const T* const p1, const T* const p2,
                    const T* const p3, const T* const gamma,
                    const T* const beta,T* residual) const
    {
        residual[0]=sqrt(pow(cos(beta[0])*_S4x+sin(beta[0])*sin(gamma[0])*_S4y+p1[0]-_U4x,2)+
                    pow(cos(gamma[0])*_S4y+p2[0]-_U4y,2)+
                    pow(-sin(beta[0])*_S4x+cos(beta[0])*sin(gamma[0])*_S4y+p3[0]-_b4,2))-_c;

        return true;
    }
    double _S4x,_S4y,_U4x,_U4y,_c;
    const double _b4;
};

struct F5
{
    explicit F5(double b5):_b5(b5),_d(106.0) {}
    template <typename T>
    bool operator()(const T* const p1, const T* const gamma,
                    const T* const beta, T* residual) const
    {
        residual[0]=T(_b5)+T(sin(beta[0]))*T(cos(gamma[0]))*T(_d)-p1[0];
        return true;
    }

    double _b5,_d;
};

// faraday_kinematics::default_initial_cartesian_value[5]=;

bool faraday_kinematics::forward_kinematics(const double actuator_mm[],double cartesian_mm[],
											double last_cartesian_mm[])
{
	//set iteration initial value
    // double p2=0.0,p3=180;  //180.0
    // double gamma=1.396263402,beta=0.0;

    double p1=last_cartesian_mm[0];
    double p2=last_cartesian_mm[1];
    double p3=last_cartesian_mm[2];
    double gamma=last_cartesian_mm[3];
    double beta=last_cartesian_mm[4];
//    double b[5]={40.0,40.0,40.0,40.0,0.0};

    ceres::Problem problem;

    problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<F1,1,1,1,1,1,1>(new F1(actuator_mm[0])), NULL,&p1,&p2,&p3,&gamma,&beta);
    problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<F2,1,1,1,1,1,1>(new F2(actuator_mm[1])), NULL,&p1,&p2,&p3,&gamma,&beta);
    problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<F3,1,1,1,1,1,1>(new F3(actuator_mm[2])), NULL,&p1,&p2,&p3,&gamma,&beta);
    problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<F4,1,1,1,1,1,1>(new F4(actuator_mm[3])), NULL,&p1,&p2,&p3,&gamma,&beta);
    problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<F5,1,1,1,1>(new F5(actuator_mm[4])), NULL,&p1,&gamma,&beta);
//    problem.AddResidualBlock(
//            new ceres::AutoDiffCostFunction<F6,1,1,1>(new F6(actuator_mm[4])), nullptr,&p2,&gamma);


    ceres::Solver::Options options;
//    options.trust_region_strategy_type=ceres::DOGLEG;     //set trust-region algorithm,default is Levenberg-Marquardt
    options.linear_solver_type=ceres::DENSE_QR;
    options.minimizer_progress_to_stdout=false; //可以控制是否显示迭代过程，false为不显示

    ceres::Solver::Summary summary;
//    chrono::steady_clock::time_point t1=chrono::steady_clock::now();
    ceres::Solve(options,&problem,&summary);
//    chrono::steady_clock::time_point t2=chrono::steady_clock::now();
//    chrono::duration<double> time_used=chrono::duration_cast<chrono::duration<double>>(t2-t1);
//    cout<<"solve time cost="<<time_used.count()<<"seconds."<<endl;

//    cout<<summary.BriefReport()<<endl;
    // std::cout<<summary.FullReport()<<std::endl;
//    cout<<"estimated p1,p2,p3,gamma,beta= ";
//    cout<<p1<<" "<<p2<<" "<<p3<<" "<<gamma<<" "<<beta<<" ";
//    cout<<endl;

    cartesian_mm[0]=p1;
    cartesian_mm[1]=p2;
    cartesian_mm[2]=p3;
    cartesian_mm[3]=gamma;
    cartesian_mm[4]=beta;

    for (int i = 0; i < 5; ++i)
    {
        if (std::isnan(cartesian_mm[i])) return false;
    }

    return true;
}

bool faraday_kinematics::calcIntermediatePoints(const double actuator_mm[],const double cartesian_mm[],
							IntermediatePoints& pts)
{
	using std::cos;
	using std::sin;

	//EulerAngles to RotationMatrix  绕动系yaw pitch roll
	Eigen::Vector3d EulerAngles(0.0,cartesian_mm[4],cartesian_mm[3]);
	Eigen::Matrix3d R;
	R=Eigen::AngleAxisd(EulerAngles[0],Eigen::Vector3d::UnitZ())
	 *Eigen::AngleAxisd(EulerAngles[1],Eigen::Vector3d::UnitY())
	 *Eigen::AngleAxisd(EulerAngles[2],Eigen::Vector3d::UnitX());

	 //变换矩阵中的平移部分
	Eigen::Vector3d P(cartesian_mm[0],cartesian_mm[1],cartesian_mm[2]);

	//计算动平台各点在固定坐标系中的坐标
	const Eigen::Vector3d faraday_S1_base=R*faraday_S1_move+P;
	const Eigen::Vector3d faraday_S2_base=R*faraday_S2_move+P;
	const Eigen::Vector3d faraday_S3_base=R*faraday_S3_move+P;
	const Eigen::Vector3d faraday_S4_base=R*faraday_S4_move+P;
	const Eigen::Vector3d faraday_S5_base=R*faraday_S5_move+P;

	Eigen::Vector3d A,B;

	// limb 1
	A[0]=faraday_B1_base[0];
	A[1]=faraday_B1_base[1];
	A[2]=actuator_mm[0];

	B=faraday_S1_base;

	pts.A.col(0)=A;
	pts.B.col(0)=B;

	// limb 2
	A[0]=faraday_B2_base[0];
	A[1]=faraday_B2_base[1];
	A[2]=actuator_mm[1];

	B=faraday_S2_base;

	pts.A.col(1)=A;
	pts.B.col(1)=B;

	// limb 3
	A[0]=faraday_B3_base[0];
	A[1]=faraday_B3_base[1];
	A[2]=actuator_mm[2];

	B=faraday_S3_base;

	pts.A.col(2)=A;
	pts.B.col(2)=B;

	// limb 4
	A[0]=faraday_B4_base[0];
	A[1]=faraday_B4_base[1];
	A[2]=actuator_mm[3];

	B=faraday_S4_base;

	pts.A.col(3)=A;
	pts.B.col(3)=B;

	// center limb
	pts.C=faraday_S5_base;


	return true;
}																	