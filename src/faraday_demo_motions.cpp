#include <ros/ros.h>
#include <faraday_kinematics/faraday_kinematics.h>
#include <trajectory_msgs/JointTrajectory.h>

static void populateHeader(std_msgs::Header& header)
{
	header.frame_id="base_link";
	header.stamp=ros::Time::now();
}

static trajectory_msgs::JointTrajectory makeCircleTrajectory()
{
	using namespace trajectory_msgs;
	// Header
	JointTrajectory traj;
	populateHeader(traj.header);

	//Create circle points
	const double r=50.0;
	const double dt=0.25;

	double pose[5];
	double joints[7];

	double total_t=dt;

	for(int i=0;i<361;++i)
	{
		pose[0]=r*std::cos(i*M_PI/180.0);
		pose[1]=r*std::sin(i*M_PI/180.0);
		pose[2]=250.0;
		pose[3]=0.0;
		pose[4]=0.0;

		JointTrajectoryPoint pt;
		if(!faraday_kinematics::inverse_kinematics(pose,joints))
		{
			std::cout<<"I am fool!\n";
			ROS_WARN_STREAM("Could not solve for: "<<pose[0]<<" "<<pose[1]<<" "<<pose[2]<<" "<<pose[3]<<" "<<" "<<pose[4]<<" "<<pose[5]);
		}
		else
		{
			pt.positions.assign(joints,joints+5);
			pt.time_from_start=ros::Duration(total_t);
			total_t+=dt;
			traj.points.push_back(pt);
		}
	}

	return traj;
}

//Linear trajectory helpers
struct FaradayPoint
{
	double joints[5];
};

struct FaradayPose
{
	FaradayPose() {}

	FaradayPose(double x,double y,double z)
	{
		pose[0]=x;
		pose[1]=y;
		pose[2]=z;
		pose[3]=0.0;
		pose[4]=0.0;
	}

	FaradayPose(double x,double y,double z,double roll,double pitch)
	{
		pose[0]=x;
		pose[1]=y;
		pose[2]=z;
		pose[3]=roll;
		pose[4]=pitch;
	}

	double pose[5];
};

FaradayPose interPose(const FaradayPose& start,const FaradayPose& stop,double ratio)
{
	FaradayPose result;
	result.pose[0]=start.pose[0]+ratio*(stop.pose[0]-start.pose[0]);
	result.pose[1]=start.pose[1]+ratio*(stop.pose[1]-start.pose[1]);
	result.pose[2]=start.pose[2]+ratio*(stop.pose[2]-start.pose[2]);
	result.pose[3]=start.pose[3]+ratio*(stop.pose[3]-start.pose[3]);
	result.pose[4]=start.pose[4]+ratio*(stop.pose[4]-start.pose[4]);

	return result;
}

bool linearMove(const FaradayPose& start,const FaradayPose& stop,double ds,std::vector<FaradayPoint>& out)
{
	std::vector<FaradayPoint> pts;	
	double delta_x=stop.pose[0]-start.pose[0];
	double delta_y=stop.pose[1]-start.pose[1];
	double delta_z=stop.pose[2]-start.pose[2];
	double delta_s=std::sqrt(delta_x*delta_x+delta_y*delta_y+delta_z*delta_z);

	unsigned steps=static_cast<unsigned>(delta_s/ds)+1;

	for (unsigned i = 0; i <= steps; i++)
	{
		double ratio=static_cast<double>(i)/steps;
		FaradayPose pose=interPose(start,stop,ratio);
		FaradayPoint pt;
		double joints[7];
		if(!faraday_kinematics::inverse_kinematics(pose.pose,joints))
		{
			return false;
		}
		for(int i=0;i<5;i++)
			pt.joints[i]=joints[i];
		pts.push_back(pt);
	}

	out=pts;

	return true;
}

std::vector<FaradayPoint> linearMove(const FaradayPose& start,const FaradayPose& stop,double ds)
{
	std::vector<FaradayPoint> pts;
	if(!linearMove(start,stop,ds,pts))
	{
		throw std::runtime_error("Couldn't plan for linear move");
	}

	return pts;
}

typedef std::vector<trajectory_msgs::JointTrajectoryPoint> TrajPointVec;

TrajPointVec toTrajPoints(const std::vector<FaradayPoint>& points,double time)
{
	TrajPointVec vec;
	double dt=time/points.size();
	double total_t=dt;

	for (size_t i = 0; i < points.size(); ++i)
	{
		trajectory_msgs::JointTrajectoryPoint pt;
		pt.positions.assign(points[i].joints,points[i].joints+5);
		pt.time_from_start=ros::Duration(total_t);
		total_t+=dt;

		vec.push_back(pt);
	}

	return vec;
}

//append b to a
TrajPointVec append(const TrajPointVec& a,const TrajPointVec& b)
{
	TrajPointVec result;
	result.reserve(a.size()+b.size());
	//insert a
	result.insert(result.end(),a.begin(),a.end());

	ros::Duration time_end_a=a.back().time_from_start;

	for (int i = 0; i < b.size(); ++i)
	{
		trajectory_msgs::JointTrajectoryPoint pt=b[i];
		pt.time_from_start+=time_end_a;
		result.push_back(pt);
	}

	return result;
}


static trajectory_msgs::JointTrajectory makeLineTrajectory()
{
	using namespace trajectory_msgs;
	//Header
	JointTrajectory traj;
	populateHeader(traj.header);

	std::vector<FaradayPoint> points;
	FaradayPose start(0.0, 0.0, 390.0);
	FaradayPose stop(0.0,0.0, 250.0);

	if(!linearMove(start,stop,5,points))
	{
		throw std::runtime_error("Linear movement planning failed");
	}

	const double dt=0.5;
	double total_t=dt;

	for (unsigned int i = 0; i < points.size(); ++i)
	{
		JointTrajectoryPoint pt;
		pt.positions.assign(points[i].joints,points[i].joints+5);
		pt.time_from_start=ros::Duration(total_t);
		total_t+=dt;
        traj.points.push_back(pt);
	}

	return traj;
}

// fix the move platform position, change its orientation
bool fixedPositionMotion(const FaradayPose& start,const FaradayPose& stop,double ds,std::vector<FaradayPoint>& out)
{
	std::vector<FaradayPoint> pts;	
	double delta_roll=std::fabs(stop.pose[3]-start.pose[3]);
	double delta_pitch=std::fabs(stop.pose[4]-start.pose[4]);
	double delta_radian;
	//calculate the maximum tilt angle
	if(delta_roll>=delta_pitch)
		delta_radian=delta_roll;
	else
		delta_radian=delta_pitch;

	unsigned steps=static_cast<unsigned>(delta_radian/ds)+1;

	for (unsigned i = 0; i <= steps; i++)
	{
		double ratio=static_cast<double>(i)/steps;
		FaradayPose pose=interPose(start,stop,ratio);

		// std::cout<<"pose: "<<pose.pose[0]<<" "<<pose.pose[1]<<" "<<pose.pose[2]<<" "<<pose.pose[3]<<" "<<pose.pose[4]<<std::endl;

		FaradayPoint pt;
		double joints[7];
		if(!faraday_kinematics::inverse_kinematics(pose.pose,joints))
		{
			return false;
		}

		for(int i=0;i<5;i++)
			pt.joints[i]=joints[i];
		pts.push_back(pt);

		// std::cout<<"pt: "<<pt.joints[0]<<" "<<pt.joints[1]<<" "<<pt.joints[2]<<" "<<pt.joints[3]<<" "<<pt.joints[4]<<std::endl;
	}

	out=pts;

	return true;
}

std::vector<FaradayPoint> fixedPositionMotion(const FaradayPose& start,const FaradayPose& stop,double ds)
{
	std::vector<FaradayPoint> pts;
	if(!fixedPositionMotion(start,stop,ds,pts))
	{
		throw std::runtime_error("Couldn't plan for linear move");
	}

	return pts;
}

// change pose from any where, just give the target pose
TrajPointVec singlePoint(const FaradayPose& pose,double dt)
{
	double joints[7];
	if(!faraday_kinematics::inverse_kinematics(pose.pose,joints))
	{
		throw std::runtime_error("Couldn't plan to point");
	}

	TrajPointVec v;
	trajectory_msgs::JointTrajectoryPoint pt;
	pt.positions.assign(joints,joints+5);
	pt.time_from_start=ros::Duration(dt);
	v.push_back(pt);

	// std::cout<<"pt: "<<pt.positions[0]<<" "<<pt.positions[1]<<" "<<pt.positions[2]<<" "<<pt.positions[3]<<" "<<pt.positions[4]<<std::endl;
	return v;
}

// test the fixed position motion
static trajectory_msgs::JointTrajectory makeFixedPositionMotionTrajectory()
{
	using namespace trajectory_msgs;
	//Header
	JointTrajectory traj;
	populateHeader(traj.header);

	std::vector<FaradayPoint> points;
	FaradayPose start(0.0, 0.0, 250.0, 0.0, 0.0);
	FaradayPose stop(0.0, 0.0, 250.0, -1.5, 1.0);

	if(!fixedPositionMotion(start,stop,0.01,points))
	{
		throw std::runtime_error("Fixed position motion planning failed");
	}

	const double dt=0.1;
	double total_t=dt;

	for (unsigned int i = 0; i < points.size(); ++i)
	{
		JointTrajectoryPoint pt;
		pt.positions.assign(points[i].joints,points[i].joints+5);
		pt.time_from_start=ros::Duration(total_t);
		total_t+=dt;
		traj.points.push_back(pt);
	}

	return traj;
}

// design a pick and place trajectory
static trajectory_msgs::JointTrajectory makePickPlaceTrajectory()
{
	trajectory_msgs::JointTrajectory traj;
	populateHeader(traj.header);

	const double LINEAR_MOVE_TIME=12.0;
	const double VERTICAL_MOVE_TIME=10.0;
	const double WAIT_PERIOD=5;

	//Home position
	// FaradayPose home_pt(0.0,0.0,300);
	FaradayPose home_pt(35.0, 35.0, 250);
	TrajPointVec vec=singlePoint(home_pt,5.0);

	//Pick spot 1
	// FaradayPose pick1(150,-300,350);
	FaradayPose pick1(-35.0, -35.0, 300);
	vec=append(vec,toTrajPoints(linearMove(home_pt,pick1,0.1),LINEAR_MOVE_TIME));

	//Down
	// FaradayPose pick1_down(150,-300,400);
	FaradayPose pick1_down(-35.0, -35.0, 350);
	vec=append(vec,toTrajPoints(linearMove(pick1,pick1_down,0.1),VERTICAL_MOVE_TIME));

	//Wait&Up
	vec=append(vec,singlePoint(pick1_down,WAIT_PERIOD));
	vec=append(vec,toTrajPoints(linearMove(pick1_down,pick1,0.1),VERTICAL_MOVE_TIME));

	//back home
	vec=append(vec,toTrajPoints(linearMove(pick1,home_pt,0.1),LINEAR_MOVE_TIME));

	traj.points=vec;

	return traj;	
}

// test the circle trajectory
static trajectory_msgs::JointTrajectory toHomeCircleTrajectory()
{
	trajectory_msgs::JointTrajectory traj;
	populateHeader(traj.header);

	//Home position
	FaradayPose home_pt(0.0, 0.0, 390.0);
	FaradayPose real_home(0.0, 0.0, 250.0);
	FaradayPose circle_st(50.0, 0.0, 250.0);

	TrajPointVec vec=singlePoint(home_pt,8.0);
	// vec=append(vec,singlePoint(home_pt,8.0));

	vec=append(vec,toTrajPoints(linearMove(home_pt,real_home,0.1),40.0));
	vec=append(vec,singlePoint(real_home,8.0));

	vec=append(vec,toTrajPoints(linearMove(real_home,circle_st,0.1),20.0));
	vec=append(vec,singlePoint(circle_st, 8.0));

	vec=append(vec,makeCircleTrajectory().points);
	vec=append(vec,singlePoint(circle_st,8.0));

	// 第一次启动需要将下面的注释掉，拆下原点校准块
	vec=append(vec,toTrajPoints(linearMove(circle_st,home_pt,0.1),60.0));
	vec=append(vec,singlePoint(home_pt,8.0));

	traj.points=vec;

	return traj;
}

// test the tilt angle trajectory
static trajectory_msgs::JointTrajectory testBigTiltAngle()
{
	trajectory_msgs::JointTrajectory traj;
	populateHeader(traj.header);

	const double MOVE_TIME=40.0;
	const double WAIT_PERIOD=8.0;

	FaradayPose home_pt(0.0, 0.0, 390.0);
	FaradayPose real_home(0.0, 0.0, 250.0);
	FaradayPose pick1(5.0, 45.0, 280.0);
	// FaradayPose pick1(0.0, 0.0, 250.0);

	TrajPointVec vec=singlePoint(home_pt,WAIT_PERIOD);	//important
	vec=append(vec,toTrajPoints(linearMove(home_pt,real_home,0.1),MOVE_TIME));
	vec=append(vec,singlePoint(real_home,WAIT_PERIOD));

	vec=append(vec,toTrajPoints(linearMove(real_home,pick1,0.1),MOVE_TIME/1.5));
	vec=append(vec,singlePoint(pick1,WAIT_PERIOD));

	FaradayPose pick2(5.0, 45.0, 280.0, -1.5, 0.0);
	// FaradayPose pick2(0.0, 0.0, 250.0, -1.5, 1.0);
	vec=append(vec,toTrajPoints(fixedPositionMotion(pick1,pick2,0.001),MOVE_TIME));
	vec=append(vec,singlePoint(pick2,WAIT_PERIOD));

	vec=append(vec,toTrajPoints(fixedPositionMotion(pick2,pick1,0.001),MOVE_TIME));
	vec=append(vec,singlePoint(pick1,WAIT_PERIOD));

	vec=append(vec,toTrajPoints(linearMove(pick1,home_pt,0.1),MOVE_TIME));

	
	traj.points=vec;

	return traj;
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"faraday_demo_motions");

	ros::NodeHandle nh;
	ros::Publisher traj_pub=nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command",16);

	// trajectory_msgs::JointTrajectory traj=makeLineTrajectory();
	// trajectory_msgs::JointTrajectory traj=makeCircleTrajectory();
	// trajectory_msgs::JointTrajectory traj=makePickPlaceTrajectory();
	trajectory_msgs::JointTrajectory traj=toHomeCircleTrajectory();
	// trajectory_msgs::JointTrajectory traj=testBigTiltAngle();
	// trajectory_msgs::JointTrajectory traj=makeFixedPositionMotionTrajectory();


	std::vector<std::string> names;
	names.push_back("joint1");
	names.push_back("joint2");
	names.push_back("joint3");
	names.push_back("joint4");
	names.push_back("joint5");
	// names.push_back("joint6");
	// names.push_back("joint7");

	traj.joint_names=names;
	ros::Duration(0.5).sleep();

	traj_pub.publish(traj);

	// ros::spin();
	ros::waitForShutdown();
}
