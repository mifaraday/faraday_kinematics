#include <ros/ros.h>
#include <faraday_kinematics/faraday_kinematics.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Header.h>

typedef std::vector<trajectory_msgs::JointTrajectoryPoint> TrajPointVec;

static void populateHeader(std_msgs::Header& header)
{
    header.frame_id="base_link";
    header.stamp=ros::Time::now();
}

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

static trajectory_msgs::JointTrajectory makeCircleTrajectory()
{
    using namespace trajectory_msgs;

    //Header 
    JointTrajectory traj;
    populateHeader(traj.header);

    // Create circle points
    const double r=30.0;
    const double dt=0.25;   //0.25

    double pose[5];

    double total_t=dt;

    // make the circle into 360 points
    for(int i=0;i<361;++i)
    {
        pose[0]=r*std::cos(i*M_PI/180);
        pose[1]=r*std::sin(i*M_PI/180);
        pose[2]=250.0;
        pose[3]=0.0;
        pose[4]=0.0;

//                pose[0]=r*std::cos(i*M_PI/180);
//                pose[1]=0.0;
//                pose[2]=280.0+r*std::sin(i*M_PI/180);
//                pose[3]=1.56;
//                pose[4]=0.0;

        // those point are still in the cartesian space
        JointTrajectoryPoint pt;
        pt.positions.assign(pose,pose+5);
        pt.time_from_start=ros::Duration(total_t);
        total_t+=dt;
        traj.points.push_back(pt);
    }

    return traj;
}

bool linearMove(const FaradayPose& start,const FaradayPose& stop,double ds,std::vector<FaradayPose>& out)
{
	std::vector<FaradayPose> pts;	
	double delta_x=stop.pose[0]-start.pose[0];
	double delta_y=stop.pose[1]-start.pose[1];
	double delta_z=stop.pose[2]-start.pose[2];
	double delta_s=std::sqrt(delta_x*delta_x+delta_y*delta_y+delta_z*delta_z);

	unsigned steps=static_cast<unsigned>(delta_s/ds)+1;

	for (unsigned i = 0; i <= steps; i++)
	{
		double ratio=static_cast<double>(i)/steps;
		FaradayPose pose=interPose(start,stop,ratio);

		pts.push_back(pose);
	}

	out=pts;

	return true;
}

std::vector<FaradayPose> linearMove(const FaradayPose& start,const FaradayPose& stop,double ds)
{
	std::vector<FaradayPose> pts;
	if(!linearMove(start,stop,ds,pts))
	{
		throw std::runtime_error("Couldn't plan for linear move");
	}

	return pts;
}

TrajPointVec toTrajPoints(const std::vector<FaradayPose>& points,double time)
{
	TrajPointVec vec;
	double dt=time/points.size();
	double total_t=dt;

	for (size_t i = 0; i < points.size(); ++i)
	{
		trajectory_msgs::JointTrajectoryPoint pt;
		pt.positions.assign(points[i].pose,points[i].pose+5);
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

	std::vector<FaradayPose> points;
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
		pt.positions.assign(points[i].pose,points[i].pose+5);
		pt.time_from_start=ros::Duration(total_t);
		total_t+=dt;
		traj.points.push_back(pt);
	}

	return traj;
}

// fix the move platform position, change its orientation
bool fixedPositionMotion(const FaradayPose& start,const FaradayPose& stop,double ds,std::vector<FaradayPose>& out)
{
	std::vector<FaradayPose> pts;	
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

        pts.push_back(pose);
        
	}

	out=pts;

	return true;
}

std::vector<FaradayPose> fixedPositionMotion(const FaradayPose& start,const FaradayPose& stop,double ds)
{
	std::vector<FaradayPose> pts;
	if(!fixedPositionMotion(start,stop,ds,pts))
	{
		throw std::runtime_error("Couldn't plan for linear move");
	}

	return pts;
}

// test the fixed position motion
static trajectory_msgs::JointTrajectory makeFixedPositionMotionTrajectory()
{
	using namespace trajectory_msgs;
	//Header
	JointTrajectory traj;
	populateHeader(traj.header);

	std::vector<FaradayPose> points;
	FaradayPose start(0.0, 0.0, 250.0, 0.0, 0.0);
	FaradayPose stop(0.0, 0.0, 250.0, -1.5, 0.0);

	if(!fixedPositionMotion(start,stop,0.01,points))
	{
		throw std::runtime_error("Fixed position motion planning failed");
	}

	const double dt=0.1;
	double total_t=dt;

	for (unsigned int i = 0; i < points.size(); ++i)
	{
		JointTrajectoryPoint pt;
		pt.positions.assign(points[i].pose,points[i].pose+5);
		pt.time_from_start=ros::Duration(total_t);
		total_t+=dt;
		traj.points.push_back(pt);
	}

	return traj;
}

// change pose from any where, just give the target pose
TrajPointVec singlePoint(const FaradayPose& target_pose,double dt)
{
	TrajPointVec v;
	trajectory_msgs::JointTrajectoryPoint pt;
	pt.positions.assign(target_pose.pose,target_pose.pose+5);
	pt.time_from_start=ros::Duration(dt);
	v.push_back(pt);

	return v;
}

// test the circle trajectory
static trajectory_msgs::JointTrajectory toHomeCircleTrajectory()
{
	trajectory_msgs::JointTrajectory traj;
	populateHeader(traj.header);

	//Home position
    FaradayPose home_pt(0.0, 0.0, 310.0);
	FaradayPose real_home(0.0, 0.0, 250.0);
    FaradayPose circle_st(30.0, 0.0, 250.0);

    TrajPointVec vec=singlePoint(home_pt,2.0);
	// vec=append(vec,singlePoint(home_pt,8.0));

	vec=append(vec,toTrajPoints(linearMove(home_pt,real_home,0.1),40.0));
//	vec=append(vec,singlePoint(real_home,8.0));

	vec=append(vec,toTrajPoints(linearMove(real_home,circle_st,0.1),20.0));
//	vec=append(vec,singlePoint(circle_st, 8.0));

	vec=append(vec,makeCircleTrajectory().points);
//	vec=append(vec,singlePoint(circle_st,8.0));

	// 第一次启动需要将下面的注释掉，拆下原点校准块
	vec=append(vec,toTrajPoints(linearMove(circle_st,home_pt,0.1),60.0));
    vec=append(vec,singlePoint(home_pt,2.0));

	traj.points=vec;

	return traj;
}

// test the tilt angle trajectory
static trajectory_msgs::JointTrajectory testBigTiltAngle()
{
    trajectory_msgs::JointTrajectory traj;
    populateHeader(traj.header);

    const double MOVE_TIME=40.0;
    const double WAIT_PERIOD=2.0;

    FaradayPose home_pt(0.0, 0.0, 310.0);
    FaradayPose real_home(0.0, 0.0, 250.0);
    FaradayPose pick1(5.0, 45.0, 280.0);
    FaradayPose pick2(5.0, 45.0, 280.0, -1.5, 0.0);

    TrajPointVec vec=singlePoint(home_pt,WAIT_PERIOD);	//important

    vec=append(vec,toTrajPoints(linearMove(home_pt,real_home,0.1),MOVE_TIME));

    vec=append(vec,toTrajPoints(linearMove(real_home,pick1,0.1),MOVE_TIME/1.5));

    vec=append(vec,toTrajPoints(fixedPositionMotion(pick1,pick2,0.001),MOVE_TIME));

    vec=append(vec,singlePoint(pick2,WAIT_PERIOD*8));

    vec=append(vec,toTrajPoints(fixedPositionMotion(pick2,pick1,0.001),MOVE_TIME));

    vec=append(vec,toTrajPoints(linearMove(pick1,home_pt,0.1),MOVE_TIME));

    vec=append(vec,singlePoint(home_pt,WAIT_PERIOD));

    traj.points=vec;

    return traj;

}

// 在U5（0， 0， 234）处实现自旋转运动
std::vector<FaradayPose> selfRotation(const double theta)
{
	std::vector<FaradayPose> pts;	

	for(int i = 0; i <= 360; i++)
	{
		double phi =  2.0 * M_PI * i / 360.0;
		double x = std::sin(phi) * std::sin(theta) * 76.0;
		double y = -std::cos(phi) * std::sin(theta) * 76.0;
		double z = 234.0 + std::cos(theta) * 76.0;
		double roll = std::asin(std::sin(theta) * cos(phi));
		double pitch = std::atan(std::tan(theta) * sin(phi));
		FaradayPose pose(x, y, z, roll, pitch);

		pts.push_back(pose);
	}

	return pts;
}
// 注意这个运动目前还没有从机床初始状态形成连续轨迹
static trajectory_msgs::JointTrajectory testSelfRotation()
{
    trajectory_msgs::JointTrajectory traj;
    populateHeader(traj.header);


    TrajPointVec vec=toTrajPoints(selfRotation(M_PI/6), 10.0);

	traj.points=vec;

    return traj;
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"faraday_cartesian_motion");

    ros::NodeHandle nh;
    ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1000);
    
//    trajectory_msgs::JointTrajectory traj=makeCircleTrajectory();

    // trajectory_msgs::JointTrajectory traj=toHomeCircleTrajectory();

//     trajectory_msgs::JointTrajectory traj=makeFixedPositionMotionTrajectory();

//    trajectory_msgs::JointTrajectory traj=testBigTiltAngle();

	trajectory_msgs::JointTrajectory traj=testSelfRotation();

    std::vector<std::string> names;
    names.push_back("joint_1");
    names.push_back("joint_2");
    names.push_back("joint_3");
    names.push_back("joint_4");
    names.push_back("joint_5");

    traj.joint_names=names;
    ros::Duration(0.5).sleep();

    traj_pub.publish(traj);

    ros::waitForShutdown();
    
    return 0;
}
