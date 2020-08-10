#ifndef KAWASAKI_ROBOT_H
#define KAWASAKI_ROBOT_H

#include "kawasaki_driver/common.h"


struct KawasakiJoint
{
	float position;			//位置
	float velocity;			//速度
	float accelerations;	//加速度
	float effort;			//力
};

struct KawasakiManipulator
{
	int axes;	//自由度
	std::vector<KawasakiJoint> jointList;		//关节列表
};
 
//初始化
void static KawasakiManipulatorInit(KawasakiManipulator & kawasakiManipulator)
{
	KawasakiJoint joint = { 0 , 0 , 0 , 0 };
	for (size_t i = 0; i < kawasakiManipulator.axes; i++)
	{
		kawasakiManipulator.jointList.push_back(joint);
	}
};


class KawasakiRobot
{
public:
	//传入action的名称
	KawasakiRobot( string name );       
	virtual ~KawasakiRobot();

	/*
	//timer回调函数，用于接收下位机数据
    void timerCallback(const ros::TimerEvent& e);
    */

    //goal回调函数
    void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

	//发布消息
	void jointStateUpdate();

	//重排序
	void reorder(trajectory_msgs::JointTrajectory trajectory);

	//路径执行
	void trackMove();

	//数据写入下位机并执行
	void kawasakiWrite(trajectory_msgs::JointTrajectoryPoint point);

	//从下位机读取数据
	void kawasakiRead();

private:

	//自由度
    const int joint_count;
    std::vector<string> joint_name;

    //句柄实例
    ros::NodeHandle nh_;

    //ros系统时间
	ros::Time time_current, time_prev;

    //ros定时器
    //ros::Timer timer;

	//定时器周期
    double period;
	
	//action名称
    string action_name;

    //定义action服务端实例
	//actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>  as_;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>  as_;

    //反馈实例
    control_msgs::FollowJointTrajectoryFeedback feedback_;

    //用来反馈action目标的执行情况，客户端由此可以得知服务端是否执行成功了
    control_msgs::FollowJointTrajectoryResult result_; 

	//路径点容器
    vector<trajectory_msgs::JointTrajectoryPoint> waypoints;

    //关节状态发布者 消息实例
    ros::Publisher joint_pub_;
    sensor_msgs::JointState msg;

	//虚拟下位机机器人实例
	KawasakiManipulator kawasakiManipulator_;
};

#endif // KAWASAKI_ROBOT_H