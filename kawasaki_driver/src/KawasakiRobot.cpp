#include "kawasaki_driver/KawasakiRobot.h"

//传入action的名称
KawasakiRobot::KawasakiRobot( string name ) 
    : as_(nh_, name, boost::bind(&KawasakiRobot::executeCB, this,  _1), false) ,joint_count(7)       
{
    //动作名
    action_name = name;
    ros::NodeHandle nh_private("~");
    

    //初始化关节变量
    joint_name.resize(joint_count);
    msg.name.resize(joint_count);
    msg.position.resize(joint_count);
    msg.velocity.resize(joint_count);
    msg.effort.resize(joint_count);
    msg.header.frame_id = "/kawasaki";


    //关节命名
    stringstream ss; ss.clear(); ss.str("");
    for (size_t i = 0; i < joint_count; i++)
    {
        ss << "joint" << i + 1;
        joint_name[i] = ss.str();
        msg.name[i] = joint_name[i];
        ss.clear();ss.str("");
    }

    /*通讯等初始化开始*/
    kawasakiManipulator_.axes = joint_count;
    KawasakiManipulatorInit(kawasakiManipulator_);

    //cout << "kawasakiManipulator_.jointList.size() " << kawasakiManipulator_.jointList.size() << endl;
    /*通讯等初始化结束*/

    //关节发布者初始化
    joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/kawasaki/joint_states", 1);

    //定时器启动，定时器周期为0.1秒
    //period = 0.1;
    //timer = nh_.createTimer(ros::Duration(period), &KawasakiRobot::timerCallback, this);

    //服务端启动
    as_.start();
}

KawasakiRobot::~KawasakiRobot()
{
    //释放资源
    cout << "\033[32m机械臂程序已退出，请等待，并关掉电源，以免电机过热。\033[0m"<< endl;
}


/*
//timer回调函数，用于接收下位机数据
void KawasakiRobot::timerCallback(const ros::TimerEvent& e)
{

}*/

//goal回调函数
void KawasakiRobot::executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
    cout << "\033[1m\033[32mmexecuteCB\033[0m : " << endl;
    //调整关节顺序
    reorder(goal->trajectory);

    //数据发送给下位机，下位机用于执行
    trackMove();

    //动作完成，反馈结果，设置完成状态
    result_.error_code = result_.SUCCESSFUL;
    as_.setSucceeded(result_);

    /*动作未完成
    result_.error_code = result_.GOAL_TOLERANCE_VIOLATED;
    as_.setAborted(result_);
    */
}

//发布者函数
void KawasakiRobot::jointStateUpdate()
{
    //cout << "jointStateUpdate " << endl;
    kawasakiRead();
    msg.header.stamp = ros::Time::now();
    joint_pub_.publish(msg);
}

//重排序(参考)
void KawasakiRobot::reorder(trajectory_msgs::JointTrajectory trajectory)
{
    feedback_.header.frame_id = trajectory.header.frame_id;
    //添加路点到容器中(参考)
    cout << "\033[1m\033[35mPoints count : \033[0m" << trajectory.points.size() << endl;
    for (size_t seq = 0; seq < trajectory.points.size(); seq ++)
    {
        waypoints.push_back(trajectory.points[seq]);
    }

    //清空
    feedback_.joint_names.clear();

    //根据名称进行排序  (参考)
    for (size_t index = 0; index < joint_count; index ++)
    {//joint_names中，逐个索引
        const char *p = trajectory.joint_names[index].c_str();
        int degree_id = 0;
        for (size_t i = 0; i < trajectory.joint_names[index].length(); i ++)
        {//string -> int。 将编号提取出来
            degree_id = degree_id * 10 + (int)(p[i + 5] - '0');
        }
        feedback_.joint_names.push_back(trajectory.joint_names[index]);
    }
}

//路径执行
void KawasakiRobot::trackMove()
{
    cout << "\033[1m\033[32mTrackMoving \033[0m" << endl;
    feedback_.desired.positions.clear();

    //将路点的终点写入feedback_中
    for (size_t i = 0; i < joint_count; i++)
    {
        feedback_.desired.positions.push_back(waypoints[waypoints.size() - 1].positions[i]);
    }

    //写入虚拟下位机，且下位机按照moveit要求执行运动
    for (size_t seq = 0; seq < waypoints.size(); seq ++)
    {
        kawasakiWrite(waypoints[seq]);
        //当前路径点的反馈,按照每个关节进行写入
        for (size_t i = 0; i < joint_count; i++)
        {   
            feedback_.actual.positions.push_back(msg.position[i]);
            feedback_.actual.velocities.push_back(msg.velocity[i]);
            feedback_.actual.effort.push_back(msg.effort[i]);
        }
        feedback_.header.stamp = msg.header.stamp;
        as_.publishFeedback(feedback_);

        //10hz速率进行反馈
        usleep(100000);  
    }

    //路径点执行完毕
    cout << "\033[1m\033[32mstop Moving\033[0m" << endl;
    waypoints.clear();
}


//数据写入下位机并执行(参考)
void KawasakiRobot::kawasakiWrite(trajectory_msgs::JointTrajectoryPoint point)
{
    //cout << "kawasakiWrite " << endl;
    //cout << point.positions.size() << " " << point.velocities.size() << " " << point.accelerations.size() << " " << point.effort.size() << endl;
    for (int i = 0; i < joint_count; i++)
    {
        
        kawasakiManipulator_.jointList[i].position = point.positions[i];
        kawasakiManipulator_.jointList[i].velocity = point.velocities[i];
        kawasakiManipulator_.jointList[i].accelerations = point.accelerations[i];
        //kawasakiManipulator_.jointList[i].effort = point.effort[i];
        
    }
    //cout << "kawasakiWrite exit" << endl;
}

//从下位机读取数据(参考)
void KawasakiRobot::kawasakiRead()
{
    //cout << "kawasakiRead\n";
    for (size_t i = 0; i < joint_count; i++)
    {   
        
        msg.position[i] = kawasakiManipulator_.jointList[i].position;
        msg.velocity[i] = kawasakiManipulator_.jointList[i].velocity;
        msg.effort[i] = kawasakiManipulator_.jointList[i].effort;
        kawasakiManipulator_.jointList[i].accelerations = 0;
    }
    //cout << "kawasakiRead exit\n";
}