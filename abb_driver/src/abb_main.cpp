#include "abb_driver/AbbRobot.h"



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "simple_node");
    AbbRobot robot("abb_controller/follow_joint_trajectory");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time time_current = ros::Time::now();
    ros::Time  time_prev = time_current;
    double elapsed = 0;
    while (ros::ok())
    {
        time_current = ros::Time::now();
        elapsed = (time_current - time_prev).toSec();
        if(elapsed >= 0.1)
        {
            time_prev = time_current;
            robot.jointStateUpdate();
        }
        usleep(5000);
    }
    cout << "\033[1m\033[32m机械臂程序正在关闭...\033[0m" << endl;
    return 0;
}