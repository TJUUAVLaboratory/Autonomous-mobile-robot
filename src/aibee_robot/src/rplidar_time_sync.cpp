#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/LaserScan.h>

uint32_t hardwareClock;

void ClockCallback(const rosgraph_msgs::Clock::ConstPtr &tempClock)
{
    //封装mavlink包
    hardwareClock = tempClock->clock.sec * 1e4 + tempClock->clock.nsec / 1e5;
}

void Lidar_callback()

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_time_sync");
    ros::NodeHandle nh;

    //   ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ros::Subscriber clock_sub = nh.subscribe("/clock", 1, &ClockCallback);
    ros::Subscriber signalLidar_sub = nh.subscribe("/singalScan0", 1, &Lidar_callback);
    // ros::Publisher rplidar_pub = nh.advertise<>

    //ros::MultiThreadedSpinner s;
    ros::spin();

    return (0);
}
