#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <chrono>
#include <pthread.h>
#include <geometry_msgs/Twist.h>

using namespace UNITREE_LEGGED_SDK;
class Custom
{
public:
    UDP low_udp;
    LowCmd low_cmd = {0};
    LowState low_state = {0};

public:
    Custom()
        : 
        low_udp(LOWLEVEL, 8091, "192.168.123.10", 8007)
    {
        low_udp.InitCmdData(low_cmd);
    }
    void lowUdpSend()
    {
        low_udp.SetSend(low_cmd);
        low_udp.Send();
    }
    void lowUdpRecv()
    {
        low_udp.Recv();
        low_udp.GetRecv(low_state);
    }
};

Custom custom;

ros::Subscriber sub_low;
ros::Publisher pub_low;
long low_count = 0;

void lowCmdCallback(const unitree_legged_msgs::LowCmd::ConstPtr &msg)
{
    custom.low_cmd = rosMsg2Cmd(msg);
}

void publishLowState()
{
    pub_low.publish(state2rosMsg(custom.low_state));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_udp_low_fast");

    ros::NodeHandle nh;

    sub_low = nh.subscribe("low_cmd", 100, lowCmdCallback, ros::TransportHints().tcpNoDelay(true));
    pub_low = nh.advertise<unitree_legged_msgs::LowState>("low_state", 1);

    LoopFunc loop_udpSend("low_udp_send", 0.002, 3, boost::bind(&Custom::lowUdpSend, &custom));
    LoopFunc loop_udpRecv("low_udp_recv", 0.002, 3, boost::bind(&Custom::lowUdpRecv, &custom));

    loop_udpSend.start();
    loop_udpRecv.start();

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Rate rate(500);

    while (ros::ok())
    {
        publishLowState();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
