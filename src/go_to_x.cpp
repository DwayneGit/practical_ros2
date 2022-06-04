#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "turtlesim/msg/pose.hpp"
#include <cstdlib>
#include <iostream>
#include <string.h>
#include <memory>

using namespace std;

geometry_msgs::msg::Twist cmdVel;
geometry_msgs::msg::Pose2D current;
geometry_msgs::msg::Pose2D desired;

const double GOAL = -2.5;
const double K1 = 1;
const double distanceTolerance = 0.1;

void misc_setup()
{
    desired.x = GOAL;
    cmdVel.linear.x = 0;
    cmdVel.linear.y = 0;
    cmdVel.linear.z = 0;
    cmdVel.angular.x = 0;
    cmdVel.angular.y = 0;
    cmdVel.angular.z = 0;
}

double getDistanceError()
{
    return desired.x - current.x;
}

void set_velocity(rclcpp::Logger logger)
{
    if (abs(getDistanceError()) > distanceTolerance)
    {
        cmdVel.linear.x = K1 * getDistanceError();
    }
    else
    {
        RCLCPP_INFO(logger, "I'm HERE!");
        cmdVel.linear.x = 0;
    }
}

class PracticalOne : public rclcpp::Node
{
public:
    PracticalOne() : Node("go_to_x")
    {
        sub_current_pose_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 0, std::bind(&PracticalOne::update_pose, this, std::placeholders::_1));
        pub_velocity_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 0);

        set_velocity(get_logger());
        pub_velocity_->publish(cmdVel);

        char update[150];
        sprintf(update, "goal x = %f \ncurrent x = %f \ndisError = %f\ncmd_vel = %f",
                desired.x, current.x, getDistanceError(), cmdVel.linear.x);
        RCLCPP_INFO(get_logger(), update);
    }

private:
    void update_pose(const turtlesim::msg::Pose &currentPose)
    {
        current.x = currentPose.x;
        current.y = currentPose.y;
        current.theta = currentPose.theta;
    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_velocity_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_current_pose_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Rate loop_rate(10);
    misc_setup();
    while (rclcpp::ok)
    {
        rclcpp::spin(std::make_shared<PracticalOne>());
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
