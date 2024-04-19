#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <chrono>

class SidewaysMoverWithFeedback
{
public:
    SidewaysMoverWithFeedback();
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void moveToGoal();

private:
    ros::NodeHandle nh;
    ros::Publisher vel_pub;
    ros::Subscriber odom_sub;
    geometry_msgs::Twist cmd_vel_msg;

    double start_x, start_y; // Starting position
    double current_x, current_y; // Current position
    bool has_start_position;
    double goal_distance;
    double move_speed; // Speed of the sideways movement
    int step_count;
    double pause_duration; // Duration to pause after each move in seconds
};

SidewaysMoverWithFeedback::SidewaysMoverWithFeedback() : has_start_position(false), start_x(0), start_y(0), current_x(0), current_y(0), step_count(0), pause_duration(0.56)
{
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    odom_sub = nh.subscribe("odom", 50, &SidewaysMoverWithFeedback::odomCallback, this);

    goal_distance = 0.65; // meters for each step
    move_speed = 0.65; // meters per second

    // Initialize the twist message for sideways movement
    cmd_vel_msg.linear.x = 0;
    cmd_vel_msg.linear.y = move_speed; // Set the speed for sideways movement
    cmd_vel_msg.linear.z = 0;
    cmd_vel_msg.angular.x = 0;
    cmd_vel_msg.angular.y = 0;
    cmd_vel_msg.angular.z = 0;
}

void SidewaysMoverWithFeedback::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;

    if (!has_start_position) {
        start_x = current_x;
        start_y = current_y;
        has_start_position = true;
    }
}

void SidewaysMoverWithFeedback::moveToGoal()
{
    ros::Rate rate(10); // 10 Hz
    while(ros::ok() && !has_start_position) {
        ros::spinOnce();
        rate.sleep();
    }

    auto last_pause_end_time = ros::Time::now();

    while(ros::ok() && step_count < 3) {
        double distance = sqrt(pow((current_x - start_x), 2) + pow((current_y - start_y), 2));

        if (distance >= goal_distance) {
            cmd_vel_msg.linear.y = 0; // Stop the sideways movement of the robot
            vel_pub.publish(cmd_vel_msg);

            step_count++;
            start_x = current_x; // Reset start position for the next step
            start_y = current_y;

            if (step_count < 3) {
                ros::Duration(pause_duration).sleep(); // Pause
            }
            continue; // Skip the rest of the loop to recheck condition
        }

        // Publish movement command only if enough time has passed since the last pause
        if ((ros::Time::now() - last_pause_end_time).toSec() > pause_duration) {
            cmd_vel_msg.linear.y = move_speed; // Ensure robot moves sideways
            vel_pub.publish(cmd_vel_msg);
        }

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sideways_mover_with_feedback");
    SidewaysMoverWithFeedback mover;
    mover.moveToGoal();

    return 0;
}

