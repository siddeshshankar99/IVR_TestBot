#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

class StraightMoverWithFeedback
{
public:
    StraightMoverWithFeedback();
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
    double move_speed;
};

StraightMoverWithFeedback::StraightMoverWithFeedback() : has_start_position(false), start_x(0), start_y(0), current_x(0), current_y(0)
{
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    odom_sub = nh.subscribe("odom", 50, &StraightMoverWithFeedback::odomCallback, this);

    goal_distance = 6.0; // meters
    move_speed = 1.16; // meters per second

    // Initialize the twist message
    cmd_vel_msg.linear.x = move_speed;
    cmd_vel_msg.linear.y = 0;
    cmd_vel_msg.linear.z = 0;
    cmd_vel_msg.angular.x = 0;
    cmd_vel_msg.angular.y = 0;
    cmd_vel_msg.angular.z = 0;
}

void StraightMoverWithFeedback::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;

    if (!has_start_position) {
        start_x = current_x;
        start_y = current_y;
        has_start_position = true;
    }
}

void StraightMoverWithFeedback::moveToGoal()
{
    ros::Rate rate(10); // 10 Hz
    while(ros::ok() && !has_start_position) {
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()) {
        double distance = sqrt(pow((current_x - start_x), 2) + pow((current_y - start_y), 2));

        if (distance >= goal_distance) {
            cmd_vel_msg.linear.x = 0; // Stop the robot
            vel_pub.publish(cmd_vel_msg);
            break;
        }

        vel_pub.publish(cmd_vel_msg);
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "straight_mover_with_feedback");
    StraightMoverWithFeedback mover;
    mover.moveToGoal();

    return 0;
}

