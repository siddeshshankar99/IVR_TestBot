#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h> // For converting orientations
#include <math.h>

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
    double start_yaw, current_yaw; // Starting and current orientation in radians
    bool has_start_position;
    double goal_distance;
    double move_speed; // Linear speed of the sideways movement
    double rotate_speed; // Angular speed
};

SidewaysMoverWithFeedback::SidewaysMoverWithFeedback() : has_start_position(false), start_x(0), start_y(0), current_x(0), current_y(0), start_yaw(0), current_yaw(0)
{
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    odom_sub = nh.subscribe("odom", 50, &SidewaysMoverWithFeedback::odomCallback, this);

    goal_distance = 2.0; // meters
    move_speed = 1.16; // meters per second, adjusted for circular movement
    rotate_speed = 1.5; // radians per second, adjust as needed

    // Initialize the twist message for circular movement
    cmd_vel_msg.linear.x = move_speed; // Set the speed for forward movement
    cmd_vel_msg.linear.y = 0; // No sideways movement for circular motion
    cmd_vel_msg.angular.z = rotate_speed; // Set the angular speed for rotation
}

void SidewaysMoverWithFeedback::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_yaw = yaw;

    if (!has_start_position) {
        start_x = current_x;
        start_y = current_y;
        start_yaw = yaw;
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

    while(ros::ok()) {
        double orientation_difference = fabs(current_yaw - start_yaw);

        // Adjust for crossing the -pi/pi boundary
        if (orientation_difference > M_PI)
            orientation_difference = 2 * M_PI - orientation_difference;

        if (orientation_difference >= M_PI/2) { // Approx. 90 degrees in radians
            cmd_vel_msg.linear.x = 0; // Stop the forward movement of the robot
            cmd_vel_msg.angular.z = 0; // Stop the rotation of the robot
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
    ros::init(argc, argv, "circular_mover_with_feedback");
    SidewaysMoverWithFeedback mover;
    mover.moveToGoal();

    return 0;
}

