#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <vector>

class OmniMoverWithWaypoints
{
public:
    OmniMoverWithWaypoints();
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void moveToWaypoints();

private:
    ros::NodeHandle nh;
    ros::Publisher vel_pub;
    ros::Subscriber odom_sub;
    geometry_msgs::Twist cmd_vel_msg;

    std::vector<std::pair<double, double>> waypoints; // List of waypoints
    size_t current_waypoint_index;
    double current_x, current_y; // Current position
    double move_speed;
    double waypoint_tolerance;

    void moveToNextWaypoint();
    double distanceToWaypoint(double waypoint_x, double waypoint_y);
};

OmniMoverWithWaypoints::OmniMoverWithWaypoints() : current_waypoint_index(0), current_x(0), current_y(0)
{
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    odom_sub = nh.subscribe("odom", 50, &OmniMoverWithWaypoints::odomCallback, this);

    waypoints.push_back(std::make_pair(0.0, 0.0));  // Starting point (assumed)
    waypoints.push_back(std::make_pair(2.0, 2.0));
    waypoints.push_back(std::make_pair(-1.0, 3.0));
    waypoints.push_back(std::make_pair(0.5, -4.0));
    waypoints.push_back(std::make_pair(4.0, 4.0));

    move_speed = 1.16; // meters per second
    waypoint_tolerance = 0.3; // meters
}

void OmniMoverWithWaypoints::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;
}

void OmniMoverWithWaypoints::moveToWaypoints()
{
    ros::Rate rate(10); // 10 Hz
    while(ros::ok() && current_waypoint_index < waypoints.size()) {
        moveToNextWaypoint();
        ros::spinOnce();
        rate.sleep();
    }
}

void OmniMoverWithWaypoints::moveToNextWaypoint()
{
    if (current_waypoint_index >= waypoints.size())
        return;

    auto& target = waypoints[current_waypoint_index];
    double distance = distanceToWaypoint(target.first, target.second);

    if (distance < waypoint_tolerance) {
        current_waypoint_index++; // Move to the next waypoint
        if (current_waypoint_index >= waypoints.size()) {
            cmd_vel_msg.linear.x = 0;
            cmd_vel_msg.linear.y = 0;
            vel_pub.publish(cmd_vel_msg); // Stop the robot
            return;
        }
    }

    // Calculate direction to the waypoint
    double angle = atan2(target.second - current_y, target.first - current_x);
    cmd_vel_msg.linear.x = move_speed * cos(angle);
    cmd_vel_msg.linear.y = move_speed * sin(angle);

    vel_pub.publish(cmd_vel_msg);
}

double OmniMoverWithWaypoints::distanceToWaypoint(double waypoint_x, double waypoint_y)
{
    return sqrt(pow(waypoint_x - current_x, 2) + pow(waypoint_y - current_y, 2));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "omni_mover_with_waypoints");
    OmniMoverWithWaypoints mover;
    mover.moveToWaypoints();

    return 0;
}
