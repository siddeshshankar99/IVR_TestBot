#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

class SidewaysZigzagMover {
public:
    SidewaysZigzagMover();
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void performSidewaysZigzag();

private:
    ros::NodeHandle nh;
    ros::Publisher vel_pub;
    ros::Subscriber odom_sub;
    geometry_msgs::Twist cmd_vel_msg;

    double start_x, start_y; // Starting position
    double current_x, current_y; // Current position
    bool has_start_position;
    double step_distance; // Distance to move sideways in each step
    double move_speed; // Speed of sideways movement
    int step; // To track the zigzag pattern steps
    int cycles; // To track the number of completed cycles
};

SidewaysZigzagMover::SidewaysZigzagMover() : has_start_position(false), start_x(0), start_y(0), current_x(0), current_y(0), step(0), cycles(0) {
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    odom_sub = nh.subscribe("odom", 50, &SidewaysZigzagMover::odomCallback, this);

    step_distance = 1.29; // meters, distance for the first half of each sideways step
    move_speed = 1.16; // meters per second
}

void SidewaysZigzagMover::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;

    if (!has_start_position) {
        start_x = current_x;
        start_y = current_y;
        has_start_position = true;
    }
}

void SidewaysZigzagMover::performSidewaysZigzag() {
    ros::Rate rate(10); // 10 Hz
    while(ros::ok() && !has_start_position) {
        ros::spinOnce();
        rate.sleep();
    }

    double target_distance = step_distance;
    double step_start_y = start_y; // Start Y for the current step

    while(ros::ok() && cycles < 2) { // Stop after 2 cycles
        double lin_y = move_speed * ((step % 2 == 0) ? 1 : -1); // Alternate direction

        // Publish the velocity
        cmd_vel_msg.linear.y = lin_y;
        vel_pub.publish(cmd_vel_msg);

        ros::spinOnce();
        rate.sleep();

        // Calculate the distance covered in the current step
        double distance_covered_y = fabs(current_y - step_start_y);

        // Check if the robot has moved the target distance sideways
        if(distance_covered_y >= target_distance) {
            step++; // Increment step number
            step_start_y = current_y; // Update start position for the next step

            // Adjust target distance for the next step
            target_distance = (step % 2 == 0) ? step_distance : step_distance * 2;

            // Briefly stop the movement before changing direction
            cmd_vel_msg.linear.y = 0;
            vel_pub.publish(cmd_vel_msg);
            ros::Duration(0.5).sleep(); // Pause for half a second

            if(step % 2 == 0) { // A cycle is completed every two steps
                cycles++;
            }
        }
    }

    // Stop the robot after 2 cycles
    cmd_vel_msg.linear.y = 0;
    vel_pub.publish(cmd_vel_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sideways_zigzag_mover");
    SidewaysZigzagMover mover;
    mover.performSidewaysZigzag();

    return 0;
}

