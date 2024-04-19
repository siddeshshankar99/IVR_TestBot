#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

class ZigzagMoverWithFeedback {
public:
    ZigzagMoverWithFeedback();
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
    double angle_deg;
    int step; // To track the zigzag pattern steps

    void calculateVelocity(double& lin_x, double& lin_y, double distance, double angle_deg);
};

ZigzagMoverWithFeedback::ZigzagMoverWithFeedback() : has_start_position(false), start_x(0), start_y(0), current_x(0), current_y(0), step(0) {
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    odom_sub = nh.subscribe("odom", 50, &ZigzagMoverWithFeedback::odomCallback, this);

    goal_distance = 5.0; // meters, total linear distance goal
    move_speed = 1.16; // meters per second
    angle_deg = 30; // angle for zigzag movement
}

void ZigzagMoverWithFeedback::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;

    if (!has_start_position) {
        start_x = current_x;
        start_y = current_y;
        has_start_position = true;
    }
}

void ZigzagMoverWithFeedback::calculateVelocity(double& lin_x, double& lin_y, double distance, double angle_deg) {
    double angle_rad = angle_deg * (M_PI / 180.0); // Convert angle to radians
    lin_x = cos(angle_rad) * move_speed;
    lin_y = sin(angle_rad) * move_speed;
}

void ZigzagMoverWithFeedback::moveToGoal() {
    ros::Rate rate(10); // 10 Hz
    while(ros::ok() && !has_start_position) {
        ros::spinOnce();
        rate.sleep();
    }

    double step_start_x = start_x; // Start X for the current step
    double step_start_y = start_y; // Start Y for the current step
    double target_distance; // Distance to move in the current step
    double cumulative_distance_x = 0.0;
    double cumulative_distance_y = 0.0;
    int step = 0; // Current step number

    while(ros::ok()) {
        if(step == 0) {
            target_distance = 1.16; // Distance for the first step
        } else {
            target_distance = 2.32; // Distance for subsequent steps
        }
        double step_angle = (step % 2 == 0) ? 30 : -30;

        double lin_x, lin_y;
        // Calculate velocity based on the desired angle and move_speed
        calculateVelocity(lin_x, lin_y, target_distance, step_angle);

        // Publish the velocity
        cmd_vel_msg.linear.x = lin_x;
        cmd_vel_msg.linear.y = lin_y;
        vel_pub.publish(cmd_vel_msg);

        ros::spinOnce();
        rate.sleep();

        // Calculate the distance covered in the current step
        double distance_covered = sqrt(pow((current_x - step_start_x), 2) + pow((current_y - step_start_y), 2));
        

        // Check if the robot has moved the target distance
        if(distance_covered >= target_distance) {
            // Prepare for the next step
            step++;
            
            // Calculate the distance covered in the current step
            double distance_covered_x = current_x - step_start_x;
            double distance_covered_y = current_y - step_start_y;
            
            // Update cumulative distances
            cumulative_distance_x += distance_covered_x;
            cumulative_distance_y += distance_covered_y;

            step_start_x = current_x; // Update start position for the next step
            step_start_y = current_y;

            // Check if the goal has been reached to exit the loop
            if(cumulative_distance_x >= goal_distance) {
                break;
            }
        }
    }

    // Stop the robot
    cmd_vel_msg.linear.x = 0;
    cmd_vel_msg.linear.y = 0;
    vel_pub.publish(cmd_vel_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "zigzag_mover_with_feedback");
    ZigzagMoverWithFeedback mover;
    mover.moveToGoal();

    return 0;
}

