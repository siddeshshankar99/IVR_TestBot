#include <memory>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include "ugv_sdk/mobile_robot/scout_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"
#include "scout_base/scout_messenger.hpp"

using namespace westonrobot;

std::unique_ptr<ScoutRobot> robot;

int main(int argc, char **argv) {
  // Setup ROS node
  ros::init(argc, argv, "scout_odom");
  ros::NodeHandle node(""), private_node("~");

  // Check whether controlling scout mini or omni
  bool is_scout_mini = false;
  bool is_scout_omni = false;
  bool simulated_robot = false;

  private_node.getParam("is_scout_mini", is_scout_mini);
  ROS_INFO("Working as scout mini: %d", is_scout_mini);

  private_node.getParam("is_scout_omni", is_scout_omni);
  ROS_INFO("Working as scout omni: %d", is_scout_omni);

  // Fetch simulated_robot parameter first to decide on the mode of operation
  private_node.getParam("simulated_robot", simulated_robot);
  ROS_INFO("Working as simulated omni: %d", simulated_robot);

  if (!simulated_robot) {
    // Fetch port_name parameter only when not in simulation mode
    std::string port_name;
    private_node.param<std::string>("port_name", port_name, "can0");

    // Check protocol version if not in simulation mode
    ProtocolDetector detector;
    try
  {
      detector.Connect("can0");
      auto proto = detector.DetectProtocolVersion(5);
      if(is_scout_mini && is_scout_omni)
      {
          if (proto == ProtocolVersion::AGX_V1) {
              std::cout << "Detected protocol: AGX_V1 omni" << std::endl;
              robot = std::unique_ptr<ScoutMiniOmniRobot>(
                          new ScoutMiniOmniRobot(ProtocolVersion::AGX_V1));
          } else if (proto == ProtocolVersion::AGX_V2) {
              std::cout << "Detected protocol: AGX_V2 omni" << std::endl;
              robot = std::unique_ptr<ScoutMiniOmniRobot>(
                          new ScoutMiniOmniRobot(ProtocolVersion::AGX_V2));
          } else {
              ROS_ERROR("Detected protocol: UNKONWN");
              //return -1;
          }
      }
      else
      {
          if (proto == ProtocolVersion::AGX_V1) {
              std::cout << "Detected protocol: AGX_V1" << std::endl;
              robot = std::unique_ptr<ScoutRobot>(
                          new ScoutRobot(ProtocolVersion::AGX_V1, is_scout_mini));
          } else if (proto == ProtocolVersion::AGX_V2) {
              std::cout << "Detected protocol: AGX_V2" << std::endl;
              robot = std::unique_ptr<ScoutRobot>(
                          new ScoutRobot(ProtocolVersion::AGX_V2, is_scout_mini));
          } else {
              ROS_ERROR("Detected protocol: UNKONWN");
              //return -1;
          }
      }
      if (robot == nullptr)
          ROS_ERROR("Failed to create robot object");
  }
  catch (const std::exception error)
  {
      ROS_ERROR("please bringup up can or make sure can port exist");
      ros::shutdown();
  } 
  } else {
    ROS_INFO("Operating in simulation mode, bypassing hardware initialization.");
  }

  ScoutROSMessenger messenger(nullptr, &node, is_scout_omni);
  ROS_INFO("scout messenger running successfully");
  
  // Fetch and set up other necessary parameters and subscriptions
  // fetch parameters before connecting to rt
  private_node.param<std::string>("odom_frame", messenger.odom_frame_,
                                  std::string("odom"));
  private_node.param<std::string>("base_frame", messenger.base_frame_,
                                  std::string("base_link"));                       
  private_node.param<bool>("simulated_robot", messenger.simulated_robot_, true);
  private_node.param<int>("control_rate", messenger.sim_control_rate_, 50);
  private_node.param<std::string>("odom_topic_name", messenger.odom_topic_name_, std::string("/odom"));
  private_node.param<bool>("pub_tf", messenger.pub_tf,true);
  

  // Call SetupSubscription to setup publishers and subscribers
  messenger.SetupSubscription();

  ros::Rate rate(50);

  while (ros::ok()) {
    if (!simulated_robot) {
      // Publish robot state for a physical robot
      messenger.PublishStateToROS();

    } else {
      // Simulate and publish robot state for a simulated robot
      double linear, angular;

      messenger.GetCurrentMotionCmdForSim(linear, angular);
      messenger.PublishSimStateToROS(linear, angular);
      
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

