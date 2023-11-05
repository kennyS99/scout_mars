// /*
//  * scout_robot_demo.cpp
//  *
//  * Created on: Jul 08, 2021 11:12
//  * Description:
//  *
//  * Copyright (c) 2021 Weston Robot Pte. Ltd.
//  */

// #include <unistd.h>
// #include <ros/ros.h>
// #include <memory>
// #include <iostream>
// #include <thread>

// #include "ugv_sdk/mobile_robot/scout_robot.hpp"
// #include "ugv_sdk/utilities/protocol_detector.hpp"

// using namespace westonrobot;

// int main(int argc, char **argv) {
//   std::string device_name = "can0";
//   std::string robot_subtype = "mini";

//   if (argc == 2) {
//     device_name = {argv[1]};
//     std::cout << "Selected interface " << device_name << ", robot type: scout"
//               << std::endl;
//   } else if (argc == 3) {
//     robot_subtype = {argv[1]};
//     device_name = {argv[2]};
//     std::cout << "Selected interface " << device_name
//               << ", robot type: " << robot_subtype << std::endl;
//   } else {
//     std::cout << "Usage: demo_scout_robot [<robot-subtype>] <interface>"
//               << std::endl
//               << "Example 1: ./demo_scout_robot can0" << std::endl
//               << "\t <robot-subtype>: mini" << std::endl;
//     return -1;
//   }

//   bool is_scout_mini = false;
//   if (robot_subtype == "mini") {
//     is_scout_mini = true;
//   } else if (!robot_subtype.empty() && robot_subtype != "scout") {
//     std::cout
//         << "Unkonwn robot subtype. Supported subtypes: \"scout\" or \"mini\""
//         << std::endl;
//   }

//   std::unique_ptr<ScoutRobot> scout;

//   ProtocolDetector detector;
//   if (detector.Connect(device_name)) {
//     auto proto = detector.DetectProtocolVersion(5);
//     if (proto == ProtocolVersion::AGX_V1) {
//       std::cout << "Detected protocol: AGX_V1" << std::endl;
//       scout = std::unique_ptr<ScoutRobot>(
//           new ScoutRobot(ProtocolVersion::AGX_V1, is_scout_mini));
//     } else if (proto == ProtocolVersion::AGX_V2) {
//       std::cout << "Detected protocol: AGX_V2" << std::endl;
//       scout = std::unique_ptr<ScoutRobot>(
//           new ScoutRobot(ProtocolVersion::AGX_V2, is_scout_mini));
//     } else {
//       std::cout << "Detected protocol: UNKONWN" << std::endl;
//       return -1;
//     }
//   } else {
//     return -1;
//   }

//   if (scout == nullptr)
//     std::cout << "Failed to create robot object" << std::endl;

//   scout->Connect(device_name);

//   if (scout->GetParserProtocolVersion() == ProtocolVersion::AGX_V2) {
//     scout->EnableCommandedMode();
//   }

//   int count = 0;
//   while (count <= 2) {
//     // motion control
//     std::cout << "Motor: 1.0, 0" << std::endl;
//     scout->SetMotionCommand(1.0, 0.0);

//     // get robot state
//     auto state = scout->GetRobotState();
//     std::cout << "-------------------------------" << std::endl;
//     std::cout << "count: " << count << std::endl;
//     std::cout << "control mode: "
//               << static_cast<int>(state.system_state.control_mode)
//               << " , vehicle state: "
//               << static_cast<int>(state.system_state.vehicle_state)
//               << " , error code: " << std::hex << state.system_state.error_code
//               << std::dec
//               << ", battery voltage: " << state.system_state.battery_voltage
//               << std::endl;
//     std::cout << "velocity (linear, angular): "
//               << state.motion_state.linear_velocity << ", "
//               << state.motion_state.angular_velocity << std::endl;
//     std::cout << "core state age (ms): "
//               << std::chrono::duration_cast<std::chrono::milliseconds>(
//                      AgxMsgRefClock::now() - state.time_stamp)
//                      .count()
//               << std::endl;

//     auto actuator = scout->GetActuatorState();
//     if (scout->GetParserProtocolVersion() == ProtocolVersion::AGX_V1) {
//       for (int i = 0; i < 4; ++i) {
//         printf("motor %d: current %f, rpm %d, driver temp %f, motor temp %f\n",
//                actuator.actuator_state[i].motor_id,
//                actuator.actuator_state[i].current,
//                actuator.actuator_state[i].rpm,
//                actuator.actuator_state[i].driver_temp,
//                actuator.actuator_state[i].motor_temp);
//       }
//       std::cout << "actuator state age (ms): "
//                 << std::chrono::duration_cast<std::chrono::milliseconds>(
//                        AgxMsgRefClock::now() - actuator.time_stamp)
//                        .count()
//                 << std::endl;
//     } else {
//       for (int i = 0; i < 4; ++i) {
//         printf("motor %d: current %f, rpm %d, driver temp %f, motor temp %f\n",
//                actuator.actuator_hs_state[i].motor_id,
//                actuator.actuator_hs_state[i].current,
//                actuator.actuator_hs_state[i].rpm,
//                actuator.actuator_ls_state[i].driver_temp,
//                actuator.actuator_ls_state[i].motor_temp);
//       }
//       std::cout << "actuator state age (ms): "
//                 << std::chrono::duration_cast<std::chrono::milliseconds>(
//                        AgxMsgRefClock::now() - actuator.time_stamp)
//                        .count()
//                 << std::endl;
//     }
//     std::cout << "-------------------------------" << std::endl;

//     // usleep(20000);
//     std::this_thread::sleep_for(std::chrono::seconds(3));
//     ++count;
//   }

//   return 0;
// }
#include <ros/ros.h>
// #include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include "scout_msgs/ScoutLightCmd.h"
double pose_x;
double pose_y;
nav_msgs::Odometry odom_pose;

scout_msgs::ScoutLightCmd light_cmd;


void Odometry_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_publisher");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>(
        "/odom", 10, Odometry_Callback);

    ros::Publisher pub = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 10);

    light_cmd.enable_cmd_light_control = 1;

    light_cmd.front_mode = 0;

    ros::Rate rate(10); // 10 Hz

    while (ros::ok())
    {
        geometry_msgs::TwistStamped msg;
        pose_x = odom_pose.pose.pose.position.x;
        pose_y = odom_pose.pose.pose.position.y;
        // Set your velocities here

        if (pose_x <= 3)
        {
            msg.twist.linear.x = 0.5;
        }else
        {
            msg.twist.linear.x = 0.0;
        }
        //msg.twist.linear.x = 0.5;
        
          // example linear velocity
        //msg.angular.z = 0.5; // example angular velocity

        pub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
