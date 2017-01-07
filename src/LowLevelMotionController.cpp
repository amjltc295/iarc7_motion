////////////////////////////////////////////////////////////////////////////
//
// LowLevelMotionController
//
// Class to implement control of the quads movement
//
////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>

#include "iarc7_motion/AccelerationPlanner.hpp"
#include "iarc7_motion/QuadVelocityController.hpp"
#include "iarc7_motion/QuadTwistRequestLimiter.hpp"

#include "iarc7_safety/SafetyClient.hpp"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"

using namespace Iarc7Motion;
using geometry_msgs::TwistStamped;
using geometry_msgs::Twist;

// This is a helper function that will limit a uav command using the twist limiter
// It will eventually no longer be needed when the quad velocity controller is converted to use twists
void limitUavCommand(QuadTwistRequestLimiter& limiter, iarc7_msgs::OrientationThrottleStamped& uav_command)
{
    TwistStamped uav_twist_stamped;
    Twist& uav_twist = uav_twist_stamped.twist;

    // Convert from uav command to twist
    uav_twist_stamped.header.stamp = uav_command.header.stamp;
    uav_twist.linear.z  = uav_command.throttle;
    uav_twist.angular.y = uav_command.data.pitch;
    uav_twist.angular.x = uav_command.data.roll;
    uav_twist.angular.z = uav_command.data.yaw;

    limiter.limitTwist(uav_twist_stamped);

    // Copy the twist to the uav command
    uav_command.header.stamp = uav_twist_stamped.header.stamp;
    uav_command.throttle     = uav_twist.linear.z;
    uav_command.data.pitch   = uav_twist.angular.y;
    uav_command.data.roll    = uav_twist.angular.x;
    uav_command.data.yaw     = uav_twist.angular.z;
}

void getPidParams(ros::NodeHandle& nh, double throttle_pid[5], double pitch_pid[5], double roll_pid[5], double yaw_pid[5])
{
    // Throttle PID settings retrieve
    nh.param("throttle_p", throttle_pid[0], 0.0);
    nh.param("throttle_i", throttle_pid[1], 0.0);
    nh.param("throttle_d", throttle_pid[2], 0.0);
    nh.param("throttle_accumulator_max", throttle_pid[3], 0.0);
    nh.param("throttle_accumulator_min", throttle_pid[4], 0.0);

    // Pitch PID settings retrieve
    nh.param("pitch_p", pitch_pid[0], 0.0);
    nh.param("pitch_i", pitch_pid[1], 0.0);
    nh.param("pitch_d", pitch_pid[2], 0.0);
    nh.param("pitch_accumulator_max", pitch_pid[3], 0.0);
    nh.param("pitch_accumulator_min", pitch_pid[4], 0.0);

    // Roll PID settings retrieve
    nh.param("roll_p", roll_pid[0], 0.0);
    nh.param("roll_i", roll_pid[1], 0.0);
    nh.param("roll_d", roll_pid[2], 0.0);
    nh.param("roll_accumulator_max", roll_pid[3], 0.0);
    nh.param("roll_accumulator_min", roll_pid[4], 0.0);

    // Yaw PID settings retrieve
    nh.param("yaw_p", yaw_pid[0], 0.0);
    nh.param("yaw_i", yaw_pid[1], 0.0);
    nh.param("yaw_d", yaw_pid[2], 0.0);
    nh.param("yaw_accumulator_max", yaw_pid[3], 0.0);
    nh.param("yaw_accumulator_min", yaw_pid[4], 0.0);
}

void getUavCommandParams(ros::NodeHandle& nh, Twist& min,  Twist& max,  Twist& max_rate)
{
    // Throttle Limit settings retrieve
    nh.param("throttle_max", max.linear.z, 0.0);
    nh.param("throttle_min", min.linear.z, 0.0);
    nh.param("throttle_max_rate", max_rate.linear.z, 0.0);

    // Pitch Limit settings retrieve
    nh.param("pitch_max", max.angular.y, 0.0);
    nh.param("pitch_min", min.angular.y, 0.0);
    nh.param("pitch_max_rate", max_rate.angular.y, 0.0);

    // Roll Limit settings retrieve
    nh.param("roll_max", max.angular.x, 0.0);
    nh.param("roll_min", min.angular.x, 0.0);
    nh.param("roll_max_rate", max_rate.angular.x, 0.0);

    // Yaw Limit settings retrieve
    nh.param("yaw_max", max.angular.z, 0.0);
    nh.param("yaw_min", min.angular.z, 0.0);
    nh.param("yaw_max_rate", max_rate.angular.z, 0.0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Low_Level_Motion_Control");

    ROS_INFO("Low_Level_Motion_Control begin");

    ros::NodeHandle nh;
    ros::NodeHandle param_nh ("low_level_motion_controller");

    ROS_INFO("low_level_motion: Attempting to form safety bond");
    Iarc7Safety::SafetyClient safety_client(nh, "low_level_motion");
    ROS_ASSERT_MSG(safety_client.formBond(), "low_level_motion: Could not form bond with safety client");

    double throttle_pid[5];
    double pitch_pid[5];
    double roll_pid[5];
    double yaw_pid[5];
    getPidParams(param_nh, throttle_pid, pitch_pid, roll_pid, yaw_pid);

    QuadVelocityController quadController(throttle_pid, pitch_pid, roll_pid, yaw_pid);

    AccelerationPlanner accelerationPlanner(nh);

    ros::Publisher uav_control_ = nh.advertise<iarc7_msgs::OrientationThrottleStamped>("uav_direction_command", 50);

    // Check for empty uav_control_ as per http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers
    // section 1
    ROS_ASSERT_MSG(uav_control_, "Could not create uav_direction_command publisher");

    Twist min;
    Twist max;
    Twist max_rate;
    getUavCommandParams(param_nh, min, max, max_rate);
    QuadTwistRequestLimiter limiter(min, max, max_rate);

    while (ros::ok() && ros::Time::now() == ros::Time(0)) {
        // wait
        ros::spinOnce();
    }

    while (ros::ok())
    {
        // Check the safety client before updating anything
        ROS_ASSERT_MSG(!safety_client.isFatalActive(), "low_level_motion: fatal event from safety");

        ros::Time current_time = ros::Time::now();

        geometry_msgs::TwistStamped target_twist;
        
        if(safety_client.isSafetyActive())
        {
            // Override whatever the Acceleration Planner wants to do and attempt to land
            target_twist.twist.linear.z = -0.2; // Try to descend
        }
        else
        {
            // If nothing is wrong get a target velocity from the acceleration planner
            accelerationPlanner.getTargetTwist(current_time, target_twist);
        }

        quadController.setTargetVelocity(target_twist.twist);

        iarc7_msgs::OrientationThrottleStamped uav_command;
        bool success = quadController.update(current_time, uav_command);
        ROS_ASSERT_MSG(success, "LowLevelMotion controller update failed");

        // Limit the uav command
        limitUavCommand(limiter, uav_command);

        // Publish the desired angles and throttle
        uav_control_.publish(uav_command);

        ros::spinOnce();
    }

    // All is good.
    return 0;
}