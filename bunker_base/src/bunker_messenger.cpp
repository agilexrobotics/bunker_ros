/*
 * bunker_messenger.cpp
 *
 * Created on: Apr 26, 2019 22:14
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "bunker_base/bunker_messenger.hpp"

#include <tf/transform_broadcaster.h>

#include "bunker_msgs/BunkerStatus.h"
#include "bunker_msgs/BunkerBmsStatus.h"
#include "bunker_msgs/BunkerRsStatus.h"

namespace westonrobot
{
  BunkerROSMessenger::BunkerROSMessenger(ros::NodeHandle *nh)
      : bunker_(nullptr), nh_(nh) {}

  BunkerROSMessenger::BunkerROSMessenger(BunkerRobot *bunker, ros::NodeHandle *nh)
      : bunker_(bunker), nh_(nh) {}

  void BunkerROSMessenger::SetupSubscription()
  {
    // odometry publisher
    odom_publisher_ = nh_->advertise<nav_msgs::Odometry>(odom_topic_name_, 50);
    status_publisher_ = nh_->advertise<bunker_msgs::BunkerStatus>("/bunker_status", 10);
    rs_status_publisher_ = nh_->advertise<bunker_msgs::BunkerRsStatus>("/rs_status",10);
    BMS_status_publisher_ = nh_->advertise<bunker_msgs::BunkerBmsStatus>("/bms_status_msg", 10);

    // cmd subscriber
    motion_cmd_subscriber_ = nh_->subscribe<geometry_msgs::Twist>(
        "/cmd_vel", 5, &BunkerROSMessenger::TwistCmdCallback, this);

  }

  void BunkerROSMessenger::TwistCmdCallback(
      const geometry_msgs::Twist::ConstPtr &msg)
  {
    if (!simulated_robot_)
    {
      bunker_->SetMotionCommand(msg->linear.x, msg->angular.z);
    }
    else
    {
      std::lock_guard<std::mutex> guard(twist_mutex_);
      current_twist_ = *msg.get();
    }
    // ROS_INFO("cmd received:%f, %f", msg->linear.x, msg->angular.z);
  }

  void BunkerROSMessenger::GetCurrentMotionCmdForSim(double &linear,
                                                    double &angular)
  {
    std::lock_guard<std::mutex> guard(twist_mutex_);
    linear = current_twist_.linear.x;
    angular = current_twist_.angular.z;
  }



  void BunkerROSMessenger::PublishStateToROS()
  {
    current_time_ = ros::Time::now();
    double dt = (current_time_ - last_time_).toSec();

    static bool init_run = true;
    if (init_run)
    {
      last_time_ = current_time_;
      init_run = false;
      return;
    }
   
    //auto state = bunker_->GetBunkerState();
    auto robot_state = bunker_->GetRobotState();
    auto actuator_state = bunker_->GetActuatorState();
    auto bms_state = BunkerCommonSensorState();

    // publish bunker state message
    bunker_msgs::BunkerStatus status_msg;
    bunker_msgs::BunkerRsStatus rs_status_msg;
    bunker_msgs::BunkerBmsStatus bms_status_msg;

    status_msg.header.stamp = current_time_;

    status_msg.linear_velocity = robot_state.motion_state.linear_velocity;
    status_msg.angular_velocity = robot_state.motion_state.angular_velocity;
    status_msg.base_state = robot_state.system_state.vehicle_state;
    status_msg.control_mode = robot_state.system_state.control_mode;
    status_msg.fault_code = robot_state.system_state.error_code;
    status_msg.battery_voltage = robot_state.system_state.battery_voltage;

    rs_status_msg.header.stamp = current_time_;
    rs_status_msg.stick_left_h = robot_state.rc_state.stick_left_h;
    rs_status_msg.stick_left_v = robot_state.rc_state.stick_left_v;
    rs_status_msg.stick_right_h = robot_state.rc_state.stick_right_h;
    rs_status_msg.stick_right_v = robot_state.rc_state.stick_right_v;

    rs_status_msg.swa = robot_state.rc_state.swa;
    rs_status_msg.swb = robot_state.rc_state.swb;
    rs_status_msg.swc = robot_state.rc_state.swc;
    rs_status_msg.swd = robot_state.rc_state.swd;

    rs_status_msg.var_a = robot_state.rc_state.var_a;

    bms_status_msg.SOC = bms_state.bms_basic_state.battery_soc;
    bms_status_msg.SOH = bms_state.bms_basic_state.battery_soh;
    bms_status_msg.battery_voltage = bms_state.bms_basic_state.voltage;
    bms_status_msg.battery_current = bms_state.bms_basic_state.current;
    bms_status_msg.battery_temperature = bms_state.bms_basic_state.temperature;
    bms_status_msg.Alarm_Status_1 = bms_state.bms_extended_state.alarm_status_1;
    bms_status_msg.Alarm_Status_2 = bms_state.bms_extended_state.alarm_status_2;
    bms_status_msg.Warning_Status_1 = bms_state.bms_extended_state.warn_status_1;
    bms_status_msg.Warning_Status_2 = bms_state.bms_extended_state.warn_status_2;


    if(bunker_->GetParserProtocolVersion() == ProtocolVersion::AGX_V1)
    {
        for (int i = 0; i < 2; ++i)
        {
            status_msg.motor_states[i].current = actuator_state.actuator_state[i].current;
            status_msg.motor_states[i].rpm = actuator_state.actuator_state[i].rpm;
            status_msg.motor_states[i].temperature = actuator_state.actuator_state[i].motor_temp;
        }
    }
    else
    {
        for (int i = 0; i < 2; ++i)
        {
            status_msg.motor_states[i].current = actuator_state.actuator_hs_state[i].current;
            status_msg.motor_states[i].rpm = actuator_state.actuator_hs_state[i].rpm;
            status_msg.motor_states[i].temperature = actuator_state.actuator_ls_state[i].motor_temp;
        }
    }

    BMS_status_publisher_.publish(bms_status_msg);
    status_publisher_.publish(status_msg);
    rs_status_publisher_.publish(rs_status_msg);


    // publish odometry and tf
    PublishOdometryToROS(robot_state.motion_state.linear_velocity, robot_state.motion_state.angular_velocity, dt);

    // record time for next integration
    last_time_ = current_time_;
  }

  void BunkerROSMessenger::PublishSimStateToROS(double linear, double angular)
  {
    current_time_ = ros::Time::now();

    double dt = (current_time_ - last_time_).toSec();

    static bool init_run = true;
    if (init_run)
    {
      last_time_ = current_time_;
      init_run = false;
      return;
    }

    // publish bunker state message
    bunker_msgs::BunkerStatus status_msg;
    status_msg.header.stamp = current_time_;
    status_msg.linear_velocity = linear;
    status_msg.angular_velocity = angular;
    status_msg.base_state = 0x00;
    status_msg.control_mode = 0x01;
    status_msg.fault_code = 0x00;
    status_msg.battery_voltage = 29.5;
    status_publisher_.publish(status_msg);

    // publish odometry and tf
    PublishOdometryToROS(linear, angular, dt);

    // record time for next integration
    last_time_ = current_time_;
  }

  void BunkerROSMessenger::PublishOdometryToROS(double linear, double angular,
                                               double dt)
  {
    // perform numerical integration to get an estimation of pose
    linear_speed_ = linear;
    angular_speed_ = angular;

    double d_x = linear_speed_ * std::cos(theta_) * dt;
    double d_y = linear_speed_ * std::sin(theta_) * dt;
    double d_theta = angular_speed_ * dt;

    position_x_ += d_x;
    position_y_ += d_y;
    theta_ += d_theta;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

    // publish tf transformation
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

    if(pub_tf_)tf_broadcaster_.sendTransform(tf_msg);

    // publish odometry and tf messages
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.twist.twist.linear.x = linear_speed_;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_speed_;

    odom_publisher_.publish(odom_msg);
  }
} // namespace westonrobot
