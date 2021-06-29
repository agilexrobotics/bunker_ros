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

namespace westonrobot
{
  BunkerROSMessenger::BunkerROSMessenger(ros::NodeHandle *nh)
      : bunker_(nullptr), nh_(nh) {}

  BunkerROSMessenger::BunkerROSMessenger(BunkerBase *bunker, ros::NodeHandle *nh)
      : bunker_(bunker), nh_(nh) {}

  void BunkerROSMessenger::SetupSubscription()
  {
    // odometry publisher
    odom_publisher_ = nh_->advertise<nav_msgs::Odometry>(odom_topic_name_, 50);
    status_publisher_ =
        nh_->advertise<bunker_msgs::BunkerStatus>("/bunker_status", 10);

    // cmd subscriber
    motion_cmd_subscriber_ = nh_->subscribe<geometry_msgs::Twist>(
        "/cmd_vel", 5, &BunkerROSMessenger::TwistCmdCallback, this);
    light_cmd_subscriber_ = nh_->subscribe<bunker_msgs::BunkerLightCmd>(
        "/bunker_light_control", 5, &BunkerROSMessenger::LightCmdCallback, this);
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

  void BunkerROSMessenger::LightCmdCallback(
      const bunker_msgs::BunkerLightCmd::ConstPtr &msg)
  {
    if (!simulated_robot_)
    {
      if (msg->enable_cmd_light_control)
      {
        BunkerLightCmd cmd;

        switch (msg->front_mode)
        {
        case bunker_msgs::BunkerLightCmd::LIGHT_CONST_OFF:
        {
          cmd.front_mode = BunkerLightCmd::LightMode::CONST_OFF;
          break;
        }
        case bunker_msgs::BunkerLightCmd::LIGHT_CONST_ON:
        {
          cmd.front_mode = BunkerLightCmd::LightMode::CONST_ON;
          break;
        }
        case bunker_msgs::BunkerLightCmd::LIGHT_BREATH:
        {
          cmd.front_mode = BunkerLightCmd::LightMode::BREATH;
          break;
        }
        case bunker_msgs::BunkerLightCmd::LIGHT_CUSTOM:
        {
          cmd.front_mode = BunkerLightCmd::LightMode::CUSTOM;
          cmd.front_custom_value = msg->front_custom_value;
          break;
        }
        }

        switch (msg->rear_mode)
        {
        case bunker_msgs::BunkerLightCmd::LIGHT_CONST_OFF:
        {
          cmd.rear_mode = BunkerLightCmd::LightMode::CONST_OFF;
          break;
        }
        case bunker_msgs::BunkerLightCmd::LIGHT_CONST_ON:
        {
          cmd.rear_mode = BunkerLightCmd::LightMode::CONST_ON;
          break;
        }
        case bunker_msgs::BunkerLightCmd::LIGHT_BREATH:
        {
          cmd.rear_mode = BunkerLightCmd::LightMode::BREATH;
          break;
        }
        case bunker_msgs::BunkerLightCmd::LIGHT_CUSTOM:
        {
          cmd.rear_mode = BunkerLightCmd::LightMode::CUSTOM;
          cmd.rear_custom_value = msg->rear_custom_value;
          break;
        }
        }

        bunker_->SetLightCommand(cmd);
      }
      else
      {
        bunker_->DisableLightCmdControl();
      }
    }
    else
    {
      std::cout << "simulated robot received light control cmd" << std::endl;
    }
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

    auto state = bunker_->GetBunkerState();

    // publish bunker state message
    bunker_msgs::BunkerStatus status_msg;

    status_msg.header.stamp = current_time_;

    status_msg.linear_velocity = state.linear_velocity;
    status_msg.angular_velocity = state.angular_velocity;

    status_msg.base_state = state.base_state;
    status_msg.control_mode = state.control_mode;
    status_msg.fault_code = state.fault_code;
    status_msg.battery_voltage = state.battery_voltage;

    for (int i = 0; i < 2; ++i)
    {
      status_msg.motor_states[i].current = state.actuator_states[i].motor_current;
      status_msg.motor_states[i].rpm = state.actuator_states[i].motor_rpm;
      status_msg.motor_states[i].temperature = state.actuator_states[i].driver_temperature;
    }

    status_msg.light_control_enabled = state.light_control_enabled;
    status_msg.front_light_state.mode = state.front_light_state.mode;
    status_msg.front_light_state.custom_value =
        state.front_light_state.custom_value;
    status_msg.rear_light_state.mode = state.rear_light_state.mode;
    status_msg.rear_light_state.custom_value =
        state.front_light_state.custom_value;
    status_publisher_.publish(status_msg);

    // publish odometry and tf
    PublishOdometryToROS(state.linear_velocity, state.angular_velocity, dt);

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

    // for (int i = 0; i < 4; ++i)
    // {
    //     status_msg.motor_states[i].current = state.motor_states[i].current;
    //     status_msg.motor_states[i].rpm = state.motor_states[i].rpm;
    //     status_msg.motor_states[i].temperature =
    //     state.motor_states[i].temperature;
    // }

    status_msg.light_control_enabled = false;
    // status_msg.front_light_state.mode = state.front_light_state.mode;
    // status_msg.front_light_state.custom_value =
    // state.front_light_state.custom_value; status_msg.rear_light_state.mode =
    // state.rear_light_state.mode; status_msg.rear_light_state.custom_value =
    // state.front_light_state.custom_value;

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

   // tf_broadcaster_.sendTransform(tf_msg);

    // publish odometry and tf messages
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = "testes";
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
