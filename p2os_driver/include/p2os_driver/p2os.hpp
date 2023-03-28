/*
 *  P2OS for ROS
 *  Copyright (C) 2009
 *     David Feil-Seifer, Brian Gerkey, Kasper Stoy,
 *      Richard Vaughan, & Andrew Howard
 *  Copyright (C) 2017
 *     Open Source Robotics Foundation, inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef _P2OSDEVICE_H
#define _P2OSDEVICE_H

#include <pthread.h>
#include <sys/time.h>
#include <iostream>
#include <cstring>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>
#include <p2os_driver/packet.hpp>
#include <p2os_driver/robot_params.hpp>

#include <p2os_msgs/msg/battery_state.hpp>
#include <p2os_msgs/msg/motor_state.hpp>
#include <p2os_msgs/msg/gripper_state.hpp>
#include <p2os_msgs/msg/sonar_array.hpp>
#include <p2os_msgs/msg/dio.hpp>
#include <p2os_msgs/msg/aio.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/twist.hpp>

/* tf2_ros */
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>


#include <p2os_parameters.hpp>


/* #include <diagnostic_updater/publisher.h> */
/* #include <diagnostic_updater/diagnostic_updater.h> */

/*! \brief Container struct.
 *
 *  Create a struct that holds the
 *  Robot's sensors.
 */
struct ros_p2os_data_t {
    //! Provides the position of the robot
    nav_msgs::msg::Odometry position;
    //! Provides the battery voltage
    p2os_msgs::msg::BatteryState batt;
    //! Provides the state of the motors (enabled or disabled)
    p2os_msgs::msg::MotorState motors;
    //! Provides the state of the gripper
    p2os_msgs::msg::GripperState gripper;
    //! Container for sonar data
    p2os_msgs::msg::SonarArray sonar;
    //! Digital In/Out
    p2os_msgs::msg::DIO dio;
    //! Analog In/Out
    p2os_msgs::msg::AIO aio;
    //! Transformed odometry frame.
    geometry_msgs::msg::TransformStamped odom_trans;
};

// this is here because we need the above typedef's before including it.
#include "sip.hpp"
#include "kinecalc.hpp"

#include "p2os_ptz.h"

class SIP;

// Forward declaration of the KineCalc_Base class declared in kinecalc_base.h
//class KineCalc;


class P2OSNode {
    /*! \brief P2OS robot driver node.
     *
     *  This class contains, essentially, the main means of communication
     *  between the robot and ROS.
     */
public:
    P2OSNode(const std::shared_ptr<rclcpp::Node>& _node);

    virtual ~P2OSNode();

    //! Setup the robot for use. Communicates with the robot directly.
    int Setup();

    //! Prepare for shutdown.
    int Shutdown();

    int SendReceive(P2OSPacket *pkt, bool publish_data = true);

    void updateDiagnostics();

    void ResetRawPositions();

    void ToggleSonarPower(unsigned char val);

    void ToggleMotorPower(unsigned char val);

    void StandardSIPPutData(rclcpp::Time &ts);

    inline double TicksToDegrees(int joint, unsigned char ticks);

    inline unsigned char DegreesToTicks(int joint, double degrees);

    inline double TicksToRadians(int joint, unsigned char ticks);

    inline unsigned char RadiansToTicks(int joint, double rads);

    inline double RadsPerSectoSecsPerTick(int joint, double speed);

    inline double SecsPerTicktoRadsPerSec(int joint, double secs);

    void SendPulse(void);

    //void spin();
    void check_and_set_vel();

    void cmdvel_cb(const std::shared_ptr<geometry_msgs::msg::Twist>);

    void check_and_set_motor_state();

    void cmdmotor_state(const std::shared_ptr<p2os_msgs::msg::MotorState>);

    void check_and_set_gripper_state();

    void gripperCallback(const std::shared_ptr<p2os_msgs::msg::GripperState>);

    const double &get_pulse() { return params_.pulse; }

    // diagnostic messages
    /* TODO(allenh1): re-enable diagnostic_updater */
    /* void check_voltage( diagnostic_updater::DiagnosticStatusWrapper &stat ); */
    /* void check_stall( diagnostic_updater::DiagnosticStatusWrapper &stat ); */

    //! Command Velocity subscriber
    geometry_msgs::msg::Twist cmdvel_;
    //! Motor state publisher
    p2os_msgs::msg::MotorState cmdmotor_state_;
    //! Gripper state publisher
    p2os_msgs::msg::GripperState gripper_state_;
    //! sensor data container
    ros_p2os_data_t p2os_data;

    //! Node Handler used for publication of data.
    std::shared_ptr<rclcpp::Node> node;

protected:
    std::shared_ptr<p2os_parameters::ParamListener> param_listener_;
    p2os_parameters::Params params_;

    /* TODO(allenh1): replace this with ROS2's version of dynamic_reconfigure */
    /* diagnostic_updater::Updater diagnostic_; */
    /* diagnostic_updater::DiagnosedPublisher<p2os_msgs::BatteryState> batt_pub_; */

    /* declare publishers */
    std::shared_ptr<rclcpp::Publisher<p2os_msgs::msg::MotorState>> mstate_pub_;
    std::shared_ptr<rclcpp::Publisher<p2os_msgs::msg::GripperState>> grip_state_pub_;
    std::shared_ptr<rclcpp::Publisher<p2os_msgs::msg::PTZState>> ptz_state_pub_;
    std::shared_ptr<rclcpp::Publisher<p2os_msgs::msg::SonarArray>> sonar_pub_;
    std::shared_ptr<rclcpp::Publisher<p2os_msgs::msg::AIO>> aio_pub_;
    std::shared_ptr<rclcpp::Publisher<p2os_msgs::msg::DIO>> dio_pub_;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> pose_pub_;
    std::shared_ptr<rclcpp::Publisher<p2os_msgs::msg::BatteryState>> batt_pub_;

    /* declare subscribers */
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmdvel_sub_;
    std::shared_ptr<rclcpp::Subscription<p2os_msgs::msg::MotorState>> cmdmstate_sub_;
    std::shared_ptr<rclcpp::Subscription<p2os_msgs::msg::GripperState>> gripper_sub_;
    std::shared_ptr<rclcpp::Subscription<p2os_msgs::msg::PTZState>> ptz_cmd_sub_;

    tf2_ros::Buffer *buffer = nullptr;
    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster = nullptr;
    rclcpp::Time veltime;

    SIP *sippacket;
//    std::string psos_serial_port;
//    std::string psos_tcp_host;
//    std::string odom_frame_id;
//    std::string base_link_frame_id;
    int psos_fd;
//    bool psos_use_tcp;
//    int psos_tcp_port;
    bool vel_dirty, motor_dirty;
    bool gripper_dirty_ = false;
    int param_idx;
    // PID settings
//    int rot_kp, rot_kv, rot_ki, trans_kp, trans_kv, trans_ki;

    //! Stall I hit a wall?
//    int bumpstall; // should we change the bumper-stall behavior?
    //! Use Joystick?
//    int joystick;
    //! Control wheel velocities individually?
//    int direct_wheel_vel_control;
//    int radio_modemp;

    //! Maximum motor speed in Meters per second.
    int motor_max_speed;
    //! Maximum turn speed in radians per second.
    int motor_max_turnspeed;
    //! Maximum translational acceleration in Meters per second per second.
    short motor_max_trans_accel;
    //! Minimum translational acceleration in Meters per second per second.
    short motor_max_trans_decel;
    //! Maximum rotational acceleration in radians per second per second.
    short motor_max_rot_accel;
    //! Minimum rotational acceleration in Meters per second per second.
    short motor_max_rot_decel;
    //! Pulse time
//    double pulse;
    double desired_freq;
    //! Last time the node received or sent a pulse.
    double lastPulseTime;
    //! Use the sonar array?
//    bool use_sonar_;

    P2OSPtz ptz_;
};

#endif
