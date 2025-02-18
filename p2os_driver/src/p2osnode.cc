/*
 *  P2OS for ROS
 *  Copyright (C) 2009
 *     Hunter Allen, David Feil-Seifer, Brian Gerkey, Kasper Stoy,
 *     Richard Vaughan, & Andrew Howard
 *
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

#include <iostream>

/* #include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h> */
#include <rclcpp/rclcpp.hpp>
#include <p2os_driver/p2os.hpp>
/* #include <p2os_msgs/MotorState.h>
   #include <tf/transform_datatypes.h> */

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto n = std::make_shared<rclcpp::Node>("p2os");

    P2OSNode *p = new P2OSNode(n);
    
    if(p->Setup())
    {
        RCLCPP_ERROR(n->get_logger(),
                     "p2os setup failed...");
        return -1;
    }
    
    p->ResetRawPositions();
    
    rclcpp::Time lastTime;
    
    while(rclcpp::ok())
    {
        p->check_and_set_vel();
        p->check_and_set_motor_state();
        p->check_and_set_gripper_state();
        
        if(p->get_pulse() > 0)
        {
            rclcpp::Time currentTime = n->now();
            rclcpp::Duration pulseInterval = currentTime - lastTime;
            if(pulseInterval.seconds() > p->get_pulse())
            { 
                RCLCPP_DEBUG(n->get_logger(),
                             "sending pulse" );
                p->SendPulse();
                lastTime = currentTime;
            }
        }
        
        // Hack fix to get around the fact that if no commands are sent to the
        // robot via SendReceive, the driver will never read SIP packets and so
        // never send data back to clients. We need a better way of doing regular
        // checks of the serial port - peek in sendreceive, maybe? Because if there
        // is no data waiting this will sit around waiting until one comes
        p->SendReceive(NULL,true);
        p->updateDiagnostics();
        rclcpp::spin_some(n);
    }
    
    if(!p->Shutdown())
    {
        RCLCPP_WARN(n->get_logger(), "p2os shutdown failed... your robot might be heading for the wall?");
    }
    delete p; //delete pointer
    
    RCLCPP_INFO(n->get_logger(), "Quitting... " );
    return 0;
}
