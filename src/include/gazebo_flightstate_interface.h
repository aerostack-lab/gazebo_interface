/*!*******************************************************************************
 *  \brief      This is the flightstate interface package for Gazebo.
 *  \authors    Alberto Rodelgo
 *  \copyright  Copyright (c) 2020 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include <thread>
#include <iostream>
#include <math.h>
#include <cmath>

//// ROS  ///////
#include "ros/ros.h"

#include <robot_process.h>

//Navmsg
#include "nav_msgs/Odometry.h"
#include <pluginlib/class_list_macros.h>
#include "cvg_string_conversions.h"
#include "aerostack_msgs/FlightState.h"
#include "aerostack_msgs/FlightActionCommand.h"
#define LANDING_TIMEOUT 60
#define LANDING_CHECK_DELAY 1

class GazeboFlightStateInterface : public RobotProcess
{
    //Constructors and destructors
public:
    GazeboFlightStateInterface();
    ~GazeboFlightStateInterface();
    int getRate();
protected:
    bool resetValues();    
private: /*RobotProcess*/
    void ownSetUp();
    void ownStart();
    void ownStop();
    void ownRun();
    //Subscribers
    ros::Subscriber odometry_sub;
    ros::Subscriber flightaction_sub;
    void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void flightActionCallback(const aerostack_msgs::FlightActionCommand &msg);

    // Publishers
    ros::Publisher flightstate_pub;
    ros::Subscriber flightstate_sub;

    aerostack_msgs::FlightState flight_state_msg;
    aerostack_msgs::FlightActionCommand flight_action_msg;
    nav_msgs::Odometry odom_msg;
    ros::Time landing_command_time_;

public:
    void statusCallBack(const aerostack_msgs::FlightState &msg);
    std::string mav_name;
    int drone_id;
    int frecuency;
};
