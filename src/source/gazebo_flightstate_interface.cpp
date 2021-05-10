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

#include "gazebo_flightstate_interface.h"

using namespace std;

GazeboFlightStateInterface::GazeboFlightStateInterface()
{
}

GazeboFlightStateInterface::~GazeboFlightStateInterface()
{
}

void GazeboFlightStateInterface::ownSetUp()
{
    ros::param::get("~drone_id", drone_id);
    ros::param::get("~mav_name", mav_name);
    ros::param::get("~frecuency", frecuency);
}

void GazeboFlightStateInterface::ownStart()
{
    ros::NodeHandle n;
    //Publishers
    flightstate_pub = n.advertise<aerostack_msgs::FlightState>("self_localization/flight_state", 1, true);

    //Subscribers
    odometry_sub = n.subscribe("/"+mav_name+cvg_int_to_string(drone_id)+"/ground_truth/odometry",1, &GazeboFlightStateInterface::odometryCallback, this);
    flightaction_sub = n.subscribe("actuator_command/flight_action",1, &GazeboFlightStateInterface::flightActionCallback, this);
    flightstate_sub = n.subscribe("self_localization/flight_state", 1, &GazeboFlightStateInterface::statusCallBack, this);

    flight_action_msg.action = aerostack_msgs::FlightActionCommand::UNKNOWN;
    flight_state_msg.state = aerostack_msgs::FlightState::UNKNOWN;
}

//Reset
bool GazeboFlightStateInterface::resetValues()
{
    return true;
}

//Stop
void GazeboFlightStateInterface::ownStop()
{
    flightstate_pub.shutdown();
    odometry_sub.shutdown();
    flightaction_sub.shutdown();
}

//Run
void GazeboFlightStateInterface::ownRun()
{   
    switch(flight_action_msg.action){
        case aerostack_msgs::FlightActionCommand::TAKE_OFF:
            if (flight_state_msg.state == aerostack_msgs::FlightState::LANDED || flight_state_msg.state == aerostack_msgs::FlightState::UNKNOWN){
                flight_state_msg.state = aerostack_msgs::FlightState::TAKING_OFF;
            }
            else{
                if (flight_state_msg.state == aerostack_msgs::FlightState::TAKING_OFF){
                    if (odom_msg.pose.pose.position.z > 0.1){
                        flight_state_msg.state = aerostack_msgs::FlightState::FLYING;
                    }
                }
            }
        break;
        case aerostack_msgs::FlightActionCommand::HOVER:{
            if(odom_msg.pose.pose.position.z > 0.1 && std::abs(odom_msg.twist.twist.linear.x) < 0.05 && std::abs(odom_msg.twist.twist.linear.y) < 0.05 && std::abs(odom_msg.twist.twist.linear.z) < 0.05 &&
            std::abs(odom_msg.twist.twist.angular.x) < 0.05 && std::abs(odom_msg.twist.twist.angular.y) < 0.05 && std::abs(odom_msg.twist.twist.angular.z) < 0.05){
                flight_state_msg.state = aerostack_msgs::FlightState::HOVERING;
            }
        }
        break;
        case aerostack_msgs::FlightActionCommand::LAND:{
            if (flight_state_msg.state != aerostack_msgs::FlightState::LANDED && flight_state_msg.state != aerostack_msgs::FlightState::LANDING){
                flight_state_msg.state = aerostack_msgs::FlightState::LANDING;
                landing_command_time_ = ros::Time::now();
            }
            else{
                if (flight_state_msg.state == aerostack_msgs::FlightState::LANDING){
                    if (fabs(odom_msg.twist.twist.linear.z) > 0.05){
                        landing_command_time_ = ros::Time::now();
                    }
                    else if (((ros::Time::now()-landing_command_time_).toSec() > LANDING_CHECK_DELAY)){
                        flight_state_msg.state = aerostack_msgs::FlightState::LANDED;
                    }
                }
            }
        }
        break;
        case aerostack_msgs::FlightActionCommand::MOVE:{
            if(std::abs(odom_msg.pose.pose.position.z) > 0.1 && (std::abs(odom_msg.twist.twist.linear.x) > 0.05 || std::abs(odom_msg.twist.twist.linear.y) > 0.05 || std::abs(odom_msg.twist.twist.linear.z) > 0.05 ||
            std::abs(odom_msg.twist.twist.angular.x) > 0.05 || std::abs(odom_msg.twist.twist.angular.y) > 0.05 || std::abs(odom_msg.twist.twist.angular.z) > 0.05)){
                flight_state_msg.state = aerostack_msgs::FlightState::FLYING;
            }
            if(std::abs(odom_msg.pose.pose.position.z) > 0.1 && std::abs(odom_msg.twist.twist.linear.x) < 0.05 && std::abs(odom_msg.twist.twist.linear.y) < 0.05 && std::abs(odom_msg.twist.twist.linear.z) < 0.05 &&
            std::abs(odom_msg.twist.twist.angular.x) < 0.05 && std::abs(odom_msg.twist.twist.angular.y) < 0.05 && std::abs(odom_msg.twist.twist.angular.z) < 0.05){
                flight_state_msg.state = aerostack_msgs::FlightState::HOVERING;
            }
        }
        break;
        case aerostack_msgs::FlightActionCommand::UNKNOWN:
        default:
        {
            if(odom_msg.pose.pose.position.z < 0.1 && std::abs(odom_msg.twist.twist.linear.x) < 0.05 && std::abs(odom_msg.twist.twist.linear.y) < 0.05 && std::abs(odom_msg.twist.twist.linear.z) < 0.05 &&
            std::abs(odom_msg.twist.twist.angular.x) < 0.05 && std::abs(odom_msg.twist.twist.angular.y) < 0.05 && std::abs(odom_msg.twist.twist.angular.z) < 0.05){
                flight_state_msg.state = aerostack_msgs::FlightState::LANDED;
            }
            if(odom_msg.pose.pose.position.z > 0.1 && (std::abs(odom_msg.twist.twist.linear.x) > 0.05 || std::abs(odom_msg.twist.twist.linear.y) > 0.05 || std::abs(odom_msg.twist.twist.linear.z) > 0.05 ||
            std::abs(odom_msg.twist.twist.angular.x) > 0.05 || std::abs(odom_msg.twist.twist.angular.y) > 0.05 || std::abs(odom_msg.twist.twist.angular.z) > 0.05)){
                flight_state_msg.state = aerostack_msgs::FlightState::FLYING;
            }
            if(odom_msg.pose.pose.position.z > 0.1 && std::abs(odom_msg.twist.twist.linear.x) < 0.05 && std::abs(odom_msg.twist.twist.linear.y) < 0.05 && std::abs(odom_msg.twist.twist.linear.z) < 0.05 &&
            std::abs(odom_msg.twist.twist.angular.x) < 0.05 && std::abs(odom_msg.twist.twist.angular.y) < 0.05 && std::abs(odom_msg.twist.twist.angular.z) < 0.05){
                flight_state_msg.state = aerostack_msgs::FlightState::HOVERING;
            }
        }
        break;
    }
    flightstate_pub.publish(flight_state_msg);
}

void GazeboFlightStateInterface::odometryCallback(const nav_msgs::Odometry::ConstPtr &msg){
    odom_msg = *msg;
}

int GazeboFlightStateInterface::getRate(){
    return frecuency;
}

void GazeboFlightStateInterface::flightActionCallback(const aerostack_msgs::FlightActionCommand &msg){
    flight_action_msg = msg;
}

void GazeboFlightStateInterface::statusCallBack(const aerostack_msgs::FlightState &msg){
  flight_state_msg.state = msg.state;
}

int main(int argc,char **argv){
    //Ros Init
    ros::init(argc, argv, "GazeboFlightStateInterface");

    cout<<"[ROSNODE] Starting GazeboFlightStateInterface"<<endl;

    //Vars
    GazeboFlightStateInterface gazebo_flightstate_interface;
    gazebo_flightstate_interface.setUp();
    gazebo_flightstate_interface.start();
 
    ros::Rate loop_rate(gazebo_flightstate_interface.getRate());

    try
    {    
        while(ros::ok())
        {  
            gazebo_flightstate_interface.run();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }
}