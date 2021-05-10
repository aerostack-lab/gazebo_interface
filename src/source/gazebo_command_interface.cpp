/*!*******************************************************************************
 *  \brief      This is the command interface package for Gazebo.
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

#include "gazebo_command_interface.h"

using namespace std;

GazeboCommandInterface::GazeboCommandInterface()
{
}

GazeboCommandInterface::~GazeboCommandInterface()
{
}

void GazeboCommandInterface::ownSetUp()
{
    ros::param::get("~drone_id", drone_id);
    ros::param::get("~mav_name", mav_name);                  
}

void GazeboCommandInterface::ownStart()
{
    ros::NodeHandle n;    
    //Publishers
    motor_speed_pub = n.advertise<mav_msgs::Actuators>("/"+mav_name+std::to_string(drone_id)+"/command/motor_speed", 1, true);

    //Subscribers
    motor_speed_sub = n.subscribe("actuator_command/motor_speed", 1, &GazeboCommandInterface::MotorSpeedCallback, this, ros::TransportHints().tcpNoDelay());
}

//Reset
bool GazeboCommandInterface::resetValues()
{
    return true;
}

int GazeboCommandInterface::getRate(){
    return frecuency;
}

//Stop
void GazeboCommandInterface::ownStop()
{
    motor_speed_pub.shutdown();   
    motor_speed_sub.shutdown();
}

//Run
void GazeboCommandInterface::ownRun()
{

}

#define KEEP_IN_RANGE(a, min, max) if (a < min) a = min; else if (a > max) a = max;

void GazeboCommandInterface::MotorSpeedCallback(const mav_msgs::Actuators& msg)
{
    motor_speed_pub.publish(msg);
}

int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, "GazeboCommandInterface");

    cout<<"[ROSNODE] Starting GazeboCommandInterface"<<endl;

    //Vars
    GazeboCommandInterface gazebo_command_interface;
    gazebo_command_interface.setUp();
    gazebo_command_interface.start();
 
    try
    {    
        while(ros::ok())
        {  
            ros::spin();
             //ros::spinOnce();
             //loop_rate.sleep();
        }
    }
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }
}