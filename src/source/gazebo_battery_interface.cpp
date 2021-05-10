/*!*******************************************************************************
 *  \brief      This is the battery interface package for Gazebo.
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

#include "gazebo_battery_interface.h"
using namespace std;

BatteryInterface::BatteryInterface(){
}

BatteryInterface::~BatteryInterface(){
}

void BatteryInterface::ownSetUp() {
    ros::param::get("~drone_id", drone_id);
    ros::param::get("~mav_name", mav_name);  
}

void BatteryInterface::ownStart(){
    ros::NodeHandle n;

    //Standard
    battery_pub = n.advertise<sensor_msgs::BatteryState>("sensor_measurement/battery_state", 1, true);
    battery_msg.percentage = 1;    
    battery_pub.publish(battery_msg);
}

//Stop
void BatteryInterface::ownStop()
{
    battery_pub.shutdown();  
}

//Reset
bool BatteryInterface::resetValues()
{
    return true;
}

//Run
void BatteryInterface::ownRun()
{

}

int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, "BatteryInterface");

    cout<<"[ROSNODE] Starting BatteryInterface"<<endl;

    //Vars
    BatteryInterface battery_interface;
    battery_interface.setUp();
    battery_interface.start();
    try
    {
        //Read messages
        ros::spin();
        return 1;

    }
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }
}