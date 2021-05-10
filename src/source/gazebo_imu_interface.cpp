/*!*******************************************************************************
 *  \brief      This is the IMU interface package for Gazebo.
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

#include "gazebo_imu_interface.h"
using namespace std;

ImuInterface::ImuInterface()
{
}

ImuInterface::~ImuInterface()
{
}

void ImuInterface::ownSetUp() {
    ros::param::get("~drone_id", drone_id);
    ros::param::get("~mav_name", mav_name);   
    ros::param::get("~frecuency", frecuency);       
}

void ImuInterface::ownStart()
{
    ros::NodeHandle n;
    //Publisher
    imu_pub = n.advertise<sensor_msgs::Imu>("sensor_measurement/imu", 1, true);

    //Subscriber
    imu_sub = n.subscribe("/"+mav_name+cvg_int_to_string(drone_id)+"/imu", 1, &ImuInterface::imuCallback, this);
}

int ImuInterface::getRate(){
    return frecuency;
}

//Reset
bool ImuInterface::resetValues()
{
    return true;
}

//Run
void ImuInterface::ownRun()
{

}

//Stop
void ImuInterface::ownStop()
{
    imu_pub.shutdown();
    imu_sub.shutdown();
}

void ImuInterface::imuCallback(const sensor_msgs::Imu &msg)
{
    imu_pub.publish(msg);
}

int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, "ImuInterface");

    cout<<"[ROSNODE] Starting ImuInterface"<<endl;

    //Vars
    ImuInterface imu_interface;
    imu_interface.setUp();
    imu_interface.start();
    ros::Rate loop_rate(imu_interface.getRate());
 
    try
    {    
        while(ros::ok())
        {  
             ros::spinOnce();
             loop_rate.sleep();
        }
    }
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }
}