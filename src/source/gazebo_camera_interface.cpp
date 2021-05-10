/*!*******************************************************************************
 *  \brief      This is the camera interface package for Gazebo.
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

#include "gazebo_camera_interface.h"
using namespace std;

CameraInterface::CameraInterface()
{
}

CameraInterface::~CameraInterface()
{
}

void CameraInterface::ownSetUp() {
    ros::param::get("~drone_id", drone_id);
    ros::param::get("~mav_name", mav_name);   
    ros::param::get("~frecuency", frecuency);       
}

void CameraInterface::ownStart()
{
    ros::NodeHandle n;
    //Publisher
    camera_pub = n.advertise<sensor_msgs::Image>("sensor_measurement/camera", 1, true);

    //Subscriber
    camera_sub = n.subscribe("/"+mav_name+cvg_int_to_string(drone_id)+"/camera_front/image_raw", 1, &CameraInterface::cameraCallback, this);
}

int CameraInterface::getRate(){
    return frecuency;
}

//Reset
bool CameraInterface::resetValues()
{
    return true;
}

//Run
void CameraInterface::ownRun()
{

}

//Stop
void CameraInterface::ownStop()
{
    camera_pub.shutdown();
    camera_sub.shutdown();
}

void CameraInterface::cameraCallback(const sensor_msgs::Image &msg)
{
    camera_pub.publish(msg);
}

int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, "CameraInterface");

    cout<<"[ROSNODE] Starting CameraInterface"<<endl;

    //Vars
    CameraInterface camera_interface;
    camera_interface.setUp();
    camera_interface.start();
    ros::Rate loop_rate(camera_interface.getRate());
 
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