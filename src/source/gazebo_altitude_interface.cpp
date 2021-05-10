/*!*******************************************************************************
 *  \brief      This is the altitude interface package for Gazebo.
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

#include "gazebo_altitude_interface.h"
using namespace std;

AltitudeInterface::AltitudeInterface()
{
}

AltitudeInterface::~AltitudeInterface()
{
}

void AltitudeInterface::ownSetUp()
{
    ros::param::get("~drone_id", drone_id);
    ros::param::get("~mav_name", mav_name);
    ros::param::get("~frecuency", frecuency);
}

int AltitudeInterface::getRate(){
    return frecuency;
}

void AltitudeInterface::ownStart()
{
    ros::NodeHandle n;
    //Publishers
    altitude_pub  = n.advertise<geometry_msgs::PointStamped>("sensor_measurement/altitude", 1, true);
    //Subscribers
    odometry_sub   = n.subscribe("/"+mav_name+cvg_int_to_string(drone_id)+"/ground_truth/odometry", 1, &AltitudeInterface::odometryCallback, this);
}

//Reset
bool AltitudeInterface::resetValues()
{
    return true;
}

//Stop
void AltitudeInterface::ownStop()
{
  altitude_pub.shutdown();
  odometry_sub.shutdown();
}

//Run
void AltitudeInterface::ownRun()
{
}

void AltitudeInterface::odometryCallback(const nav_msgs::Odometry &msg)
{
    ros::Time current_timestamp = ros::Time::now();

    // double zraw_t = (-1.0) * msg.pose.pose.position.z;

    // time_t tv_sec; suseconds_t tv_usec;
    // {
    // tv_sec  = current_timestamp.sec;
    // tv_usec = current_timestamp.nsec / 1000.0;
    // filtered_derivative_wcb.setInput( zraw_t, tv_sec, tv_usec);
    // }

    // double z_t, dz_t;
    // filtered_derivative_wcb.getOutput(z_t, dz_t);


    //Read Altitude from Pose
    altitude_msg.header = msg.header;
    altitude_msg.header.stamp  = current_timestamp;
    altitude_msg.point.z = (-1.0) * msg.pose.pose.position.z;
    altitude_msg.point.x = 0;
    altitude_msg.point.y = 0;
    altitude_pub.publish(altitude_msg);
}

int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, "AltitudeInterface");

    cout<<"[ROSNODE] Starting AltitudeInterface"<<endl;

    //Vars
    AltitudeInterface pose_interface;
    pose_interface.setUp();
    pose_interface.start();

    ros::Rate loop_rate(pose_interface.getRate());
 
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