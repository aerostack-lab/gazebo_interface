/*!*******************************************************************************
 *  \brief      This is the speed interface package for Gazebo.
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

#include "gazebo_speed_interface.h"
using namespace std;

SpeedInterface::SpeedInterface()
{
}

SpeedInterface::~SpeedInterface()
{
}

void SpeedInterface::ownSetUp()
{
    ros::param::get("~drone_id", drone_id);
    ros::param::get("~mav_name", mav_name);
    ros::param::get("~frecuency", frecuency);
}

void SpeedInterface::ownStart()
{
   ros::NodeHandle n;
    //Publishers
    sensor_measurement_speed_publisher = n.advertise<geometry_msgs::TwistStamped>("sensor_measurement/linear_speed",1, true);

   // Subscribers
   odometry_sub = n.subscribe("/"+mav_name+cvg_int_to_string(drone_id)+"/ground_truth/odometry",1, &SpeedInterface::odometryCallback, this);

}

//Reset
bool SpeedInterface::resetValues()
{
    return true;
}

//Stop
void SpeedInterface::ownStop()
{
  odometry_sub.shutdown();
  sensor_measurement_speed_publisher.shutdown();
}

int SpeedInterface::getRate(){
    return frecuency;
}

//Run
void SpeedInterface::ownRun()
{
}

void SpeedInterface::odometryCallback(const nav_msgs::Odometry &msg)
{
      /* Calculating Roll, Pitch, Yaw */
      tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
      tf::Matrix3x3 m(q);

      //convert quaternion to euler angels
      double y, p, r;
      m.getEulerYPR(y, p, r);

      // BodyFrame to WorldFrame
      Eigen::Vector3f BodyFrame;
      Eigen::Vector3f GlobalFrame;
      Eigen::Matrix3f RotationMat;

      BodyFrame(0,0) = (+1)*msg.twist.twist.linear.x;
      BodyFrame(1,0) = (+1)*msg.twist.twist.linear.y;
      BodyFrame(2,0) = 0;

      RotationMat(0,0) = cos(y);
      RotationMat(1,0) = -sin(y);
      RotationMat(2,0) = 0;

      RotationMat(0,1) = sin(y);
      RotationMat(1,1) = cos(y);
      RotationMat(2,1) = 0;

      RotationMat(0,2) = 0;
      RotationMat(1,2) = 0;
      RotationMat(2,2) = 1;

      GlobalFrame = RotationMat.transpose()*BodyFrame;

      //Speed
       speed_stamped_msg.header.stamp = ros::Time::now();
       
       speed_stamped_msg.twist.linear.x = GlobalFrame(0);
       speed_stamped_msg.twist.linear.y = GlobalFrame(1);
       speed_stamped_msg.twist.linear.z = msg.twist.twist.linear.z;
       speed_stamped_msg.twist.angular = msg.twist.twist.angular;  
       sensor_measurement_speed_publisher.publish(speed_stamped_msg);
}

int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, "SpeedInterface");

    cout<<"[ROSNODE] Starting SpeedInterface"<<endl;

    //Vars
    SpeedInterface pose_interface;
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