/*!*******************************************************************************
 *  \brief      This is the ground truth interface package for Gazebo.
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

#include "gazebo_ground_truth_interface.h"
using namespace std;

GroundTruthInterface::GroundTruthInterface()
{
}

GroundTruthInterface::~GroundTruthInterface()
{
}

void GroundTruthInterface::ownSetUp()
{
    ros::param::get("~drone_id", drone_id);
    ros::param::get("~mav_name", mav_name);
    ros::param::get("~frecuency", frecuency);
    ros::param::get("~estimated_pose", estimated_pose); 
    ros::param::get("~estimated_speed", estimated_speed); 
}

void GroundTruthInterface::ownStart()
{
   ros::NodeHandle n;
    //Publishers
   self_localization_pose_publisher = n.advertise<geometry_msgs::PoseStamped>(estimated_pose,1, true);
   self_localization_speed_publisher = n.advertise<geometry_msgs::TwistStamped>(estimated_speed,1, true);

   /* Ground truth subscribers */
   odometry_sub = n.subscribe("/"+mav_name+cvg_int_to_string(drone_id)+"/ground_truth/odometry",1, &GroundTruthInterface::odometryCallback, this,ros::TransportHints().tcpNoDelay());
}

//Reset
bool GroundTruthInterface::resetValues()
{
    return true;
}

//Stop
void GroundTruthInterface::ownStop()
{
  odometry_sub.shutdown();
  self_localization_pose_publisher.shutdown();
  self_localization_speed_publisher.shutdown();
}

//Run
void GroundTruthInterface::ownRun()
{
}

void GroundTruthInterface::odometryCallback(const nav_msgs::Odometry &msg)
{
      /* Calculating Roll, Pitch, Yaw */
      tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
      tf::Matrix3x3 m(q);

      //convert quaternion to euler angels
      double y, p, r;
      m.getEulerYPR(y, p, r);

      // BodyFrame to GlobalFrame
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

      //Self_localization/pose
      pose_stamped_msg.header = msg.header;   
      pose_stamped_msg.pose.orientation = msg.pose.pose.orientation;
      pose_stamped_msg.pose.position = msg.pose.pose.position;
      self_localization_pose_publisher.publish(pose_stamped_msg);

      //Self_localization/speed
      speed_stamped_msg.header = msg.header;
      speed_stamped_msg.twist.linear.x = GlobalFrame(0);
      speed_stamped_msg.twist.linear.y = GlobalFrame(1);
      speed_stamped_msg.twist.linear.z = msg.twist.twist.linear.z;
      speed_stamped_msg.twist.angular = msg.twist.twist.angular;       
      self_localization_speed_publisher.publish(speed_stamped_msg);
}

int GroundTruthInterface::getRate(){
    return frecuency;
}

int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, "GroundTruthInterface");

    cout<<"[ROSNODE] Starting GroundTruthInterface"<<endl;

    //Vars
    GroundTruthInterface pose_interface;
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
