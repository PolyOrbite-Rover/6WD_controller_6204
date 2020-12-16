/*
 * Copyright (C) 2017, Jonathan Cacace

 * Email id : jonathan.cacace@gmail.com

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
*/

 #pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>

#include <stdlib.h> 
#include <iostream>
#include <std_msgs/Float64MultiArray.h> 
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include <tf/tf.h>

namespace controller_6204_ns {

	class Controller_6204: public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
	public:
		//Controller_6204();
		//virtual ~Controller_6204();
		bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
		void update(const ros::Time& time, const ros::Duration& period);
		void starting(const ros::Time& time);
		void stopping(const ros::Time& time);
		
		void ground_truth_Callback(const nav_msgs::Odometry::ConstPtr& msg);
		void velocity_command_Callback(const geometry_msgs::Twist::ConstPtr& msg);
		double * getTorque(const ros::Duration& period);

	private:
		std::vector<hardware_interface::JointHandle> effortJointHandles;

        std::vector<double> buffer_current_positions;
        std::vector<double> buffer_current_velocities;
        std::vector<double> buffer_past_velocities;
        std::vector<double> buffer_current_efforts;
        double u;
        double v;
        double r;
        double theta;
        
        double ud;
        double rd = 0;
		double thetad;	
        
		double prevEpsilon_theta = 0;
		double prevUd = 0;
		double prevRd = 0;
		 
        bool receivedCommand = 0;
        
		ros::Subscriber sub_command_;
		ros::Subscriber sub_command2_;
		ros::Publisher pub_torque_;
	};
}


