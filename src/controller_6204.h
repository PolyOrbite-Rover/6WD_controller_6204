
/* This project is created for the class ELE6204A at Polytechnique Montreal, as a project on nonlinear systems.
 * 
 * This is the implementation of a 6WD controller based on the article: 
 * "Dynamic control of the 6WD skid-steering mobile robot RobuROC6
using sliding mode technique" by E. Lucet et al.
 * 
 * This ROS controller is based on the tutorials and examples of of: 
 * Jonathan Cacace - https://github.com/jocacace/my_controller
 * NASA JSC Robotics - https://gitlab.com/nasa-jsc-robotics/valkyrie/-/wikis/How-To-Write-A-Controller-For-Valkyrie
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


