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


#include "controller_6204.h"



	
namespace controller_6204_ns {
	
	//Constructor
	//Controller_6204::Controller_6204()
	//{}
	// Desctructor
	//Controller_6204::Controller_6204()
	//{}
	
	
		
	void Controller_6204::velocity_command_Callback(const geometry_msgs::Twist::ConstPtr& msg)
	{
		ud = msg->linear.x;
		//thetad = msg->angular.z;
		rd = msg->angular.z;
		receivedCommand = 1;
	}
	
	//Ground_truth_Callback
	void Controller_6204::ground_truth_Callback(const nav_msgs::Odometry::ConstPtr& msg)
	{

		tf::Quaternion q(
			msg->pose.pose.orientation.x,
			msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z,
			msg->pose.pose.orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch;
		m.getRPY(roll, pitch, theta);
		
		u = cos(theta)*msg->twist.twist.linear.x + sin(theta)*msg->twist.twist.linear.y;
		v = -sin(theta)*msg->twist.twist.linear.x + cos(theta)*msg->twist.twist.linear.y;
		r = msg->twist.twist.angular.z;
		
		
	}

	

	
	//Controller initialization
	bool Controller_6204::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
	{
		effortJointHandles.clear();

		//Retrieve the joint object to control
		std::vector<std::string> jointNames;
		
		
		if(n.getParam("joints", jointNames))
		{   
			if(hw)
			{
				for(unsigned int i=0; i < jointNames.size(); ++i)
				{
					//Ici, les roues gauches sont aux positions 0 à 3
					effortJointHandles.push_back(hw->getHandle(jointNames[i]));
					std::cout << jointNames[i] << " :" << i << std::endl;
				}
				
				buffer_current_positions.resize(effortJointHandles.size(), 0.0);
				buffer_current_velocities.resize(effortJointHandles.size(), 0.0);
				buffer_past_velocities.resize(effortJointHandles.size(), 0.0);
				buffer_current_efforts.resize(effortJointHandles.size(), 0.0);
          }
          else
          {
              ROS_ERROR("Effort Joint Interface is empty in hardware interace.");
              return false;
          }
       }
       else
       {
          ROS_ERROR("No joints in given namespace: %s", n.getNamespace().c_str());
          return false;
       }

	//Suscribe to ground truth
	sub_command_ = n.subscribe("/ground_truth/state", 1, &Controller_6204::ground_truth_Callback, this);
	
	//Suscribe to velocity command
	sub_command2_ = n.subscribe("/control_6204/command", 5, &Controller_6204::velocity_command_Callback,this);
	
	//Publisher of torque data
	pub_torque_ = n.advertise<std_msgs::Float64MultiArray>("/control_6204/torque", 1);
	
	return true;
	}

	//Controller startup
	void Controller_6204::starting(const ros::Time& time) 
	{
    	for(unsigned int i = 0; i < effortJointHandles.size(); ++i)
		{
			buffer_current_positions[i] = effortJointHandles[i].getPosition();
			buffer_current_velocities[i] = effortJointHandles[i].getVelocity();
		}
	}
	
	
	
	// Torque calculation according to "Dynamic control of the 6WD skid-steering mobile robot RobuROC6 using sliding mode technique"
	double * Controller_6204::getTorque(const ros::Duration& duration)
	{
		std_msgs::Float64MultiArray torque_msg;
		double dt = double(duration.sec) + double(duration.nsec)*1e-9;
		
		double dotOmega_bl = (buffer_current_velocities[0] - buffer_past_velocities[0])/dt;
		double dotOmega_fl = (buffer_current_velocities[1] - buffer_past_velocities[1])/dt;
		double dotOmega_ml = (buffer_current_velocities[2] - buffer_past_velocities[2])/dt;
		double dotOmega_br = (buffer_current_velocities[3] - buffer_past_velocities[3])/dt; 
		double dotOmega_fr = (buffer_current_velocities[4] - buffer_past_velocities[4])/dt;
		double dotOmega_mr = (buffer_current_velocities[5] - buffer_past_velocities[5])/dt;
	
  
		double dotOmega= dotOmega_bl + dotOmega_fl + dotOmega_ml + dotOmega_br + dotOmega_fr + dotOmega_mr;
  
	
				//Robot parameters
		double length = 1; //meters
		double width = 0.5; //meters
		double height = 0.474; //meters
		double inertia = 90.0; //kg*m2
		double mass = 45.0; //kg
		double Rwheel = 0.127; //m
		double Mwheel = 1.0; //kg
		double Inwheel = 0.011; //kg*m2
		
		
		//Gain and simulation parameters
		double Ku_p = 1.0; //s^-1
		double Ktheta_p = 12.0; // s^-2
		double Ktheta_d = 3; //s^-1
		double xi = 0.7;
		double Tr = 2; //s
		double beta_u = 0.01; //ms^-1
		double beta_theta = 0.01;
		double a = 0.1;
		double b = 0.1;
		
		//Sliding mode control gains
		double rho = 1.0; //ms^-2
		double mu = 18.0;
		
		
		
		
		// Longitudinal control - Calculated parameters
		double epsilon_u = ud-u; 
		double dotu_d = (prevUd-ud)/dt;
		
		double gamma = 6/(Rwheel*mass);
		double lambdaMaj_u = -Inwheel/(Rwheel*mass);
		
		// Longitudinal control - Torque
		double torque_long = (1/gamma)*(dotu_d + Ku_p*epsilon_u + rho*epsilon_u/(std::norm(epsilon_u)+beta_u) - lambdaMaj_u*dotOmega - r*v);
		
		
		// Angular control - Calculated parameters
		//rd=5*(thetad-theta);
		thetad=theta; //10*(theta+(rd-r)*1);
		
		double epsilon_theta = thetad-theta;
		double dotr_d = (prevRd-rd)/dt; 
		
		
		double dotepsilon_theta = (rd-r)/dt;//(prevEpsilon_theta-epsilon_theta)/dt;
		
		
		double lambda = 3*width/2*inertia*Rwheel;
		
		double lambdaMaj_theta = width*Inwheel/(inertia*Rwheel);
		
		double p21 = a*xi*xi*Tr*Tr/35.28;
		double p22 = b*Tr/16.8 + a*xi*xi*Tr*Tr/296.352;
		double BtPx = p21*epsilon_theta+p22*dotepsilon_theta;
		
		double torque_ang = (1/lambda)*(dotr_d + Ktheta_p*epsilon_theta + Ktheta_d*dotepsilon_theta + mu*BtPx/(std::norm(BtPx)+beta_theta) -lambdaMaj_theta*dotOmega); 
		std::cout << "dotr_d: " << dotr_d << " Ktheta_p*epsilon_theta: " << Ktheta_p*epsilon_theta << " Ktheta_d*dotepsilon_theta: " << Ktheta_d*dotepsilon_theta <<" Gros terme angulaire: " << mu*BtPx/(std::norm(BtPx)+beta_theta) << " LambdaMaj_u*dotOmega: " << lambdaMaj_theta*dotOmega << std::endl;
		std::cout << "r: " << r << " rd: " << rd << std::endl;
		// Assigning previous values for the next iteration
		buffer_past_velocities = buffer_current_velocities;
		prevEpsilon_theta = epsilon_theta;
		prevUd = ud;
		prevRd = rd;
		
		
		
		//Torque_left, torque_right
		static double torque[2];
		torque[0]=torque_long - 0.5*torque_ang;
		torque[1]=torque_long + 0.5*torque_ang;
		
		torque_msg.data.push_back(torque_long);
		torque_msg.data.push_back(torque_ang);
		torque_msg.data.push_back(torque[0]);
		torque_msg.data.push_back(torque[1]);
		pub_torque_.publish(torque_msg);
		std::cout << "Wheel velocities: " << buffer_current_velocities[0] << " " << buffer_current_velocities[1] << " " << buffer_current_velocities[2] << " " << buffer_current_velocities[3] << " " << buffer_current_velocities[4] << " "<< buffer_current_velocities[5]  << std::endl;
		//std::cout << "torque left: " << torque[0] << " torque_right: " << torque[1]  << " theta: " << theta <<std::endl;
	
		return torque;
	}


	//Controller running
	void Controller_6204::update(const ros::Time& time, const ros::Duration& period)
	{
		for(unsigned int i = 0; i < effortJointHandles.size(); ++i)
		{
			buffer_current_positions[i] = effortJointHandles[i].getPosition();
			buffer_current_velocities[i] = effortJointHandles[i].getVelocity();
		}   
		if (receivedCommand){
			double *torque;
			torque = Controller_6204::getTorque(period);


			
			effortJointHandles[0].setCommand(torque[0]);
			effortJointHandles[1].setCommand(torque[0]);
			effortJointHandles[2].setCommand(torque[0]);
			effortJointHandles[3].setCommand(torque[1]);
			effortJointHandles[4].setCommand(torque[1]);
			effortJointHandles[5].setCommand(torque[1]);
			
		}
	}

	//Controller exiting
	void Controller_6204::stopping(const ros::Time& time) { }

}

//Register the plugin: PLUGINLIB_EXPORT_CLASS(my_namespace::MyPlugin, base_class_namespace::PluginBaseClass)
PLUGINLIB_EXPORT_CLASS(controller_6204_ns::Controller_6204, controller_interface::ControllerBase);
