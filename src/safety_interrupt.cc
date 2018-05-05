#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>

// Global variables
std_msgs::UInt8 interrupt_signal;
ros::Publisher signal_interrupt;
// ros::Subscriber safety 	= nh.subscribe("uav_nav/safety", 1, &safetyCb);
double threshold_sec = 0.5;
float ultrasonic_threshold = 2;
float ultrasonic_intensity = 1;
bool safety_isSolved = false;
bool safetyFlag = true;
	
// Callback functions
void ultrasonic_callback(const sensor_msgs::LaserScan& msg)
{
	std::string name = "ultrasonic sensor";
	static double tmp_time_sec = ros::Time::now().toSec();
	double time_now_sec = ros::Time::now().toSec();
	
	// Check range values
	for (int n = 1; n  < 5; n++)
	{
	  if (msg.ranges[n] < ultrasonic_threshold && msg.ranges[n] > 0 && msg.intensities[n] == ultrasonic_intensity)
	  {
	    safetyFlag = true;
            ROS_ERROR("Interrupt signal was triggered. Range reading was %f", msg.ranges[n]);
	  }
	}
	// If range values were
	if (safetyFlag)
	{
	   interrupt_signal.data = 2;
	   signal_interrupt.publish(interrupt_signal);
	}
	else
	{
	   interrupt_signal.data = 0;
	   signal_interrupt.publish(interrupt_signal);
	   ROS_ERROR("Range reading were OK, resetting interrupt signal");
	}
	// Check time between function calls
	if (time_now_sec-tmp_time_sec > threshold_sec)
	{
		interrupt_signal.data = 1;
		signal_interrupt.publish(interrupt_signal);
		ROS_ERROR("Interrupt signal was triggered. Time between %s readings was %f.", name.c_str(), time_now_sec-tmp_time_sec );
	}
	
	tmp_time_sec = time_now_sec;
	safetyFlag = false;
	return;
}
/*
void safetyCb(const std_msgs::Bool& msg)
{
	safety_isSolved = msg->data;
	if (safety_isSolved == true)
	{
		interrupt_signal.data = 0;	// Reset the interrupt signal
	}
	return;
}
*/
void laser_scan_callback(const sensor_msgs::LaserScan& msg)
{
	std::string name = "depth sensor";
	static double tmp_time_sec = ros::Time::now().toSec();
	double time_now_sec = ros::Time::now().toSec();

	if (time_now_sec-tmp_time_sec > threshold_sec)
	{
		interrupt_signal.data = 1;
		signal_interrupt.publish(interrupt_signal);
		ROS_ERROR("Interrupt signal was triggered. Time between %s readings was %f.", name.c_str(), time_now_sec-tmp_time_sec );
	}
	tmp_time_sec = time_now_sec;

	return;
}

void rpy_callback(const geometry_msgs::Vector3Stamped& msg)
{
	std::string name = "roll pitch yaw";
	static double tmp_time_sec = ros::Time::now().toSec();
	double time_now_sec = ros::Time::now().toSec();

	if (time_now_sec-tmp_time_sec > threshold_sec)
	{
		interrupt_signal.data = 1;
		signal_interrupt.publish(interrupt_signal);
		ROS_ERROR("Interrupt signal was triggered. Time between %s readings was %f.", name.c_str(), time_now_sec-tmp_time_sec );
	}
	tmp_time_sec = time_now_sec;

	return;
}

void local_pos_callback(const geometry_msgs::PointStamped& msg)
{
	std::string name = "local position";
	static double tmp_time_sec = ros::Time::now().toSec();
	double time_now_sec = ros::Time::now().toSec();

	if (time_now_sec-tmp_time_sec > threshold_sec)
	{
		interrupt_signal.data = 1;
		signal_interrupt.publish(interrupt_signal);
		ROS_ERROR("Interrupt signal was triggered. Time between %s readings was %f.", name.c_str(), time_now_sec-tmp_time_sec );
	}
	tmp_time_sec = time_now_sec;

	return;
}

void velocity_callback(const geometry_msgs::Vector3Stamped& msg)
{
	std::string name = "velocity";
	static double tmp_time_sec = ros::Time::now().toSec();
	double time_now_sec = ros::Time::now().toSec();

	if (time_now_sec-tmp_time_sec > threshold_sec)
	{
		interrupt_signal.data = 1;
		signal_interrupt.publish(interrupt_signal);
		ROS_ERROR("Interrupt signal was triggered. Time between %s readings was %f.", name.c_str(), time_now_sec-tmp_time_sec );
	}
	tmp_time_sec = time_now_sec;

	return;
}

void vel_cmd_callback(const sensor_msgs::Joy& msg)
{
	std::string name = "velocity control commands";
	static double tmp_time_sec = ros::Time::now().toSec();
	double time_now_sec = ros::Time::now().toSec();

	if (time_now_sec-tmp_time_sec > threshold_sec)
	{
		interrupt_signal.data = 1;
		signal_interrupt.publish(interrupt_signal);
		ROS_ERROR("Interrupt signal was triggered. Time between %s was %f.", name.c_str(), time_now_sec-tmp_time_sec );
	}
	tmp_time_sec = time_now_sec;

	return;
}

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "safety_interrupt");
	ros::NodeHandle nh;

	// Subscribers
	ros::Subscriber ultrasonic_sub	= nh.subscribe("uav_nav/guidance/ultrasonic", 1, &ultrasonic_callback);// sensor_feedback
	ros::Subscriber laser_scan_sub  = nh.subscribe("uav_nav/guidance/laser_scan_from_depthIMG",  1, &laser_scan_callback);// depth_generation
  	ros::Subscriber rpy_sub = nh.subscribe("uav_nav/roll_pitch_yaw", 1, &rpy_callback); // drone_control
 	ros::Subscriber loc_pos_sub = nh.subscribe("dji_sdk/local_position", 1, &local_pos_callback);// dji_sdk
 	ros::Subscriber vel_sub = nh.subscribe("dji_sdk/velocity", 1, &velocity_callback);
	ros::Subscriber vel_cmd_pub = nh.subscribe("uav_nav/vel_cmd", 1, &vel_cmd_callback);// vfh
	
	// Publisher
	signal_interrupt = nh.advertise<std_msgs::UInt8>("uav_nav/signal_interrupt", 1);

	interrupt_signal.data = 0;
	ros::spin();

	return 0;
}