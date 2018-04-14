#include "uav_nav/test_vel_cmd.h"

// Global variables
ros::ServiceClient               query_version_service;
ros::ServiceClient               set_loc_pos_ref_service;
ros::ServiceClient               sdk_ctrl_authority_service;
ros::ServiceClient               drone_task_service;
ros::Publisher 				           ctrl_vel_cmd_pub;                  // Velocity control command sent to the FC
ros::Publisher 				           rpy_pub;                           // Publish roll, pitch, yaw in radians
geometry_msgs::TwistStamped	     vel_cmd;
geometry_msgs::QuaternionStamped attitude_state;
sensor_msgs::NavSatFix           current_gps_position;
uint8_t                          flight_status               = 255; // Enum representing drone state upon take off
uint8_t                          current_gps_health          = 0;   // Number of GPS satellite connections
int 						                 ctrl_state 	        			 = 0;   // State machine controller

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_vel_cmd");
  ros::NodeHandle nh;

  // Services
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>	  ("dji_sdk/query_drone_version");
  set_loc_pos_ref_service    = nh.serviceClient<dji_sdk::SetLocalPosRef>		  ("dji_sdk/set_local_pos_ref");
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>	("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>	  ("dji_sdk/drone_task_control");

  // Subscribe to messages from dji_sdk_node
  ros::Subscriber flight_status_sub = nh.subscribe("dji_sdk/flight_status",   1, &flightStatusCb);
  ros::Subscriber gps_pos_sub       = nh.subscribe("dji_sdk/gps_position",    1, &GPSPositionCb);
  ros::Subscriber gps_health_sub    = nh.subscribe("dji_sdk/gps_health",      1, &GPSHealthCb);
  ros::Subscriber attitude 		      = nh.subscribe("dji_sdk/attitude",        1, &attitudeCb);

  // Publish the control signal
  ctrl_vel_cmd_pub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 1);
  rpy_pub          = nh.advertise<geometry_msgs::Vector3Stamped>("uav_nav/roll_pitch_yaw",     1);

  ros::Duration(20).sleep(); // Finger-preservation delay

  if(isM100() && setLocalPositionRef())
  {
    bool ready = obtainControl(true) ? monitoredTakeOff() : false;
    if(ready)
    ctrl_state = 1;

    ros::Duration(1).sleep();
    ros::spinOnce();
  }
  else
  {
    ROS_ERROR("Only the M100 is supported! || Could not set local position reference.");
    return 0;
  }

  switch(ctrl_state) {
    case 0:	break;
    case 1:
      sendDemoCmd(1.5, 1, 0, 0); 	// Fly forward for 1.5 sec...
      sendDemoCmd(5, 0, 0, 0);		// ...Wait for 5 sec...
      sendDemoCmd(1.5, -1, 0, 0);	// ...and fly back
      sendDemoCmd(5, 0, 0, 0);
      sendDemoCmd(1.5, 0, 1, 0);	// Fly along y axis for 1.5 sec
      sendDemoCmd(5, 0, 0, 0);
      sendDemoCmd(1.5, 0, -1, 0);
      sendDemoCmd(1.5, 0, 0, 1);  // Positive yawrate
      break;
  }

  sendDemoCmd(5, 0, 0, 0);  // Stop
  monitoredLanding();
  obtainControl(false);

  return 0;
}

// Services
bool isM100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  return (query.response.version == DJISDK::DroneFirmwareVersion::M100_31);
}

bool setLocalPositionRef()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_loc_pos_ref_service.call(localPosReferenceSetter);

  return (bool)localPosReferenceSetter.response.result;
}

bool obtainControl(bool b)
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable = b ? 1 : 0;
  sdk_ctrl_authority_service.call(authority);

  return authority.response.result;
}

bool takeoffLand(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = task;
  drone_task_service.call(droneTaskControl);

  return droneTaskControl.response.result;
}

bool monitoredTakeOff()
{
  ros::Time start_time = ros::Time::now();
  float home_altitude = current_gps_position.altitude;

  if(!takeoffLand(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  return false;

  ROS_DEBUG("M100 taking off!");
  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // If M100 is not in the air after 10 seconds, fail.
  while(ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR || (current_gps_position.altitude - home_altitude < 1.0))
  {
    ROS_ERROR("Take-off failed.");
    return false;
  }
  else
  {
    ROS_DEBUG("Successful take-off!");
    ros::spinOnce();
  }

  return true;
}

bool monitoredLanding()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoffLand(dji_sdk::DroneTaskControl::Request::TASK_LAND))
    return false;

  ROS_DEBUG("M100 landing!");
  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // If M100 is not on the ground after 10 seconds, fail.
  while(ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_FINISHED_LANDING)
  {
    ROS_ERROR("Landing failed.");
    return false;
  }
  else
  {
    ROS_DEBUG("Successful take-off!");
    ros::spinOnce();
  }

  return true;
}

// Callbacks
void flightStatusCb(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void GPSPositionCb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  current_gps_position = *msg;
}

void GPSHealthCb(const std_msgs::UInt8::ConstPtr& msg)
{
  current_gps_health = msg->data;


  if (current_gps_health <= 3)
  {
    ctrl_state = 0;
    ROS_WARN("Cannot execute local position control. Not enough GPS satellites");
  }
}

void attitudeCb(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  attitude_state = *msg;

  quatToEuler();
}

void sendDemoCmd(float delay, float xCmd, float yCmd, float yawCmd)
{
  ROS_DEBUG("Vx: %.2f /t Vy: %.2f /t Yaw_rate: %.2f", xCmd, yCmd, yawCmd);

  sensor_msgs::Joy controlVelYawRate;
  uint8_t flag = (DJISDK::VERTICAL_VELOCITY | DJISDK::HORIZONTAL_VELOCITY | DJISDK::YAW_RATE | DJISDK::HORIZONTAL_BODY | DJISDK::STABLE_ENABLE);
  controlVelYawRate.axes.push_back(xCmd);
  controlVelYawRate.axes.push_back(yCmd);
  controlVelYawRate.axes.push_back(0);
  controlVelYawRate.axes.push_back(yawCmd);
  controlVelYawRate.axes.push_back(flag);

  ros::Time start_time = ros::Time::now();
  while((ros::Time::now() - start_time) < ros::Duration(delay) && ros::ok())
  {
    ctrl_vel_cmd_pub.publish(controlVelYawRate);
    ros::spinOnce();
  }
}

void quatToEuler()
{
  geometry_msgs::Vector3Stamped rpy;
  rpy.header.stamp    = ros::Time::now();
  rpy.header.frame_id = "RPY";

  tf::Quaternion q = tf::Quaternion(attitude_state.quaternion.x,
                                    attitude_state.quaternion.y,
                                    attitude_state.quaternion.z,
                                    attitude_state.quaternion.w);
  tf::Matrix3x3(q).getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);

  rpy.vector.z = wrapToPi(rpy.vector.z - C_PI/2); // Proper FLU - 'x' towards North = 0 yaw; yaw > 0 ccw

  rpy_pub.publish(rpy);
}
