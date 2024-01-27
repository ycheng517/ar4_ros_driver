#include <sstream>
#include <ar3_hardware_interface/ar3_hardware_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>


using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

namespace ar3_hardware_interface
{

AR3HardwareInterface::AR3HardwareInterface(ros::NodeHandle &nh) : nh_(nh)
{
	init();

	// init ros controller manager
	controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

	nh_.param("/ar3/hardware_interface/loop_hz", loop_hz_, 0.1);
	ROS_DEBUG_STREAM_NAMED("constructor", "Using loop freqency of " << loop_hz_ << " hz");
	ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);
	non_realtime_loop_ = nh_.createTimer(update_freq, &AR3HardwareInterface::update, this);

	// initialize controller
	for (int i = 0; i < num_joints_; ++i)
	{
		ROS_DEBUG_STREAM_NAMED("constructor", "Loading joint name: " << joint_names_[i]);

		// Create joint state interface
		JointStateHandle jointStateHandle(joint_names_[i], &joint_positions_[i], &joint_velocities_[i], &joint_efforts_[i]);
		joint_state_interface_.registerHandle(jointStateHandle);

		// Create position joint interface
		JointHandle jointPositionHandle(jointStateHandle, &joint_position_commands_[i]);
		position_joint_interface_.registerHandle(jointPositionHandle);
	}

	// get encoder calibration
	std::vector<double> enc_steps_per_deg(num_joints_);

	for (int i = 0; i < num_joints_; ++i)
	{
		if(!nh_.getParam("/ar3/hardware_driver/encoder_steps_per_deg/" + joint_names_[i], enc_steps_per_deg[i]))
		{
			ROS_WARN("Failed to get params for /ar3/hardware_driver/encoder_steps_per_deg/");
		}

		if (!nh_.getParam("/ar3/hardware_interface/joint_offsets/" + joint_names_[i], joint_offsets_[i]))
		{
			ROS_WARN("Failed to get params for /ar3/hardware_interface/joint_offsets/");
		}

	}

	// init motor driver
	std::string serial_port;
	int baudrate;
	nh_.getParam("/ar3/hardware_driver/serial_port", serial_port);
	nh_.getParam("/ar3/hardware_driver/baudrate", baudrate);
	driver_.init(serial_port, baudrate, num_joints_, enc_steps_per_deg);

	// set velocity limits
	for (int i = 0; i < num_joints_; ++i)
	{
		nh_.getParam("/ar3/joint_limits/" + joint_names_[i] + "/max_velocity", velocity_limits_[i]);
		nh_.getParam("/ar3/joint_limits/" + joint_names_[i] + "/max_acceleration", acceleration_limits_[i]);
		velocity_limits_[i] = radToDeg(velocity_limits_[i]);
		acceleration_limits_[i] = radToDeg(acceleration_limits_[i]);
	}
	driver_.setStepperSpeed(velocity_limits_, acceleration_limits_);

	// calibrate joints if needed
	bool use_existing_calibrations = false;
	nh_.getParam("/ar3/hardware_interface/use_existing_calibrations", use_existing_calibrations);
	if (!use_existing_calibrations)
	{
		// run calibration
		ROS_INFO("Running joint calibration...");
		driver_.calibrateJoints();
	}

	// init position commands at current positions
	driver_.getJointPositions(actuator_positions_);
	for (int i = 0; i < num_joints_; ++i)
	{
		// apply offsets, convert from deg to rad for moveit
		joint_positions_[i] = degToRad(actuator_positions_[i] + joint_offsets_[i]);
		joint_position_commands_[i] = joint_positions_[i];
	}

	registerInterface(&joint_state_interface_);
	registerInterface(&position_joint_interface_);
}

AR3HardwareInterface::~AR3HardwareInterface()
{
}

void AR3HardwareInterface::init()
{
	// get joint names
	nh_.getParam("/ar3/hardware_interface/joints", joint_names_);
	if (joint_names_.size() == 0)
	{
		ROS_FATAL_STREAM_NAMED("init", "No joints found on parameter server for controller. Did you load the proper yaml file?");
	}
	num_joints_ = static_cast<int>(joint_names_.size());

	// resize vectors
	actuator_commands_.resize(num_joints_);
	actuator_positions_.resize(num_joints_);
	joint_positions_.resize(num_joints_);
	joint_velocities_.resize(num_joints_);
	joint_efforts_.resize(num_joints_);
	joint_position_commands_.resize(num_joints_);
	joint_velocity_commands_.resize(num_joints_);
	joint_effort_commands_.resize(num_joints_);
	joint_offsets_.resize(num_joints_);
	joint_lower_limits_.resize(num_joints_);
	joint_upper_limits_.resize(num_joints_);
	velocity_limits_.resize(num_joints_);
	acceleration_limits_.resize(num_joints_);
}

void AR3HardwareInterface::update(const ros::TimerEvent &e)
{
	std::string logInfo = "\n";
	logInfo += "Joint Position Command:\n";
	for (int i = 0; i < num_joints_; i++)
	{
		std::ostringstream jointPositionStr;
		jointPositionStr << radToDeg(joint_position_commands_[i]);
		logInfo += "  " + joint_names_[i] + ": " + jointPositionStr.str() + "\n";
	}

	elapsed_time_ = ros::Duration(e.current_real - e.last_real);
	
	write(elapsed_time_);
	read();

	logInfo += "Joint Position State:\n";
	for (int i = 0; i < num_joints_; i++)
	{
		std::ostringstream jointPositionStr;
		jointPositionStr << radToDeg(joint_positions_[i]);
		logInfo += "  " + joint_names_[i] + ": " + jointPositionStr.str() + "\n";
	}

	controller_manager_->update(ros::Time::now(), elapsed_time_);

	ROS_INFO_STREAM(logInfo);
}

void AR3HardwareInterface::read()
{		
	driver_.update(actuator_commands_, actuator_positions_);
	for (int i = 0; i < num_joints_; ++i)
	{
		// apply offsets, convert from deg to rad for moveit
		joint_positions_[i] = degToRad(actuator_positions_[i] + joint_offsets_[i]);
	}
}

void AR3HardwareInterface::write(ros::Duration elapsed_time)
{
	for (int i = 0; i < num_joints_; ++i)
	{
		// convert from rad to deg, apply offsets
		actuator_commands_[i] = radToDeg(joint_position_commands_[i]) - joint_offsets_[i];
	}
}

double AR3HardwareInterface::degToRad(double deg)
{
	return deg / 180.0 * M_PI;
}

double AR3HardwareInterface::radToDeg(double rad)
{
	return rad / M_PI * 180.0;
}

} // namespace ar3_hardware_interface
