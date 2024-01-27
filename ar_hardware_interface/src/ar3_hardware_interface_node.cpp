#include <ros/callback_queue.h>
#include <ar3_hardware_interface/ar3_hardware_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ar3_hardware_interface");
	ros::CallbackQueue ros_queue;

  ros::NodeHandle nh;
	nh.setCallbackQueue(&ros_queue);
  ar3_hardware_interface::AR3HardwareInterface ar3(nh);

	ros::MultiThreadedSpinner spinner(0);
	spinner.spin(&ros_queue);

  return 0;
}
