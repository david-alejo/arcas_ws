#include<motion_action_handler/motionactionhandler.h>

int main(int argc, char** argv)
{

  ros::init(argc, argv, "motion_action_handler");

  ros::NodeHandle n(MODULE_NAMESPACE);

  MotionActionHandler motion_action_handler(&n);

  ros::spin();

  //Finish.

  return 0;
}
