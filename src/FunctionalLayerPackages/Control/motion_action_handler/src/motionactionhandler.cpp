#include <motion_action_handler/motionactionhandler.h>


MotionActionHandler::MotionActionHandler(ros::NodeHandle *n):
	AARCASModuleMain(n, MODULE_RATE),
	takeoffActionclient(std::string("/ual_1")),
	landActionClient(std::string("/ual_1")),
	translationActionClient(std::string("/ual_1"))
{
	std::cerr << "Waiting for TakeOff Action Server" << std::endl;
	takeoffActionclient.waitForServer();

	std::cerr << "Waiting for Land Action Server" << std::endl;
	landActionClient.waitForServer();

	std::cerr << "Waiting for Close Interaction Action Server" << std::endl;
	closeInteractionActionClient.waitForServer();

	std::cerr << "Waiting for Translation Action Server" << std::endl;
	translationActionClient.waitForServer();

	std::cerr << "Waiting for Inserction Action Server" << std::endl;
	inserctionCheckActionClient.waitForServer();

	std::cerr << "Waiting for AAL Extension Action Server" << std::endl;
	aalExtensionActionClient.waitForServer();

	std::cerr << std::endl << std::endl << "All Servers OK!!!" << std::endl << std::endl ;


	//Demo initialization

	arcas_msgs::AerialVehicleTrajectoryPoint point;
	point.quad_control_ref.heading = 0;
	point.quad_control_ref.velocity_ref = 0.5;
	point.quad_control_ref.position_ref.status = 1;

	point.quad_control_ref.position_ref.x = 2.0;
	point.quad_control_ref.position_ref.y = 2.0;
	point.quad_control_ref.position_ref.z = 2.0;
	translation_goal_.trajectory_points.push_back(point);


	point.quad_control_ref.position_ref.x = -2.0;
	point.quad_control_ref.position_ref.y = 2.0;
	point.quad_control_ref.position_ref.z = 1.0;
	translation_goal_.trajectory_points.push_back(point);


	point.quad_control_ref.position_ref.x = -2.0;
	point.quad_control_ref.position_ref.y = -2.0;
	point.quad_control_ref.position_ref.z = 2.2;
	translation_goal_.trajectory_points.push_back(point);

	point.quad_control_ref.position_ref.x = 0;
	point.quad_control_ref.position_ref.y = 0;
	point.quad_control_ref.position_ref.z = 1.2;
	translation_goal_.trajectory_points.push_back(point);

	usleep(3000000);
	current_state = Init;
}

MotionActionHandler::~MotionActionHandler()
{

}

void MotionActionHandler::mainLoop(const ros::TimerEvent& te)
{
	current_state = nextState();

}
States MotionActionHandler::nextState()
{
	States next_state = current_state;
	switch (current_state) {
		case Init:
			next_state = SendTakeOff;
			break;
		case SendTakeOff:
			std::cerr << "Sending TakeOff" << std::endl;
			takeoffActionclient.takeOff();
			next_state = WaitForTakedOff;
			break;
		case WaitForTakedOff:
			if(!takeoffActionclient.hasGoalRunning())
			{
				next_state = SendTranslationAction;
			}
			break;
		case SendTranslationAction:
			std::cerr << "Sending Translation Action" << std::endl;
			if(translationActionClient.initTranslation(translation_goal_))
			{
				next_state = WaitingForTranslationAction;
			}
			break;
		case WaitingForTranslationAction:
			if(!translationActionClient.hasGoalRunning())
			{
				next_state = SendExtendArmAction;
			}
			break;
		case SendExtendArmAction:
		std::cerr << "Sending Extend Action" << std::endl;
			if(aalExtensionActionClient.extendArm())
			{
				next_state = WaitingForExtendArmAction;
			}
			break;
		case WaitingForExtendArmAction:
		if(!aalExtensionActionClient.hasGoalRunning())
		{
			next_state = SendContractArmAction;
		}
			break;
		case SendContractArmAction:
			std::cerr << "Sending Contract Action" << std::endl;
			if(aalExtensionActionClient.contractArm())
			{
				next_state = WaitingForContractArmAction;
			}
			break;
		case WaitingForContractArmAction:
			if(!aalExtensionActionClient.hasGoalRunning())
			{
				next_state = SendLandAction;
			}
			break;
		case SendLandAction:
			std::cerr << "Sending Land Action" << std::endl;
			if(landActionClient.land())
			{
				next_state = WaitingForLandAction;
			}
			break;
		case WaitingForLandAction:
			if(!landActionClient.hasGoalRunning())
			{
				next_state = Done;
			}
			break;
		default:
			break;
	}
	return next_state;
}
