/*!
 * \file
 * \brief Motion Action Handler Node
 *  this node has to control all actions in ARCAS environment
 */
#ifndef	MOTION_ACTION_HANDLER_H
#define MOTION_ACTION_HANDLER_H
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <arcas_module_main/AARCASModuleMain.h>
#include <motion_action_handler/actions/takeoff_action_client.h>
#include <motion_action_handler/actions/close_interaction_action_client.h>
#include <motion_action_handler/actions/land_action_client.h>
#include <motion_action_handler/actions/translation_action_client.h>
#include <motion_action_handler/actions/inserction_check_action_client.h>
#include <motion_action_handler/actions/extend_arm_action_client.h>


/*!
 * \brief Framerate for this node.
 */
#define MODULE_RATE 5.0

/*!
 * \brief Namespace for this node's topics
 */
#define MODULE_NAMESPACE "control"

/*!
 * \brief Posibles states of the control state machine
 */
enum States{Init,
			SendTakeOff,
			WaitForTakedOff,
			SendTranslationAction,
			WaitingForTranslationAction,
            SendExtendArmAction,
            WaitingForExtendArmAction,
            SendContractArmAction,
            WaitingForContractArmAction,
            SendLandAction,
            WaitingForLandAction,
			Done};

/*!
 * \brief Main class of Motion Action Handler.
 */
class MotionActionHandler : public AARCASModuleMain
{
public:
	MotionActionHandler(ros::NodeHandle *n);
	~MotionActionHandler();
private:


    /*!
     * \brief Main loop, to update the node state.
     * \param TimerEvent data from ROS.
     */
	void mainLoop(const ros::TimerEvent& te);

    /*
     *Decide the next state of the control state machine.
    */
	States nextState();


    /*!
     * \brief Current state of control state machine
     */
	States current_state;

    /*!
     * \brief Action client to send takeoff tasks to uav
     */
    TakeOffActionWrapper takeoffActionclient;

    /*!
     * \brief Action client to send land tasks to uav
     */
    LandActionWrapper landActionClient;

    /*!
     * \brief Action client to send Close Interaction task to uav
     */
    CloseInteractionActionWrapper closeInteractionActionClient;

    /*!
     * \brief Action client to move the uav
     */
    TranslationActionWrapper translationActionClient;

    /*!
     * \brief Action client to send Check Inserction of the bar
     */
    InserctionCheckActionWrapper inserctionCheckActionClient;

    /*!
     * \brief Action client to contract/extend actionClient
     */
    AALExtensionActionWrapper aalExtensionActionClient;


	/*
	 *Translation goal message
	*/
	TranslationGoal translation_goal_;

};

#endif // MOTION_ACTION_HANDLER_H
