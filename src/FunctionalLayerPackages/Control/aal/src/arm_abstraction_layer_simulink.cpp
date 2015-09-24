#include "aal/arm_abstraction_layer_simulink.h"

ArmAbstractionLayerSimulink::ArmAbstractionLayerSimulink(ros::NodeHandle n):
	ArmAbstractionLayer(n),
	action_object_(n)
{
	//Joint names in gazebo.
	joint_names_[0]="shoulder_y_arm_joint";
	joint_names_[1]="shoulder_p_arm_joint";
	joint_names_[2]="elbow_0_p_arm_joint";
	joint_names_[3]="elbow_0_r_arm_joint";
	joint_names_[4]="wirst_0_p_arm_joint";
	joint_names_[5]="wirst_1_r_arm_joint";
	joint_names_[6]="claw_0_y_arm_joint";
	joint_names_[7]="claw_1_y_arm_joint";

	joint_states_sub_ = n.subscribe("/bonebraker/joints_state", 0,
									&ArmAbstractionLayerSimulink::jointStatesCallback,
									this);
	arm_control_references_gazebo_pub_ = n.advertise<arcas_msgs::ArmControlReferencesStamped>("/bonebraker/arm_control_ref_gazebo",0);

	//Initialize matlab model for invert cinematic
	/* Initialize model */
	brazo_ros_initialize();
}

ArmAbstractionLayerSimulink::~ArmAbstractionLayerSimulink()
{
	joint_states_sub_.shutdown();
}


void ArmAbstractionLayerSimulink::jointStatesCallback(const sensor_msgs::JointStateConstPtr &js)
{
	this->last_joint_states_[js->name[0]] = *js;
}

void ArmAbstractionLayerSimulink::receiveJointState(){
	action_object_.updateJointState(&last_joint_states_,joint_names_);
}

void ArmAbstractionLayerSimulink::update()
{
	if(!action_object_.isExtended())
	{

		vector<double> generated_angles = action_object_.getExtensionAngles();

		arm_control_references_to_send_.arm_control_references.position_ref[0] = generated_angles[0];
		arm_control_references_to_send_.arm_control_references.position_ref[1] = generated_angles[1];
		arm_control_references_to_send_.arm_control_references.position_ref[2] = generated_angles[2];
		arm_control_references_to_send_.arm_control_references.position_ref[3] = generated_angles[3];
		arm_control_references_to_send_.arm_control_references.position_ref[4] = generated_angles[4];
		arm_control_references_to_send_.arm_control_references.position_ref[5] = generated_angles[5];
		arm_control_references_to_send_.arm_control_references.position_ref[6] = generated_angles[6];
	}
	else
	{
		//Modo angulos, copiar directamente.
		arm_control_references_to_send_.arm_control_references.position_ref[0] = last_control_command_.arm_control_references.position_ref[0];
		arm_control_references_to_send_.arm_control_references.position_ref[1] = last_control_command_.arm_control_references.position_ref[1];
		arm_control_references_to_send_.arm_control_references.position_ref[2] = last_control_command_.arm_control_references.position_ref[2];
		arm_control_references_to_send_.arm_control_references.position_ref[3] = last_control_command_.arm_control_references.position_ref[3];
		arm_control_references_to_send_.arm_control_references.position_ref[4] = last_control_command_.arm_control_references.position_ref[4];
		arm_control_references_to_send_.arm_control_references.position_ref[5] = last_control_command_.arm_control_references.position_ref[5];
		arm_control_references_to_send_.arm_control_references.position_ref[6] = last_control_command_.arm_control_references.position_ref[6];
	}
	arm_control_references_to_send_.arm_control_references.battery = last_control_command_.arm_control_references.battery;
}

void ArmAbstractionLayerSimulink::sendJointControl()
{
	arm_control_references_to_send_.header.stamp = ros::Time::now();
	arm_control_references_to_send_.header.seq++;
	arm_control_references_to_send_.header.frame_id="aal";
	arm_control_references_gazebo_pub_.publish(arm_control_references_to_send_);
}

void ArmAbstractionLayerSimulink::matlab_step()
{
	//MATLAB CODE
	static boolean_T OverrunFlag = 0;
	if (OverrunFlag) {
	  rtmSetErrorStatus(brazo_ros_M, "Overrun");
	  return;
	}

	OverrunFlag = TRUE;
	brazo_ros_step();
	OverrunFlag = FALSE;
}
