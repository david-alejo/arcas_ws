#include "aal/arm_abstraction_layer_gazebo.h"

ArmAbstractionLayerGazebo::ArmAbstractionLayerGazebo(ros::NodeHandle n):
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

	//Initialize joint names to send.
	command_to_send_[0].joint_name = joint_names_[0];
	command_to_send_[1].joint_name = joint_names_[1];
	command_to_send_[2].joint_name = joint_names_[2];
	command_to_send_[3].joint_name = joint_names_[3];
	command_to_send_[4].joint_name = joint_names_[4];
	command_to_send_[5].joint_name = joint_names_[5];

	//Claw
	command_to_send_[6].joint_name = joint_names_[6];
	command_to_send_[7].joint_name = joint_names_[6];

	joint_states_sub_ = n.subscribe("/bonebraker/joints_state", 0,
									&ArmAbstractionLayerGazebo::jointStatesCallback,
									this);
	joint_gazebo_pub_ = n.advertise<arcas_msgs::JointControl>("/bonebraker/set_joint_position",0);

	//Initialize matlab model for invert cinematic
	/* Initialize model */
	brazo_ros_initialize();
}

ArmAbstractionLayerGazebo::~ArmAbstractionLayerGazebo()
{
	joint_states_sub_.shutdown();
}


void ArmAbstractionLayerGazebo::jointStatesCallback(const sensor_msgs::JointStateConstPtr &js)
{
	if(js->name[0]==joint_names_[0])
	{
		this->last_joint_states_[js->name[0]] = *js;
		this->last_joint_states_[js->name[0]] = this->last_joint_states_[js->name[0]];
	}
	if(js->name[0]==joint_names_[1])
	{
		this->last_joint_states_[js->name[0]] = *js;
		this->last_joint_states_[js->name[0]].position[0] = (this->last_joint_states_[js->name[0]].position[0] + (3.14159))*(-1);
	}
	if(js->name[0]==joint_names_[2])
	{
		this->last_joint_states_[js->name[0]] = *js;
		this->last_joint_states_[js->name[0]].position[0] = (this->last_joint_states_[js->name[0]].position[0] - 3.14159)*(-1);
	}
	if(js->name[0]==joint_names_[3])
	{
		this->last_joint_states_[js->name[0]] = *js;
		this->last_joint_states_[js->name[0]] = this->last_joint_states_[js->name[0]];
	}
	if(js->name[0]==joint_names_[4])
	{
		this->last_joint_states_[js->name[0]] = *js;
		this->last_joint_states_[js->name[0]].position[0] = this->last_joint_states_[js->name[0]].position[0]*(-1);
	}
	if(js->name[0]==joint_names_[5])
	{
		this->last_joint_states_[js->name[0]] = *js;
		this->last_joint_states_[js->name[0]].position[0] = this->last_joint_states_[js->name[0]].position[0] + 1.5707;
	}
	if(js->name[0]==joint_names_[6])
	{
		this->last_joint_states_[js->name[0]] = *js;
		this->last_joint_states_[js->name[0]] = this->last_joint_states_[js->name[0]];
	}
}

/*!
 * \brief Update the next angles commands and send
 */
void ArmAbstractionLayerGazebo::updateAALState()
{
	if(last_joint_states_.size()>5)
	{
		arm_state_estimation_to_send_.header.seq++;
		arm_state_estimation_to_send_.header.stamp = ros::Time::now();

		static JointState *joint1,*joint2,*joint3,*joint4,*joint5,*joint6,*joint_claw;
		joint1 = &last_joint_states_[joint_names_[0]];
		joint2 = &last_joint_states_[joint_names_[1]];
		joint3 = &last_joint_states_[joint_names_[2]];
		joint4 = &last_joint_states_[joint_names_[3]];
		joint5 = &last_joint_states_[joint_names_[4]];
		joint6 = &last_joint_states_[joint_names_[5]];
		joint_claw = &last_joint_states_[joint_names_[6]];

		arm_state_estimation_to_send_.arm_state_estimation.position[0] = joint1->position[0];
		arm_state_estimation_to_send_.arm_state_estimation.position[1] = joint2->position[0];
		arm_state_estimation_to_send_.arm_state_estimation.position[2] = joint3->position[0];
		arm_state_estimation_to_send_.arm_state_estimation.position[3] = joint4->position[0];
		arm_state_estimation_to_send_.arm_state_estimation.position[4] = joint5->position[0];
		arm_state_estimation_to_send_.arm_state_estimation.position[5] = joint6->position[0];
		arm_state_estimation_to_send_.arm_state_estimation.position[6] = joint_claw->position[0];

	}
		//Hay que incluirlo
		if(action_object_.isExtended())
			arm_state_estimation_to_send_.arm_state_estimation.arm_state = arm_state_estimation_to_send_.arm_state_estimation.EXTENDED;
		else
			arm_state_estimation_to_send_.arm_state_estimation.arm_state = arm_state_estimation_to_send_.arm_state_estimation.CONTRACTED;

}

void ArmAbstractionLayerGazebo::receiveJointState(){
	action_object_.updateJointState(&last_joint_states_,joint_names_);
}

void ArmAbstractionLayerGazebo::update()
{
	if(!action_object_.isExtended())
	{

		vector<double> generated_angles = action_object_.getExtensionAngles();

		command_to_send_[0].position = generated_angles[0];
		command_to_send_[1].position = (generated_angles[1]+ 3.14159)*(-1);
		command_to_send_[2].position = (generated_angles[2] - 3.14159)*(-1);
		command_to_send_[3].position = generated_angles[3];
		command_to_send_[4].position = generated_angles[4]*(-1);
		command_to_send_[5].position = generated_angles[5]- 1.5707;
		command_to_send_[6].position = generated_angles[6];


	}
	else
	{
		//Modo angulos, copiar directamente.
		command_to_send_[0].position = last_control_command_.arm_control_references.position_ref[0];
		command_to_send_[1].position = (last_control_command_.arm_control_references.position_ref[1] + 3.14159)*(-1);
		command_to_send_[2].position = (last_control_command_.arm_control_references.position_ref[2] - 3.14159)*(-1);
		command_to_send_[3].position = last_control_command_.arm_control_references.position_ref[3];
		command_to_send_[4].position = last_control_command_.arm_control_references.position_ref[4]*(-1);
		command_to_send_[5].position = last_control_command_.arm_control_references.position_ref[5]- 1.5707;
		command_to_send_[6].position = last_control_command_.arm_control_references.position_ref[6];
	}

	updateAALState();
}

void ArmAbstractionLayerGazebo::sendJointControl()
{
	for(int i=0;i<7;i++)
	{
		joint_gazebo_pub_.publish(command_to_send_[i]);
	}
}

void ArmAbstractionLayerGazebo::matlab_step()
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
