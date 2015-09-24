#include "aal/arm_abstraction_layer_qnx.h"

ArmAbstractionLayerQNX::ArmAbstractionLayerQNX(ros::NodeHandle n,string host, int port):
	ArmAbstractionLayer(n),
	client(host.c_str(),port),
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
	//	command_to_send_[0].joint_name = joint_names_[0];
	//	command_to_send_[1].joint_name = joint_names_[1];
	//	command_to_send_[2].joint_name = joint_names_[2];
	//	command_to_send_[3].joint_name = joint_names_[3];
	//	command_to_send_[4].joint_name = joint_names_[4];
	//	command_to_send_[5].joint_name = joint_names_[5];

	//	//Claw
	//	command_to_send_[6].joint_name = joint_names_[6];
	//	command_to_send_[7].joint_name = joint_names_[6];

	joint_states_sub_ = n.subscribe("/ual_1/qnx_udp_arm_state", 0,
									&ArmAbstractionLayerQNX::jointStatesCallback,
									this);


	data_to_send_to_qnx.type = ArmControlReferences;
	client.connect();
}

ArmAbstractionLayerQNX::~ArmAbstractionLayerQNX()
{
	joint_states_sub_.shutdown();
}


void ArmAbstractionLayerQNX::jointStatesCallback(
		const arcas_msgs::ArmStateEstimationStamped &js)
{
	unsigned char contracted=0, extended=0;

	this->arm_state_estimation_to_send_ = js;

	if(this->arm_state_estimation_to_send_.arm_state_estimation.arm_state == this->arm_state_estimation_to_send_.arm_state_estimation.CONTRACTED)
	{
		contracted = 1;
	}else if(this->arm_state_estimation_to_send_.arm_state_estimation.arm_state == this->arm_state_estimation_to_send_.arm_state_estimation.EXTENDED)
	{
		extended = 1;
	}

	action_object_.updateState(extended,contracted);
	action_object_.updateCommands(extended,contracted);

	if(contracted!=0)
	{
		this->arm_state_estimation_to_send_.arm_state_estimation.arm_state = this->arm_state_estimation_to_send_.arm_state_estimation.CONTRACTING;
	}else if(extended!=0)
	{
		this->arm_state_estimation_to_send_.arm_state_estimation.arm_state = this->arm_state_estimation_to_send_.arm_state_estimation.EXTENDING;
	}


}

void ArmAbstractionLayerQNX::receiveJointState(){
	action_object_.updateJointState(&last_joint_states_,joint_names_);
}

void ArmAbstractionLayerQNX::update()
{
	if(!action_object_.isExtended())
	{
		data_to_send_to_qnx.arm_control_references.armPosition.sJoint1 = 0;
		data_to_send_to_qnx.arm_control_references.armPosition.sJoint2 = -0.7854;
		data_to_send_to_qnx.arm_control_references.armPosition.sJoint3 = 1.5708;
		data_to_send_to_qnx.arm_control_references.armPosition.sJoint4 = 0;
		data_to_send_to_qnx.arm_control_references.armPosition.sJoint5 = -0.7854;
		data_to_send_to_qnx.arm_control_references.armPosition.sJoint6 = 0;
		data_to_send_to_qnx.arm_control_references.armPosition.sJoint7 = 0;

		data_to_send_to_qnx.arm_control_references.armVelocity.sJoint1 = 20;
		data_to_send_to_qnx.arm_control_references.armVelocity.sJoint2 = 20;
		data_to_send_to_qnx.arm_control_references.armVelocity.sJoint3 = 20;
		data_to_send_to_qnx.arm_control_references.armVelocity.sJoint4 = 20;
		data_to_send_to_qnx.arm_control_references.armVelocity.sJoint5 = 20;
		data_to_send_to_qnx.arm_control_references.armVelocity.sJoint6 = 20;
		data_to_send_to_qnx.arm_control_references.armVelocity.sJoint7 = 20;
	}
	else
	{
		//Modo angulos, copiar directamente.
		data_to_send_to_qnx.arm_control_references.armPosition.sJoint1 =
				last_control_command_.arm_control_references.position_ref[0];
		data_to_send_to_qnx.arm_control_references.armPosition.sJoint2 =
				last_control_command_.arm_control_references.position_ref[1];
		data_to_send_to_qnx.arm_control_references.armPosition.sJoint3 =
				last_control_command_.arm_control_references.position_ref[2];
		data_to_send_to_qnx.arm_control_references.armPosition.sJoint4 =
				last_control_command_.arm_control_references.position_ref[3];
		data_to_send_to_qnx.arm_control_references.armPosition.sJoint5 =
				last_control_command_.arm_control_references.position_ref[4];
		data_to_send_to_qnx.arm_control_references.armPosition.sJoint6 =
				last_control_command_.arm_control_references.position_ref[5];
		data_to_send_to_qnx.arm_control_references.armPosition.sJoint7 =
				last_control_command_.arm_control_references.position_ref[6];
		data_to_send_to_qnx.arm_control_references.armPosition.sJoint8 =
				last_control_command_.arm_control_references.battery;

		if(last_control_command_.arm_control_references.velocity_ref.size()>7)
		{
			data_to_send_to_qnx.arm_control_references.armVelocity.sJoint1 =
					last_control_command_.arm_control_references.velocity_ref[1];
			data_to_send_to_qnx.arm_control_references.armVelocity.sJoint2 =
					last_control_command_.arm_control_references.velocity_ref[2];
			data_to_send_to_qnx.arm_control_references.armVelocity.sJoint3 =
					last_control_command_.arm_control_references.velocity_ref[3];
			data_to_send_to_qnx.arm_control_references.armVelocity.sJoint4 =
					last_control_command_.arm_control_references.velocity_ref[4];
			data_to_send_to_qnx.arm_control_references.armVelocity.sJoint5 =
					last_control_command_.arm_control_references.velocity_ref[5];
			data_to_send_to_qnx.arm_control_references.armVelocity.sJoint6 =
					last_control_command_.arm_control_references.velocity_ref[6];
			data_to_send_to_qnx.arm_control_references.armVelocity.sJoint7 =
					last_control_command_.arm_control_references.velocity_ref[7];
		}


	}

	data_to_send_to_qnx.arm_control_references.heartBeat.uiHeartBeat++;

	action_object_.updateCommands(data_to_send_to_qnx.arm_control_references.uiArmExtensionFlag,
								  data_to_send_to_qnx.arm_control_references.uiArmRetractionFlag);
}

void ArmAbstractionLayerQNX::sendJointControl()
{
	//	for(int i=0;i<7;i++)
	//	{
	//		joint_gazebo_pub_.publish(command_to_send_[i]);
	//	}
	//Send to UDP server.
	client.send(&data_to_send_to_qnx,sizeof(ArcasUDPControlReferences));
}

