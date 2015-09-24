#include <ual/ualqnx.h>

// NOTE: Jhonny te hemos quitado el hardcode "1" de forma chapucera, si no te gusta lo cambias
std::string int2String( int toConvert){
  char res[10];
  sprintf(res,"%d",toConvert);
  return std::string(res);
}

UALQnx::UALQnx(ros::NodeHandle *n, int uavId, std::string qnx_host, unsigned int statePort, unsigned int commandPort):
   generalCommunications(n,uavId),
   qnxCommunications(n, qnx_host, commandPort, statePort),
   actual_state(LANDED),
   landActionServer(std::string("land"),int2String(uavId)),
   takeoffActionServer(std::string("take_off"),int2String(uavId))
{
   update_timer_ = n->createTimer(ros::Duration(1/UPDATE_RATE),
               boost::bind(&UALQnx::updateLoop, this, _1));
}

void UALQnx::updateLoop(const ros::TimerEvent& te)
{


   QuadStateEstimationWithCovarianceStamped last_quad_state =
         qnxCommunications.getQuadStateEstimation();

    //Offset due to changed real testbed center
    last_quad_state.quad_state_estimation_with_covariance.position.x += 0.44;
    last_quad_state.quad_state_estimation_with_covariance.position.y += 0.345;
   generalCommunications.setQuadStateEstimation(last_quad_state);

   actual_state = (StateUAV)last_quad_state.quad_state_estimation_with_covariance.flying_state;

   landActionServer.uavUpdateState(actual_state);
   takeoffActionServer.uavUpdateState(actual_state);

   static arcas_msgs::QuadControlReferences reference_command;
   reference_command =
         generalCommunications.getQuadControlReferences().quad_control_references;

    //Offset due to changed testbed center
    reference_command.position_ref.x -= 0.44;
    reference_command.position_ref.y -= 0.345;

   //Update command to send.
   qnxCommunications.setQuadControlReferences(reference_command,
               takeoffActionServer.hasReceivedTakeoffAction(),
               landActionServer.hasReceivedLandAction());
}
