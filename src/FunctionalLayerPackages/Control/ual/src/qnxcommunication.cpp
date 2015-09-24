#include <ual/qnxcommunication.h>

QnxCommunication::QnxCommunication(ros::NodeHandle *n,  std::string host, int commandPort, int statePort):
   _receive_task(*n,statePort),
   _qnx_sender(host.c_str(),commandPort)

{
   _send_loop_timer_ = n->createTimer(ros::Duration(1/SEND_RATE),
               boost::bind(&QnxCommunication::sendLoop, this, _1));

   _qnx_sender.connect();
   _control_ref_to_qnx.quad_control_references.heartBeat.uiHeartBeat = 0;

   _udp_receiver_thread = new boost::thread(_receive_task);
}

arcas_msgs::QuadStateEstimationWithCovarianceStamped QnxCommunication::getQuadStateEstimation()
{
   //std::cerr << "Qnx Communication: FlyingState: " << (unsigned short)_receive_task.getLastQuadState().quad_state_estimation_with_covariance.flying_state << std::endl;
   return _receive_task.getLastQuadState();
}

void QnxCommunication::setQuadControlReferences(arcas_msgs::QuadControlReferences ctrl_ref, unsigned char takeoff, unsigned char land)
{

   _control_ref_to_qnx.type = QuadControlReferences;

   _control_ref_to_qnx.quad_control_references.posRef.dPosRefLat = ctrl_ref.position_ref.x;
   _control_ref_to_qnx.quad_control_references.posRef.dPosRefLon = ctrl_ref.position_ref.y;
   _control_ref_to_qnx.quad_control_references.posRef.dPosRefH = ctrl_ref.position_ref.z;
   _control_ref_to_qnx.quad_control_references.posRef.dTypePosRef = 0;

   _control_ref_to_qnx.quad_control_references.velControlRef.dVelControlRef = ctrl_ref.velocity_ref;
   _control_ref_to_qnx.quad_control_references.headingControlRef.dHeadingControlRef = ctrl_ref.heading;

   _control_ref_to_qnx.quad_control_references.uiTakeOff = takeoff;
   _control_ref_to_qnx.quad_control_references.uiLand = land;

   _control_ref_to_qnx.quad_control_references.heartBeat.uiHeartBeat++;

}

void QnxCommunication::sendLoop(const ros::TimerEvent& te)
{
//	std::cerr << "Qnx Communication: Sending uiHearBeat" << std::endl;
//	_control_ref_to_qnx.quad_control_references.heartBeat.uiHeartBeat++;
    //std::cerr << "Size IControlRefArcas"<<sizeof(IControlRefArcas) << std::endl;
   if(!_qnx_sender.send(&_control_ref_to_qnx,sizeof(ArcasUDPControlReferences)))
   {
      std::cerr << "Qnx Communication: Cannot send control references to UAV" << std::endl;
   }
}
