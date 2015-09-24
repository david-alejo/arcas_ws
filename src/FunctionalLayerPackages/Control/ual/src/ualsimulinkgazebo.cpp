#include <ual/ualsimulinkgazebo.h>

UALSimulinkGazebo::UALSimulinkGazebo(ros::NodeHandle *n, int uavId):
   generalCommunications(n,uavId),
   gazeboCommunications(n),
   actual_state(LANDED),
   landActionServer(std::string("land"),std::string("1")),
   takeoffActionServer(std::string("take_off"),std::string("1"))
{
   update_timer_ = n->createTimer(ros::Duration(1/UPDATE_RATE),
               boost::bind(&UALSimulinkGazebo::updateLoop, this, _1));


   //Minimmum time for takeoff/land
   takeoff_land_minimmum_time_.fromSec(4);

   lastUpdatePID = ros::Time::now();


   //Initialize PID
   pid_x.Init(5,0,2,0.0,0.0,2,-2);
   pid_y.Init(5,0,2,0.0,0.0,2,-2);
   pid_z.Init(1.0,0,0.0,0.0,0.0,2,-2);
   pid_yaw.Init(2.5,0.0,0.0,0.0,0.0,2.0,-2.0);
}

void UALSimulinkGazebo::updateLoop(const ros::TimerEvent& te)
{

   QuadStateEstimationWithCovarianceStamped last_quad_state =
         gazeboCommunications.getQuadStateEstimation();

   last_quad_state.quad_state_estimation_with_covariance.flying_state = actual_state;
   generalCommunications.setQuadStateEstimation(last_quad_state);

   landActionServer.uavUpdateState(actual_state);
   takeoffActionServer.uavUpdateState(actual_state);

   static arcas_msgs::QuadControlReferences reference_command;
   reference_command = generalCommunications.getQuadControlReferences().quad_control_references;

   tf::Vector3 wanted_position(0,0,0);

   if(actual_state==TAKING_OFF)
   {
      if((ros::Time::now() - takeoff_land_time_callback_) > takeoff_land_minimmum_time_
         && gazeboCommunications.getQuadStateEstimation().quad_state_estimation_with_covariance.position.z >= (TAKE_OFF_Z-0.05))
      {
         actual_state = FLYING;
      }
      else
      {
         wanted_position.setX(gazeboCommunications.getQuadStateEstimation().quad_state_estimation_with_covariance.position.x);
         wanted_position.setY(gazeboCommunications.getQuadStateEstimation().quad_state_estimation_with_covariance.position.y);
         wanted_position.setZ(TAKE_OFF_Z);
         reference_command.velocity_ref = 0.5;

      }
   }
   else if(actual_state==LANDING)
   {
      if((ros::Time::now() - takeoff_land_time_callback_) > takeoff_land_minimmum_time_
         && gazeboCommunications.getQuadStateEstimation().quad_state_estimation_with_covariance.position.z >= (LAND_Z+0.05))
      {
         actual_state = LANDED;
      }
      else
      {
         wanted_position.setX(gazeboCommunications.getQuadStateEstimation().quad_state_estimation_with_covariance.position.x);
         wanted_position.setY(gazeboCommunications.getQuadStateEstimation().quad_state_estimation_with_covariance.position.y);
         wanted_position.setZ(LAND_Z);
         reference_command.velocity_ref = 0.5;
      }
   }else
   {
      wanted_position.setX(generalCommunications.getQuadControlReferences().quad_control_references.position_ref.x);
      wanted_position.setY(generalCommunications.getQuadControlReferences().quad_control_references.position_ref.y);
      wanted_position.setZ(generalCommunications.getQuadControlReferences().quad_control_references.position_ref.z);
   }

   reference_command.position_ref.x = wanted_position.getX();
   reference_command.position_ref.y = wanted_position.getY();
   reference_command.position_ref.z = wanted_position.getZ();

   switch(actual_state)
   {
      case LANDED:

         if(takeoffActionServer.hasReceivedTakeoffAction())
         {
            actual_state = TAKING_OFF;
         }else
         {
            actual_state = LANDED;
         }
         break;
      case TAKING_OFF:
      case LANDING:
      case FLYING:
         if(actual_state==FLYING && landActionServer.hasReceivedLandAction())
         {
            actual_state = LANDING;
         }else
         {
            actual_state = actual_state;
         }
         break;
      default:
            actual_state = actual_state;
            break;
   }

   //Update command to send.
   gazeboCommunications.setQuadControlReferences(reference_command);
}
