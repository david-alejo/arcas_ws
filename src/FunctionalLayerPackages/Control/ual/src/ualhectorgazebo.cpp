#include <ual/ualhectorgazebo.h>
#include <stdlib.h>

std::string int2String( int toConvert){
   char res[10];
   sprintf(res,"%d",toConvert);
   return std::string(res);
}

UALHectorGazebo::UALHectorGazebo(ros::NodeHandle *n, int uavId):
   generalCommunications(n,uavId),
   gazeboCommunications(n),
   actual_state(LANDED),
   landActionServer(std::string("land"), int2String(uavId)),
   takeoffActionServer(std::string("take_off"), int2String(uavId))
{
   update_timer_ = n->createTimer(ros::Duration(1/UPDATE_RATE),
                                  boost::bind(&UALHectorGazebo::updateLoop, this, _1));


   //Minimmum time for takeoff/land
   takeoff_land_minimmum_time_.fromSec(4);

   lastUpdatePID = ros::Time::now();


   //Initialize PID
   pid_x.Init(5,0,2,0.0,0.0,2,-2);
   pid_y.Init(5,0,2,0.0,0.0,2,-2);
   pid_z.Init(1.0,0,0.0,0.0,0.0,2,-2);
   pid_yaw.Init(2.5,0.0,0.0,0.0,0.0,2.0,-2.0);
}

void UALHectorGazebo::updateLoop(const ros::TimerEvent& te)
{


   QuadStateEstimationWithCovarianceStamped last_quad_state =
         gazeboCommunications.getQuadStateEstimation();

   last_quad_state.quad_state_estimation_with_covariance.flying_state = actual_state;

   if (buffer_.size() < PURE_DELAY * UPDATE_RATE) {
      buffer_.push_back(last_quad_state);
   } else {
      buffer_.push_back(last_quad_state);
      buffer_.erase(buffer_.begin());
   }
   if (buffer_.begin() == buffer_.end() || PURE_DELAY <= 0.0) {
      generalCommunications.setQuadStateEstimation(last_quad_state);
   } else {
      generalCommunications.setQuadStateEstimation(*buffer_.begin());
   }

   landActionServer.uavUpdateState(actual_state);
   takeoffActionServer.uavUpdateState(actual_state);

   geometry_msgs::Twist velocityCommand;
   tf::Vector3 wanted_position(0,0,0);
   tf::Vector3 rotated_position(0,0,0);

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

      }
   }
   else if(actual_state==LANDING)
   {

      if((ros::Time::now() - takeoff_land_time_callback_) > takeoff_land_minimmum_time_
            && gazeboCommunications.getQuadStateEstimation().quad_state_estimation_with_covariance.position.z <= (LAND_Z+0.05))
      {
         actual_state = LANDED;
      }
      else
      {
         wanted_position.setX(gazeboCommunications.getQuadStateEstimation().quad_state_estimation_with_covariance.position.x);
         wanted_position.setY(gazeboCommunications.getQuadStateEstimation().quad_state_estimation_with_covariance.position.y);
         wanted_position.setZ(LAND_Z);
      }
   }else
   {
      wanted_position.setX(generalCommunications.getQuadControlReferences().quad_control_references.position_ref.x);
      wanted_position.setY(generalCommunications.getQuadControlReferences().quad_control_references.position_ref.y);
      wanted_position.setZ(generalCommunications.getQuadControlReferences().quad_control_references.position_ref.z);
   }


   double headingPIDError=0;

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

      velocityCommand.linear.x = 0;
      velocityCommand.linear.y = 0;
      velocityCommand.linear.z = -0.1;

      break;
   case TAKING_OFF:
   case LANDING:
   case FLYING:
      if(actual_state==FLYING && landActionServer.hasReceivedLandAction())
      {
         //std::cerr << "Request to land" << std::endl;
         actual_state = LANDING;
      }else
      {
         actual_state = actual_state;
      }

      pid_x.SetCmd(generalCommunications.getQuadControlReferences().
                   quad_control_references.velocity_ref);
      pid_y.SetCmd(generalCommunications.getQuadControlReferences().
                   quad_control_references.velocity_ref);
      pid_z.SetCmd(generalCommunications.getQuadControlReferences().
                   quad_control_references.velocity_ref);


      //Rotate Position to pid control
      static tf::Vector3 tf_pos;
      tf_pos.setX(gazeboCommunications.getQuadStateEstimation().
                  quad_state_estimation_with_covariance.position.x);
      tf_pos.setY(gazeboCommunications.getQuadStateEstimation().
                  quad_state_estimation_with_covariance.position.y);
      tf_pos.setZ(gazeboCommunications.getQuadStateEstimation().
                  quad_state_estimation_with_covariance.position.z);

      static tf::Quaternion rotation;
      rotation.setRPY(gazeboCommunications.getQuadStateEstimation().
                      quad_state_estimation_with_covariance.attitude.roll,
                      gazeboCommunications.getQuadStateEstimation().
                      quad_state_estimation_with_covariance.attitude.pitch,
                      gazeboCommunications.getQuadStateEstimation().
                      quad_state_estimation_with_covariance.attitude.yaw);

      static tf::Transform transformMatrix;
      transformMatrix.setOrigin(tf_pos);
      transformMatrix.setRotation(rotation);

      //Final rotated position (Command)
      rotated_position = transformMatrix.inverse() * wanted_position;


      //Calculate elapsed time
      static ros::Duration rosTimeIncrement = ros::Time::now() - lastUpdatePID;
      static common::Time timeIncrement(rosTimeIncrement.sec,rosTimeIncrement.nsec);
      lastUpdatePID = ros::Time::now();

      velocityCommand.linear.x = pid_x.Update(rotated_position.getX()*(-1),
                                              timeIncrement);
      velocityCommand.linear.y = pid_y.Update(rotated_position.getY()*(-1),
                                              timeIncrement);
      velocityCommand.linear.z = pid_z.Update((wanted_position.getZ()-tf_pos.getZ())*(-1),
                                              timeIncrement);

      headingPIDError = (generalCommunications.getQuadControlReferences().quad_control_references.heading-
                         gazeboCommunications.getQuadStateEstimation().quad_state_estimation_with_covariance.attitude.yaw)*(-1);



      //if(headingPIDError>3.14159)
      //{
      //std::cerr << "Error en yaw (" << generalCommunications.getQuadControlReferences().quad_control_references.heading <<";" <<
      //		gazeboCommunications.getQuadStateEstimation().quad_state_estimation_with_covariance.attitude.yaw  << ") : " << headingPIDError << std::endl;
      //}

      while(headingPIDError>3.14159)
         headingPIDError-= 2*3.14159;

      while(headingPIDError<-3.14159)
         headingPIDError += 2*3.14159;


      velocityCommand.angular.z = pid_yaw.Update(
               headingPIDError,
               timeIncrement);


      /*wanted_position.setX(5.0);
         wanted_position.setY(0.0);
         wanted_position.setZ(0.0);
         rotated_position = transformMatrix.inverse() * wanted_position;*/

      //	std::cerr << "error: " << velocityCommand.linear.x << " - " << velocityCommand.linear.y << ";" << std::endl;
      //			  tf_pos.getZ()<< std::endl;
      break;
   default:
      actual_state = actual_state;
      break;
   }

   //Limit velocity
   tf::Vector3 actualVel(velocityCommand.linear.x,
                         velocityCommand.linear.y,
                         velocityCommand.linear.z);
   if(generalCommunications.getQuadControlReferences().quad_control_references.velocity_ref>0 &&
         actualVel.length()>generalCommunications.getQuadControlReferences().quad_control_references.velocity_ref)
   {
      actualVel =
            (actualVel/actualVel.length())*generalCommunications.getQuadControlReferences().quad_control_references.velocity_ref;
   }

   velocityCommand.linear.x = actualVel.getX();
   velocityCommand.linear.y = actualVel.getY();
   velocityCommand.linear.z = actualVel.getZ();

   //Update command to send.
   gazeboCommunications.setTwist(velocityCommand);
}
