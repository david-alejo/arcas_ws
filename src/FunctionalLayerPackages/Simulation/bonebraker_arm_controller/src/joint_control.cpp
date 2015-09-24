/*************************************************************************
 *
 * FADA-CATEC
 * __________________
 *
 *  [2013] FADA-CATEC
 *  All Rights Reserved.
 *
 * NOTICE:  All information contained herein is, and remains
 * the property of FADA-CATEC and its suppliers,
 * if any.  The intellectual and technical concepts contained
 * herein are proprietary to FADA-CATEC
 * and its suppliers and may be covered by Europe and Foreign Patents,
 * patents in process, and are protected by trade secret or copyright law.
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from FADA-CATEC.
 *
 * Created on: 20-Mar-2013
 * Engineer: Jonathan Ruiz Páez
 * Email: jruiz@catec.aero
 */

#include "bonebraker_arm_controller/joint_control.h"

namespace BoneBraker
{

JointControl::JointControl(physics::JointPtr j, common::Time currTime)
{
   this->maxVelocity = 0.2;
   this->velocity = 0.2;
   this->maxEffort = 0;
   this->finalPosition = 0;
   this->prevUpdateTime_ = currTime;
   this->actualReference = 0;
   this->joint = j;

   this->Reset();
}
JointControl::~JointControl()
{

}

physics::JointPtr JointControl::GetJoint()
{
   return this->joint;
}

double JointControl::GetFinalPosition()
{
   return this->finalPosition;
}
void JointControl::SetFinalPosition(double f)
{
   this->finalPosition = f;
}

double JointControl::GetMaxEffort()
{
   return this->maxEffort;
}
void JointControl::SetMaxEffort(double e)
{
   this->maxEffort = e;
   this->pid.SetCmdMax(e);
   this->pid.SetCmdMin(e*(-1));
}

double JointControl::GetVelocity()
{
   return this->velocity;
}
void JointControl::SetVelocity(double v)
{
   if(this->velocity < this->maxVelocity)
      this->velocity = v;
   else
      this->velocity = this->maxVelocity;
}

double JointControl::GetMaxVelocity()
{
   return this->maxVelocity;
}
void JointControl::SetMaxVelocity(double v)
{
   this->maxVelocity = v;
}

void JointControl::SetPIDGains(double Kp, double Ki, double Kd, double maxI, double minI)
{
   this->pid.SetPGain(Kp);
   this->pid.SetIGain(Ki);
   this->pid.SetDGain(Kd);
   this->pid.SetIMin(minI);
   this->pid.SetIMax(maxI);
}
void JointControl::Update(common::Time currTime)
{
   common::Time stepTime = currTime - this->prevUpdateTime_;
   this->prevUpdateTime_ = currTime;

   //Get actual position.
   double pos_curr = this->joint->GetAngle(0).Radian();

   //If I have to rotate.
   if(this->actualReference != this->finalPosition)
   {
      //In wich direction have to rotate.
      double direction = -1;
      if(this->finalPosition > this->actualReference)
         direction = 1;

      //Compute next PID reference.
      this->actualReference += direction * this->velocity * (stepTime.nsec * 1e-9);

      //If final position has been exceeded
      if((this->finalPosition - this->actualReference)* direction <0)
      {
         this->actualReference = this->finalPosition;
      }

   }

   double pos_err = pos_curr - this->actualReference;

   this->joint->SetForce(0,this->pid.Update(pos_err, stepTime));
}
void JointControl::Reset()
{
   this->maxVelocity = 0.5;
   this->velocity = 0.3;
   this->maxEffort = 0.15;
   this->finalPosition = 0;
   this->actualReference = 0;
   this->pid.SetCmdMin(this->maxEffort * (-1));
   this->pid.SetCmdMax(this->maxEffort);
   this->SetPIDGains(100, 100, 1,0.1,-0.1);
}

}
