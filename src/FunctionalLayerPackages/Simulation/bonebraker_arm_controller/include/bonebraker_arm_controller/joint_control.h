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

#ifndef JOINT_CONTROL_H_
#define JOINT_CONTROL_H_

#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/PID.hh"
#include "gazebo/physics/physics.hh"

using namespace gazebo;

namespace BoneBraker
{
    class JointControl
    {
    public:

      JointControl(physics::JointPtr j, common::Time currTime);
      ~JointControl();

      physics::JointPtr GetJoint();

      double GetFinalPosition();
      void SetFinalPosition(double f);

      double GetMaxEffort();
      void SetMaxEffort(double e);

      double GetVelocity();
      void SetVelocity(double v);

      double GetMaxVelocity();
      void SetMaxVelocity(double v);

      void SetPIDGains(double Kp, double Ki, double Kd, double maxI, double minI);

      void Update(common::Time currTime);
      void Reset();


    private:
      common::Time prevUpdateTime_;
      physics::JointPtr joint;
      common::PID pid;
      double finalPosition;
      double actualReference;
      double velocity;
      double maxVelocity;
      double maxEffort;
    };

}


#endif /* JOINT_CONTROL_H_ */
