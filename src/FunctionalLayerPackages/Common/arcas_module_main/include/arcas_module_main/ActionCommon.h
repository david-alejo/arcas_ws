/*!
 * \file
 * \brief Base class for actions
 */
#ifndef ACTIONCOMMON_H
#define ACTIONCOMMON_H
#include <ros/ros.h>
#include<actionlib/server/action_server.h>

template <class RosAction, class UpdateType>
class ActionCommon
{
public:
  ActionCommon()
  {
  }
  ~ActionCommon()
  {

  }

  /*!
   * \brief Return if action has an active goal.
   * \return true if has an active goal
   */
  virtual bool getHasGoal()=0;

  /*!
   * \brief FinishAction in wanted state
   * \param true if action has done correctyle, false if not
   */
  virtual void finishAction(bool SUCCEDED=true)=0;

  /*!
   * \brief get currect goal message
   * \return receive message in goal callback.
   */
  virtual typename RosAction::_action_goal_type::_goal_type getGoal()=0;

  /*!
   * \brief If the action need to be updated, you have to use this method
   * \param Generic Type, TBD
   */
  virtual bool updateData(UpdateType data)=0;

  /*!
   * \brief Callback for new goals
   * \param goal handle of the new petitions.
   */
  virtual void goalCB(actionlib::ServerGoalHandle<RosAction> goal_handle)=0;

  /*!
   * \brief Callback for cancel request
   * \param goal handle of the current task.
   */
  virtual void preemptCB(actionlib::ServerGoalHandle<RosAction> goal_handle)=0;
};

#endif // ACTIONCOMMON_H
