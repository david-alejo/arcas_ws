/*
 * RVOSimulator.cpp
 * RVO2-3D Library
 *
 * Copyright (c) 2008-2011 University of North Carolina at Chapel Hill.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for educational, research, and non-profit purposes, without
 * fee, and without a written agreement is hereby granted, provided that the
 * above copyright notice, this paragraph, and the following four paragraphs
 * appear in all copies.
 *
 * Permission to incorporate this software into commercial products may be
 * obtained by contacting the authors <geom@cs.unc.edu> or the Office of
 * Technology Development at the University of North Carolina at Chapel Hill
 * <otd@unc.edu>.
 *
 * This software program and documentation are copyrighted by the University of
 * North Carolina at Chapel Hill. The software program and documentation are
 * supplied "as is," without any accompanying services from the University of
 * North Carolina at Chapel Hill or the authors. The University of North
 * Carolina at Chapel Hill and the authors do not warrant that the operation of
 * the program will be uninterrupted or error-free. The end-user understands
 * that the program was developed for research purposes and is advised not to
 * rely exclusively on the program for any reason.
 *
 * IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE
 * AUTHORS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS
 * SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT
 * CHAPEL HILL OR THE AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY
 * DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY
 * STATUTORY WARRANTY OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON
 * AN "AS IS" BASIS, AND THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE
 * AUTHORS HAVE NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES,
 * ENHANCEMENTS, OR MODIFICATIONS.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

#include "RVOSimulator.h"

#ifdef _OPENMP
#include <omp.h>
#endif

#include "Agent.h"
#include "KdTree.h"
#include <iostream>
#include "Convexer.h"
#include <sstream>

using namespace std;

namespace RVO_UNSTABLE {
	string Plane::toString() const {
	  ostringstream os;
	  
	  os << "Normal = " << normal.x() << ", " << normal.y() << ", " << normal.z() << "\t";
	  os << "Point = " << point.x() << ", " << point.y() << ", " << point.z() << "\n";
	  
	  return os.str();
	}
  
	RVOSimulator::RVOSimulator() : defaultAgent_(NULL), kdTree_(NULL), globalTime_(0.0f), timeStep_(0.0f)
	{
		kdTree_ = new KdTree(this);
	}

	RVOSimulator::RVOSimulator(float timeStep, float neighborDist, size_t maxNeighbors, float timeHorizon, float radius, 
				   float radius_obstacle, float maxSpeed, float timeObstacle, const Vector3 &velocity, float pure_delay, float exponent
	) : defaultAgent_(NULL), kdTree_(NULL), globalTime_(0.0f), timeStep_(timeStep), pure_delay(pure_delay)
	{
		kdTree_ = new KdTree(this);
		defaultAgent_ = new Agent(this);

		defaultAgent_->maxNeighbors_ = maxNeighbors;
		defaultAgent_->maxSpeed_ = maxSpeed;
		defaultAgent_->neighborDist_ = neighborDist;
		defaultAgent_->radius_ = radius;
		defaultAgent_->timeHorizon_ = timeHorizon;
		defaultAgent_->velocity_ = velocity;
		defaultAgent_->radius_z = radius;
		defaultAgent_->radius_obstacle_ = radius_obstacle;
		defaultAgent_->radius_obstacle_z = radius_obstacle;
		defaultAgent_->exponent = exponent;
		defaultAgent_->exponent_z = 1.0;
		
		if (timeObstacle > 0.0) {
		  defaultAgent_->timeObstacle_ = timeObstacle;
// 		  std::cout << "Time Obstacle = " <<  timeObstacle << std::endl;
		} else {
		  defaultAgent_->timeObstacle_ = timeHorizon;
		}
	}

	RVOSimulator::~RVOSimulator()
	{
		if (defaultAgent_ != NULL) {
			delete defaultAgent_;
		}

		for (size_t i = 0; i < agents_.size(); ++i) {
			delete agents_[i];
		}

		if (kdTree_ != NULL) {
			delete kdTree_;
		}
		
		// Static obstacles stuff
// 		delete scene;
		for (unsigned int i = 0; i < models.size(); i++) {
		  delete models.at(i);
		}
		models.clear();
	}

	size_t RVOSimulator::getAgentNumAgentNeighbors(size_t agentNo) const
	{
		return agents_[agentNo]->agentNeighbors_.size();
	}

	size_t RVOSimulator::getAgentAgentNeighbor(size_t agentNo, size_t neighborNo) const
	{
		return agents_[agentNo]->agentNeighbors_[neighborNo].second->id_;
	}

	size_t RVOSimulator::getAgentNumORCAPlanes(size_t agentNo) const
	{
		return agents_[agentNo]->orcaPlanes_.size();
	}

	const Plane &RVOSimulator::getAgentORCAPlane(size_t agentNo, size_t planeNo) const
	{
		return agents_[agentNo]->orcaPlanes_[planeNo];
	}

	void RVOSimulator::removeAgent(size_t agentNo)
	{
		delete agents_[agentNo];
		agents_[agentNo] = agents_.back();
		agents_.pop_back();
	}

	size_t RVOSimulator::addAgent(const Vector3 &position)
	{
		if (defaultAgent_ == NULL) {
			return RVO_ERROR;
		}

		Agent *agent = new Agent(this);

		agent->position_ = position;
		agent->maxNeighbors_ = defaultAgent_->maxNeighbors_;
		agent->maxSpeed_ = defaultAgent_->maxSpeed_;
		agent->neighborDist_ = defaultAgent_->neighborDist_;
		agent->radius_ = defaultAgent_->radius_;
		agent->timeHorizon_ = defaultAgent_->timeHorizon_;
		agent->velocity_ = defaultAgent_->velocity_;
		agent->timeObstacle_ = defaultAgent_->timeObstacle_;
		agent->radius_z = defaultAgent_->radius_z;
		agent->radius_obstacle_ = defaultAgent_->radius_obstacle_;
		agent->radius_obstacle_z = defaultAgent_->radius_obstacle_z;
		agent->obstacleDist_ = defaultAgent_->obstacleDist_;
		agent->radius_warning = defaultAgent_->radius_warning;
		agent->collision_multiplier = defaultAgent_->collision_multiplier;
		agent->id_ = agents_.size();
		agent->frozen_mult = defaultAgent_->frozen_mult;
		agent->z_multiplier = defaultAgent_->z_multiplier;

		agents_.push_back(agent);

		return agents_.size() - 1;
	}

	size_t RVOSimulator::addAgent(const Vector3 &position, float neighborDist, size_t maxNeighbors, float timeHorizon, float radius, float maxSpeed, float timeObstacle, const Vector3 &velocity)
	{
		Agent *agent = new Agent(this);

		agent->position_ = position;
		agent->maxNeighbors_ = maxNeighbors;
		agent->maxSpeed_ = maxSpeed;
		agent->neighborDist_ = neighborDist;
		agent->radius_ = radius;
		agent->radius_ = radius;
		agent->timeHorizon_ = timeHorizon;
		agent->velocity_ = velocity;
		agent->timeObstacle_ = timeObstacle;
		agent->id_ = agents_.size();
		agent->radius_obstacle_ = radius;
		agent->radius_obstacle_z = radius;
		agents_.push_back(agent);
		return agents_.size() - 1;
	}

	void RVOSimulator::doStep()
	{
		kdTree_->buildAgentTree();
		
		for (unsigned int i = 0; i < agents_.size(); i++) {
		  if (pure_delay > 0.0) {
		    doPureForward(propagate_commands);
		  }
		}
		
		for (unsigned int i = 0; i < agents_.size(); i++) {
		  agents_[i]->computeNeighbors();
		  agents_[i]->computeNewVelocity(models);
		  agents_[i]->update();
		}
		
		globalTime_ += timeStep_;
	}
	
	void RVOSimulator::doStep(unsigned int i)
	{
		kdTree_->buildAgentTree();
		
		if (pure_delay > 0.0) {
		  doPureForward();
		}

		agents_[i]->computeNeighbors();
		agents_[i]->computeNewVelocity(models);
		agents_[i]->update();
		
		globalTime_ += timeStep_;
	}


	size_t RVOSimulator::getAgentMaxNeighbors(size_t agentNo) const
	{
		return agents_[agentNo]->maxNeighbors_;
	}

	float RVOSimulator::getAgentMaxSpeed(size_t agentNo) const
	{
		return agents_[agentNo]->maxSpeed_;
	}

	float RVOSimulator::getAgentNeighborDist(size_t agentNo) const
	{
		return agents_[agentNo]->neighborDist_;
	}

	const Vector3 &RVOSimulator::getAgentPosition(size_t agentNo) const
	{
		return agents_[agentNo]->position_;
	}

	const Vector3 &RVOSimulator::getAgentPrefVelocity(size_t agentNo) const
	{
		return agents_[agentNo]->prefVelocity_;
	}

	float RVOSimulator::getAgentRadius(size_t agentNo) const
	{
		return agents_[agentNo]->radius_;
	}

	float RVOSimulator::getAgentTimeHorizon(size_t agentNo) const
	{
		return agents_[agentNo]->timeHorizon_;
	}

	const Vector3 &RVOSimulator::getAgentVelocity(size_t agentNo) const
	{
		return agents_[agentNo]->velocity_;
	}

	float RVOSimulator::getGlobalTime() const
	{
		return globalTime_;
	}

	size_t RVOSimulator::getNumAgents() const
	{
		return agents_.size();
	}

	float RVOSimulator::getTimeStep() const
	{
		return timeStep_;
	}

	void RVOSimulator::setAgentDefaults(float neighborDist, size_t maxNeighbors, float timeHorizon, float radius, float maxSpeed, float timeObstacle, const Vector3 &velocity)
	{
		if (defaultAgent_ == NULL) {
			defaultAgent_ = new Agent(this);
		}

		defaultAgent_->maxNeighbors_ = maxNeighbors;
		defaultAgent_->maxSpeed_ = maxSpeed;
		defaultAgent_->neighborDist_ = neighborDist;
		defaultAgent_->radius_ = radius;
		defaultAgent_->timeHorizon_ = timeHorizon;
		defaultAgent_->timeObstacle_ = timeObstacle;
		defaultAgent_->velocity_ = velocity;
	}

	void RVOSimulator::setAgentMaxNeighbors(size_t agentNo, size_t maxNeighbors)
	{
		agents_[agentNo]->maxNeighbors_ = maxNeighbors;
	}

	void RVOSimulator::setAgentMaxSpeed(size_t agentNo, float maxSpeed)
	{
		agents_[agentNo]->maxSpeed_ = maxSpeed;
	}

	void RVOSimulator::setAgentNeighborDist(size_t agentNo, float neighborDist)
	{
		agents_[agentNo]->neighborDist_ = neighborDist;
	}

	void RVOSimulator::setAgentPosition(size_t agentNo, const Vector3 &position)
	{
		agents_[agentNo]->position_ = position;
	}

	void RVOSimulator::setAgentPrefVelocity(size_t agentNo, const Vector3 &prefVelocity)
	{
		agents_[agentNo]->prefVelocity_ = prefVelocity;
	}

	void RVOSimulator::setAgentRadius(size_t agentNo, float radius)
	{
		agents_[agentNo]->radius_ = radius;
	}
	
	void RVOSimulator::setAgentRadiusZ(size_t agentNo, float radius)
	{
		agents_[agentNo]->radius_z = radius;
	}
	
	void RVOSimulator::setAgentDefaultRadiusZ(float radius)
	{
		defaultAgent_->radius_z = radius;
	}
	
	void RVOSimulator::setAgentDefaultRadiusObstacleZ(float radius)
	{
		defaultAgent_->radius_obstacle_z = radius;
		std::cout << "Radius Obstacle z = " <<  radius << std::endl;
	}
	
	void RVOSimulator::setAgentDefaultObstacleDist(float dist)
	{
		defaultAgent_->obstacleDist_ = dist;
	}
	
	void RVOSimulator::setAgentDefaultWarningDist(float dist)
	{
		defaultAgent_->radius_warning = dist;
	}
	
	void RVOSimulator::setAgentDefaultAcceleration(float ac)
	{
		defaultAgent_->a_max = ac;
	}
	
	

	void RVOSimulator::setAgentTimeHorizon(size_t agentNo, float timeHorizon)
	{
		agents_[agentNo]->timeHorizon_ = timeHorizon;
	}
	
	void RVOSimulator::setAgentVelocity(size_t agentNo, const Vector3 &velocity)
	{
		agents_[agentNo]->velocity_ = velocity;
	}
	
	void RVOSimulator::setTimeStep(float timeStep)
	{
		timeStep_ = timeStep;
	}
	



float RVOSimulator::getObstacleDistance(unsigned int i) const {
  return agents_.at(i)->getObstacleDistance(models);
}

void RVOSimulator::setAgentDefaultCollisionMultiplier(float coll) {
  defaultAgent_->collision_multiplier = coll;
}


void RVOSimulator::setAgentDefaultZMultiplier(float coll) {
  defaultAgent_->z_multiplier = coll;
}

void RVOSimulator::setAgentDefaultFrozen(float coll) {
  defaultAgent_->frozen_mult = coll;
}

void RVOSimulator::setAgentDefaultExponent(float exponent, float exponent_z)
{
  defaultAgent_->exponent = exponent;
  defaultAgent_->exponent_z = exponent_z;
}


bool RVOSimulator::loadScenario(const std::string& file, bool make_convex) {
  models = Convexer::loadScenario(file, make_convex);
  return models.size() > 0;
}

void RVOSimulator::doPureForward(bool propagate_commands)
{
  if (agents_.size() > 0) {
    int steps = pure_delay / timeStep_;
    for (unsigned int i = 0; i < agents_.size();i++) {
      agents_[i]->newVelocity_ = agents_[i]->velocity_;
      std::list<Vector3>::iterator it = agents_[i]->velocity_history.begin();
      for (int j = 0; j < steps; j++, it++) {
	if (agents_[i]->velocity_history.size() > 0 && propagate_commands) {
	  if (it == agents_[i]->velocity_history.end()) {
	    it--;
	  }
	  agents_[i]->newVelocity_ = *it;
	}
	agents_[i]->update();
      }
    }
  }
}

void RVOSimulator::setLastCommand(unsigned int uav_id, RVO_UNSTABLE::Vector3 command)
{
  if (pure_delay > 0.0 && propagate_commands) {
    unsigned int n_steps = pure_delay / timeStep_;
    agents_[uav_id]->velocity_history.push_back(command);
    if (agents_[uav_id]->velocity_history.size() > n_steps) {
      agents_[uav_id]->velocity_history.erase(agents_[uav_id]->velocity_history.begin());
    }
  }
}

vector< PQP_DistanceResult > RVOSimulator::getClosestPoints(unsigned int i) const
{
  return agents_[i]->getClosestPoints(models);
}




}
