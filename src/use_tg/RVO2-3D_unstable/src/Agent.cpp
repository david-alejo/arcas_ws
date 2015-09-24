/*
 * Agent.cpp
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

#include "Agent.h"

#include <cmath>
#include <algorithm>
#include "Definitions.h"
#include "KdTree.h"
#include "functions/RealVector.h"
#include <functions/functions.h>
#include <boost/math/special_functions/sign.hpp>

static PQP_REAL unitary[3][3] = { {1, 0 ,0}, {0, 1, 0}, {0, 0 ,1} };
static PQP_REAL zeros[3] = {0, 0 ,0};

namespace RVO_UNSTABLE {
	/**
	 * \brief   A sufficiently small positive number.
	 */
	const float RVO_EPSILON = 0.00001f;

	/**
	 * \brief   Defines a directed line.
	 */
	class Line {
	public:
		/**
		 * \brief   The direction of the directed line.
		 */
		Vector3 direction;
		
		/**
		 * \brief   A point on the directed line.
		 */
		Vector3 point;
	};

	/**
	 * \brief   Solves a one-dimensional linear program on a specified line subject to linear constraints defined by planes and a spherical constraint.
	 * \param   planes        Planes defining the linear constraints.
	 * \param   planeNo       The plane on which the line lies.
	 * \param   line          The line on which the 1-d linear program is solved
	 * \param   radius        The radius of the spherical constraint.
	 * \param   optVelocity   The optimization velocity.
	 * \param   directionOpt  True if the direction should be optimized.
	 * \param   result        A reference to the result of the linear program.
	 * \return  True if successful.
	 */
	bool linearProgram1(const std::vector<Plane> &planes, size_t planeNo, const Line &line, float radius, const Vector3 &optVelocity, bool directionOpt, Vector3 &result);

	/**
	 * \brief   Solves a two-dimensional linear program on a specified plane subject to linear constraints defined by planes and a spherical constraint.
	 * \param   planes        Planes defining the linear constraints.
	 * \param   planeNo       The plane on which the 2-d linear program is solved
	 * \param   radius        The radius of the spherical constraint.
	 * \param   optVelocity   The optimization velocity.
	 * \param   directionOpt  True if the direction should be optimized.
	 * \param   result        A reference to the result of the linear program.
	 * \return  True if successful.
	 */
	bool linearProgram2(const std::vector<Plane> &planes, size_t planeNo, float radius, const Vector3 &optVelocity, bool directionOpt, Vector3 &result);

	/**
	 * \brief   Solves a three-dimensional linear program subject to linear constraints defined by planes and a spherical constraint.
	 * \param   planes        Planes defining the linear constraints.
	 * \param   radius        The radius of the spherical constraint.
	 * \param   optVelocity   The optimization velocity.
	 * \param   directionOpt  True if the direction should be optimized.
	 * \param   result        A reference to the result of the linear program.
	 * \return  The number of the plane it fails on, and the number of planes if successful.
	 */
	size_t linearProgram3(const std::vector<Plane> &planes, float radius, const Vector3 &optVelocity, bool directionOpt, Vector3 &result);

	/**
	 * \brief   Solves a four-dimensional linear program subject to linear constraints defined by planes and a spherical constraint.
	 * \param   planes     Planes defining the linear constraints.
	 * \param   beginPlane The plane on which the 3-d linear program failed.
	 * \param   radius     The radius of the spherical constraint.
	 * \param   result     A reference to the result of the linear program.
	 * \param   numObstPlanes The number of obstacle planes. They are located in the first part of the OrcaPlanes
	 */
	bool linearProgram4(const std::vector< RVO_UNSTABLE::Plane >& planes, size_t beginPlane, float radius, RVO_UNSTABLE::Vector3& result, size_t numObstPlanes);

	Agent::Agent(RVOSimulator *sim) : sim_(sim), id_(0), maxNeighbors_(0), maxSpeed_(0.0f), neighborDist_(0.0f), radius_(0.0f), timeHorizon_(0.0f), radius_z(-1.0f),
	radius_obstacle_(-1.0f), radius_obstacle_z(-1.0f), radius_warning(-1.0f), obstacleDist_(-1.0f), exponent(1.0f), exponent_z(1.0f){ }

	void Agent::computeNeighbors()
	{
		agentNeighbors_.clear();

		if (maxNeighbors_ > 0) {
			sim_->kdTree_->computeAgentNeighbors(this, neighborDist_ * neighborDist_);
		}
	}
	
	void Agent::computeNewVelocity() {
	  std::vector<PQP_Model *> obs;
	  computeNewVelocity(obs);
	}

	void Agent::computeNewVelocity(const std::vector< PQP_Model* >& obs)
	{
		float percentage_action = 0.5f;
		orcaPlanes_.clear();
		float invTimeHorizon = 1.0f / timeHorizon_;
		float invTimeObstacle = 1.0f / timeObstacle_;
		float neighborDist_ = this->neighborDist_;
		float obstacleDist_ = this->obstacleDist_;
		
		bool right_to_other = false;
		
		
		if (radius_obstacle_ < 0.0) {
		  radius_obstacle_ = radius_;
		}
		if (radius_obstacle_z < 0.0) {
		  radius_obstacle_z = radius_obstacle_;
		}
		if (obstacleDist_ < 0.0f) {
		  obstacleDist_ = neighborDist_;
		}
		if (radius_warning < radius_) {
		  radius_warning = radius_;
		}
		if (collision_multiplier < 5.0) {
		  collision_multiplier = 5.0;
		}
		float invTimeStep = invTimeObstacle * collision_multiplier; // Reaction when collision 5 times higher than usual
		
		// Add the constraints due to the static obstacles
		
		// First generate the model, a square of 10cm side
		PQP_Model uav;
		PQP_REAL p1[3] = {position_.x() + 0.05, position_.y() + 0.05, position_.z()};
		PQP_REAL p2[3] = {position_.x() + 0.05, position_.y() - 0.05, position_.z()};
		PQP_REAL p3[3] = {position_.x() - 0.05, position_.y() + 0.05, position_.z()};
		PQP_REAL p4[3] = {position_.x() - 0.05, position_.y() - 0.05, position_.z()};
		uav.AddTri(p1, p2, p3, 0);
		uav.AddTri(p4, p2, p3, 1);
		uav.EndModel();
		
		// Multiplier of the velocity: it would be equal to frozen if a collision is detected
		float mult = 1.0f;
		
		// Then compute the distance to the obstacles and its nearest triangle
		PQP_DistanceResult res;
		for (unsigned int i = 0; i < obs.size(); i++) {
		  PQP_Distance(&res, unitary, zeros, &uav, unitary, zeros, obs.at(i), 0.001, 0.001);
		  Plane plane;
		  
		  // Compute the normal and the distance
// 		  Tri *tri = obs.at(i)->last_tri;
		  Vector3 relativePosition;
		  
		  getObstacleDistance(obs.at(i), relativePosition);
		  
		  functions::Point3D relativePosition_(relativePosition.x(), relativePosition.y(), relativePosition.z());
		  
		  
		  const Vector3 w = velocity_ - invTimeObstacle * relativePosition;
		  /* Vector from cutoff center to relative velocity. */
		  const float wLengthSq = absSq(w);
		  const float dotProduct = w * relativePosition;
		  const float distSq = relativePosition_*relativePosition_;
		  
		  // Once the triangle is obtained get the constrains related to it!
		  Vector3 u;
		  
		  if (relativePosition_.norm() > obstacleDist_ ) {
		    continue;
		  }
		  
		  float radius_2 = radius_obstacle_;
		  
		  if (distSq > radius_2) {
		    // No collision 
		    /* Always project on cut-off plane. */
		    const float wLength = std::sqrt(wLengthSq);
		    const Vector3 unitW = w / wLength;
		    u = (radius_2 * invTimeObstacle - wLength) * unitW;
		    plane.normal = unitW;
		  } else {
		    // Collision
		    std::cerr << "Collision with obstacles. Distsq = " << distSq << " ";
		    
		    const Vector3 w = velocity_ - invTimeHorizon * collision_multiplier * relativePosition;
		    const float wLength = abs(w);
		    const Vector3 unitW = w / wLength;

		    plane.normal = unitW;
		    u = (radius_2 * invTimeStep - wLength) * unitW;
		    
		    mult = frozen_mult;
		  }
// 		  --------DEBUG---------------
// 		  functions::Point3D p(u.x(), u.y(), u.z());
// 		  functions::Point3D p1(plane.normal.x(), plane.normal.y(), plane.normal.z());
// 		  std::cout << "Reaction: u = " << p.toString() <<std::endl ; //<< ". Normal = " << p1.toString() << ;
// 		  functions::Point3D vel_(velocity_.x(), velocity_.y(), velocity_.z());
		  if (radius_obstacle_z < radius_obstacle_ && radius_obstacle_ > 0.0) {
		    u[2] *= radius_obstacle_ / radius_obstacle_z;
		    u[2] *= z_multiplier;
		  }
		  
		  plane.point = velocity_ + 1.1 * u; // In this case the agent has to deal with all the reaction.
		  orcaPlanes_.push_back(plane);
		}
		
		size_t numObstPlanes = orcaPlanes_.size();
		std::cout << "NumObstPlanes = " << orcaPlanes_.size() << std::endl;

		/* Create agent ORCA planes. */
		bool collision = false;
		
		Vector3 perpPos;
		for (size_t i = 0; i < agentNeighbors_.size(); ++i) {
			bool not_pass = false;
			const Agent *const other = agentNeighbors_[i].second;
			Vector3 relativePosition = other->position_ - position_;
			if ( radius_z > 0.0f && radius_z != radius_) {
			  relativePosition[2] *= radius_ / radius_z; // TODO: preprocess this ratio once
			}
			
			const Vector3 relativeVelocity = velocity_ - other->velocity_;
			const float distSq = absSq(relativePosition);
			const float combinedRadius = radius_ + other->radius_;
			const float combinedRadiusSq = sqr(combinedRadius);
			const float combinedWarning = radius_warning + other->radius_warning;
			const float combinedWarningSq = sqr(combinedWarning);
			
			// Further transform the shape 1. x pow 3
// 			bool exponentiate = false;
// 			double x = sqrt(pow(relativePosition[0], 2.0) + pow(relativePosition[1], 2.0)) / combinedRadius;
// 			double z = relativePosition[2] / combinedRadius;
// 			if ( x < 1.0 ) {
// 			  relativePosition[0] *= pow(x, exponent);
// 			  relativePosition[1] *= pow(x, exponent);
// 			  exponentiate = true;
// 			}
// 			if (  relativePosition[2] < combinedRadius ) {
// 			  relativePosition[2] = pow(boost::math::sign(z) * z, exponent_z) * combinedRadius;
// 			}
			
			Plane plane;
			Vector3 u;

			if (distSq > combinedWarningSq) {
				/* No collision. */
				const Vector3 w = relativeVelocity - invTimeHorizon * relativePosition;
				/* Vector from cutoff center to relative velocity. */
				const float wLengthSq = absSq(w);

				const float dotProduct = w * relativePosition;

				if (dotProduct < 0.0f && sqr(dotProduct) > combinedRadiusSq * wLengthSq) {
					/* Project on cut-off circle. */
					const float wLength = std::sqrt(wLengthSq);
					const Vector3 unitW = w / wLength;

					plane.normal = unitW;
					u = (combinedRadius * invTimeHorizon - wLength) * unitW;
				}
				else {
					/* Project on cone. */
					const float a = distSq;
					const float b = relativePosition * relativeVelocity;
					const float c = absSq(relativeVelocity) - absSq(cross(relativePosition, relativeVelocity)) / (distSq - combinedRadiusSq);
					const float t = (b + std::sqrt(sqr(b) - a * c)) / a;
					const Vector3 w = relativeVelocity - t * relativePosition;
					const float wLength = abs(w);
					const Vector3 unitW = w / wLength;

					plane.normal = unitW;
					u = (combinedRadius * t - wLength) * unitW;
				}
			} else if (distSq > combinedRadiusSq) {
			  /* Conflict. */
//  			  percentage_action = 0.5 + 0.5 * (combinedWarningSq - distSq) /
//  						   ( combinedWarningSq - combinedRadiusSq);
				const Vector3 w = relativeVelocity - invTimeStep * relativePosition;
				const float wLength = abs(w);
				const Vector3 unitW = w / wLength;

				plane.normal = unitW;
				u = (combinedWarning * invTimeStep - wLength) * unitW;
				if (combinedRadiusSq < combinedWarningSq) {
				  mult = functions::minimum(( (1 - frozen_mult) * distSq + frozen_mult * combinedWarningSq - combinedRadiusSq) / (combinedWarningSq - combinedRadiusSq), 
			      mult);
// 				  std::cerr << "Conflict: multiplier " << mult << std::endl;
			  }
			}
			else {
				/* Collision. */
// 				const float invTimeStep = 1.0f / sim_->timeStep_;
				const Vector3 w = relativeVelocity - invTimeStep * relativePosition;
				const float wLength = abs(w);
				const Vector3 unitW = w / wLength;

				plane.normal = unitW;
				u = (combinedRadius * invTimeStep - wLength) * unitW;
				not_pass = true; // This condition cannot be relaxed!
				collision = true;
				percentage_action = 0.55;
				mult = frozen_mult;
			}
			if (radius_ > radius_z && radius_z > 0.0) {
			  u[2] *= radius_/ radius_z;
			  u[2] *= z_multiplier;
			}
// 			if (exponentiate) {
// 			  if (x > 0.001) {
// 			    relativePosition[0] /= pow(x, exponent);
// 			    relativePosition[1] /= pow(x, exponent);
// 			  }
// 			}
			
			// Casuistic of HRVO: wrong or right side?
			plane.point = velocity_ + percentage_action * u;
// 			plane.normal = u / abs(u);
			
			orcaPlanes_.push_back(plane);
			if (not_pass) {
			  std::cerr << "Collision detected between: " << id_ << " and " << agentNeighbors_[i].second->id_ ;
			  std::cerr << "DistSq: " << distSq << "\n";
// 			  std::cout << "Adding the plane in the first element.\n";
// 			  std::cout << "Plane to add: " << plane.toString() << "\n";
// 			  orcaPlanes_.push_back(plane);
			  
			  for (int i = orcaPlanes_.size() - 2; i >= 0 ; i--) {
			    orcaPlanes_[i + 1] = orcaPlanes_[i];
			  }
			  orcaPlanes_[0] = plane;
			  std::cout << "Plane at 0: " << orcaPlanes_.at(0).toString() << "\n";
			  numObstPlanes++;
			}
		}

		const size_t planeFail = linearProgram3(orcaPlanes_, maxSpeed_, prefVelocity_, false, newVelocity_);
		bool recompute = false;

		if (planeFail < orcaPlanes_.size()) {
		  
// 		  std::cerr << "Unfeasible problem.\n";
		  if (!linearProgram4(orcaPlanes_, planeFail, maxSpeed_, newVelocity_, numObstPlanes)) {
		    recompute = true;
		    
// 		    computeNewVelocity(obs, true);
		  }
		} else {
		  recompute = true;
		}
		if (recompute) {
		  std::vector<Plane> copy;
		  if (orcaPlanes_.size() > 0) {
		    copy.push_back(orcaPlanes_.at(0));
		  }
		  if (orcaPlanes_.size() > 1) {
		    copy.push_back(orcaPlanes_.at(1));
		  } 
		    
		  const size_t planeFail = linearProgram3(copy, maxSpeed_, prefVelocity_, false, newVelocity_);
		  
		  if (planeFail < orcaPlanes_.size()) {
		    if (!linearProgram4(orcaPlanes_, planeFail, maxSpeed_, newVelocity_, 0)) {
// 		      std::cerr << "Esto no tiene arreglo\n";
		      orcaPlanes_.clear();
		      orcaPlanes_.push_back(copy.at(0));
		      const size_t planeFail = linearProgram3(orcaPlanes_, maxSpeed_, prefVelocity_, false, newVelocity_);
		      if (planeFail < orcaPlanes_.size()) {
			if (!linearProgram4(orcaPlanes_, planeFail, maxSpeed_, newVelocity_, 0)) {
// 			  std::cerr << "Esto no tiene arreglo2\n";
			}
		      }
		    } else{
// 			std::cerr << "Relaxing conditions\n";
		    }
		  }
		}
		if (collision) {
		  newVelocity_ *= mult;
		}
		
	}
	
	

	void Agent::insertAgentNeighbor(const Agent *agent, float &rangeSq)
	{
		if (this != agent) {
			const float distSq = absSq(position_ - agent->position_);

			if (distSq < rangeSq) {
				if (agentNeighbors_.size() < maxNeighbors_) {
					agentNeighbors_.push_back(std::make_pair(distSq, agent));
				}

				size_t i = agentNeighbors_.size() - 1;

				
				while (i != 0 && distSq < agentNeighbors_[i - 1].first) {
					agentNeighbors_[i] = agentNeighbors_[i - 1];
					--i;
				}

				agentNeighbors_[i] = std::make_pair(distSq, agent);

				if (agentNeighbors_.size() == maxNeighbors_) {
					rangeSq = agentNeighbors_.back().first;
				}
			}
		}
	}

	void Agent::update()
	{
		velocity_ = newVelocity_;
		position_ += velocity_ * sim_->timeStep_;
	}

	bool linearProgram1(const std::vector<Plane> &planes, size_t planeNo, const Line &line, float radius, const Vector3 &optVelocity, bool directionOpt, Vector3 &result)
	{
		const float dotProduct = line.point * line.direction;
		const float discriminant = sqr(dotProduct) + sqr(radius) - absSq(line.point);

		if (discriminant < 0.0f) {
			/* Max speed sphere fully invalidates line. */
			return false;
		}

		const float sqrtDiscriminant = std::sqrt(discriminant);
		float tLeft = -dotProduct - sqrtDiscriminant;
		float tRight = -dotProduct + sqrtDiscriminant;

		for (size_t i = 0; i < planeNo; ++i) {
			const float numerator = (planes[i].point - line.point) * planes[i].normal;
			const float denominator = line.direction * planes[i].normal;

			if (sqr(denominator) <= RVO_EPSILON) {
				/* Lines line is (almost) parallel to plane i. */
				if (numerator > 0.0f) {
					return false;
				}
				else {
					continue;
				}
			}

			const float t = numerator / denominator;

			if (denominator >= 0.0f) {
				/* Plane i bounds line on the left. */
				tLeft = std::max(tLeft, t);
			}
			else {
				/* Plane i bounds line on the right. */
				tRight = std::min(tRight, t);
			}

			if (tLeft > tRight) {
				return false;
			}
		}

		if (directionOpt) {
			/* Optimize direction. */
			if (optVelocity * line.direction > 0.0f) {
				/* Take right extreme. */
				result = line.point + tRight * line.direction;
			}
			else {
				/* Take left extreme. */
				result = line.point + tLeft * line.direction;
			}
		}
		else {
			/* Optimize closest point. */
			const float t = line.direction * (optVelocity - line.point);

			if (t < tLeft) {
				result = line.point + tLeft * line.direction;
			}
			else if (t > tRight) {
				result = line.point + tRight * line.direction;
			}
			else {
				result = line.point + t * line.direction;
			}
		}

		return true;
	}

	bool linearProgram2(const std::vector<Plane> &planes, size_t planeNo, float radius, const Vector3 &optVelocity, bool directionOpt, Vector3 &result)
	{
		const float planeDist = planes[planeNo].point * planes[planeNo].normal;
		const float planeDistSq = sqr(planeDist);
		const float radiusSq = sqr(radius);

		if (planeDistSq > radiusSq) {
			/* Max speed sphere fully invalidates plane planeNo. */
			return false;
		}

		const float planeRadiusSq = radiusSq - planeDistSq;

		const Vector3 planeCenter = planeDist * planes[planeNo].normal;

		if (directionOpt) {
			/* Project direction optVelocity on plane planeNo. */
			const Vector3 planeOptVelocity = optVelocity - (optVelocity * planes[planeNo].normal) * planes[planeNo].normal;
			const float planeOptVelocityLengthSq = absSq(planeOptVelocity);

			if (planeOptVelocityLengthSq <= RVO_EPSILON) {
				result = planeCenter;
			}
			else {
				result = planeCenter + std::sqrt(planeRadiusSq / planeOptVelocityLengthSq) * planeOptVelocity;
			}
		}
		else {
			/* Project point optVelocity on plane planeNo. */
			result = optVelocity + ((planes[planeNo].point - optVelocity) * planes[planeNo].normal) * planes[planeNo].normal;

			/* If outside planeCircle, project on planeCircle. */
			if (absSq(result) > radiusSq) {
				const Vector3 planeResult = result - planeCenter;
				const float planeResultLengthSq = absSq(planeResult);
				result = planeCenter + std::sqrt(planeRadiusSq / planeResultLengthSq) * planeResult;
			}
		}

		for (size_t i = 0; i < planeNo; ++i) {
			if (planes[i].normal * (planes[i].point - result) > 0.0f) {
				/* Result does not satisfy constraint i. Compute new optimal result. */
				/* Compute intersection line of plane i and plane planeNo. */
				Vector3 crossProduct = cross(planes[i].normal, planes[planeNo].normal);

				if (absSq(crossProduct) <= RVO_EPSILON) {
					/* Planes planeNo and i are (almost) parallel, and plane i fully invalidates plane planeNo. */
					return false;
				}

				Line line;
				line.direction = normalize(crossProduct);
				const Vector3 lineNormal = cross(line.direction, planes[planeNo].normal);
				line.point = planes[planeNo].point + (((planes[i].point - planes[planeNo].point) * planes[i].normal) / (lineNormal * planes[i].normal)) * lineNormal;

				if (!linearProgram1(planes, i, line, radius, optVelocity, directionOpt, result)) {
					return false;
				}
			}
		}

		return true;
	}

	size_t linearProgram3(const std::vector<Plane> &planes, float radius, const Vector3 &optVelocity, bool directionOpt, Vector3 &result)
	{
		if (directionOpt) {
			/* Optimize direction. Note that the optimization velocity is of unit length in this case. */
			result = optVelocity * radius;
		}
		else if (absSq(optVelocity) > sqr(radius)) {
			/* Optimize closest point and outside circle. */
			result = normalize(optVelocity) * radius;
		}
		else {
			/* Optimize closest point and inside circle. */
			result = optVelocity;
		}

		for (size_t i = 0; i < planes.size(); ++i) {
			if (planes[i].normal * (planes[i].point - result) > 0.0f) {
				/* Result does not satisfy constraint i. Compute new optimal result. */
				const Vector3 tempResult = result;

				if (!linearProgram2(planes, i, radius, optVelocity, directionOpt, result)) {
					result = tempResult;
					return i;
				}
			}
		}

		return planes.size();
	}

	bool linearProgram4(const std::vector<Plane> &planes, size_t beginPlane, float radius, Vector3 &result, size_t numObstPlanes)
	{
		float distance = 0.0f;
		bool ret = true;

		for (size_t i = beginPlane; i < planes.size(); ++i) {
			if (planes[i].normal * (planes[i].point - result) > distance) {
				/* Result does not satisfy constraint of plane i. */
				std::vector<Plane> projPlanes(planes.begin(), planes.begin() + static_cast<ptrdiff_t>(numObstPlanes));

				for (size_t j = numObstPlanes; j < i; ++j) {
					Plane plane;

					const Vector3 crossProduct = cross(planes[j].normal, planes[i].normal);

					if (absSq(crossProduct) <= RVO_EPSILON) {
						/* Plane i and plane j are (almost) parallel. */
						if (planes[i].normal * planes[j].normal > 0.0f) {
							/* Plane i and plane j point in the same direction. */
							continue;
						}
						else {
							/* Plane i and plane j point in opposite direction. */
							plane.point = 0.5f * (planes[i].point + planes[j].point);
						}
					}
					else {
						/* Plane.point is point on line of intersection between plane i and plane j. */
						const Vector3 lineNormal = cross(crossProduct, planes[i].normal);
						plane.point = planes[i].point + (((planes[j].point - planes[i].point) * planes[j].normal) / (lineNormal * planes[j].normal)) * lineNormal;
					}

					plane.normal = normalize(planes[j].normal - planes[i].normal);
					projPlanes.push_back(plane);
				}
				
				const Vector3 tempResult = result;
				
				if (linearProgram3(projPlanes, radius, planes[i].normal, true, result) < projPlanes.size()) {
					ret = false;
					/* This should in principle not happen.  The result is by definition already in the feasible region of this linear program. If it fails, it is due to small floating point error, and the current result is kept. */
					result = tempResult;
					if (abs(tempResult) > 1.0) {
					  std::cerr << "Really unfeasible --> replanning with the closest agents with closest obstacle. \n ";
					  result = Vector3(0.0f, 0.0f, 0.0f);
					}
					
					
// 				 	std::cerr << "Frozen mult: " << frozen_mult << std::endl;
					
// 					std::cerr << result.x() << " " <<result.y() << " "<< result.z() << ".\n";
				}
				
				distance = planes[i].normal * (planes[i].point - result);
			}
		}
		return ret;
	}
	
	Vector3 Agent::getNormal(const Tri *t) const {
	  functions::Point3D t1(t->p1[0], t->p1[1], t->p1[2]);
	  functions::Point3D t2(t->p2[0], t->p2[1], t->p2[2]);
	  functions::Point3D t3(t->p3[0], t->p3[1], t->p3[2]);
	  functions::Point3D a, b;
	  a = t2 - t1;
	  b = t3 - t1;
	  functions::Point3D norm = a.crossProduct(b);
	  norm.normalize();
	  Vector3 ret(norm.x, norm.y, norm.z);
	  
	  // Debug
	  
// 	  std::cout << "Normal = " << norm.toString() << std::endl;
	  
	  
	  return ret;
	  
	 
	}
	
	float Agent::getObstacleDistance(const std::vector<PQP_Model *> &obs) {
		float ret = 1e100;
	  	PQP_DistanceResult res;
		// First generate the model, a square of 10cm side
		PQP_Model uav;
		p1[0] = position_.x() + 0.05;p1[1] = position_.y() + 0.05;p1[2]= position_.z();
		p2[0] = position_.x() + 0.05;p2[1] = position_.y() - 0.05;p2[2]= position_.z();
		p3[0] = position_.x() - 0.05;p3[1] = position_.y() + 0.05;p3[2]= position_.z();
		p4[0] = position_.x() - 0.05;p4[1] = position_.y() - 0.05;p4[2]= position_.z();
		uav.AddTri(p1, p2, p3, 0);
		uav.AddTri(p4, p2, p3, 1);
		uav.EndModel();
		for (unsigned int i = 0; i < obs.size(); i++) {
		  PQP_Distance(&res, unitary, zeros, &uav, unitary, zeros, obs.at(i), 0.001, 0.001);
		  ret = (res.Distance() > ret) ? ret : res.Distance();
		}
		return ret;
	}
	
	float Agent::getObstacleDistance(PQP_Model *obs, Vector3 &relativePosition) {
		float ret = 1e100;
	  	PQP_DistanceResult res;
		// First generate the model, a square of 10cm side
		PQP_Model uav;
		p1[0] = position_.x() + 0.05;p1[1] = position_.y() + 0.05;p1[2]= position_.z();
		p2[0] = position_.x() + 0.05;p2[1] = position_.y() - 0.05;p2[2]= position_.z();
		p3[0] = position_.x() - 0.05;p3[1] = position_.y() + 0.05;p3[2]= position_.z();
		p4[0] = position_.x() - 0.05;p4[1] = position_.y() - 0.05;p4[2]= position_.z();
		uav.AddTri(p1, p2, p3, 0);
		uav.AddTri(p4, p2, p3, 1);
		uav.EndModel();
		
		ret = PQP_Distance(&res, unitary, zeros, &uav, unitary, zeros, obs, 0.001, 0.001);
		
		functions::Point3D p1_(res.p1[0], res.p1[1], res.p1[2]);
		functions::Point3D p2_(res.p2[0], res.p2[1], res.p2[2]);
		functions::Point3D relativePosition_ = p2_ - p1_;
		if ( radius_obstacle_z > 0.0f && radius_obstacle_z != radius_obstacle_) {
		  relativePosition_.z *= radius_obstacle_ / radius_obstacle_z; // TODO: preprocess this ratio once
		}
		relativePosition[0] = relativePosition_.x;
		relativePosition[1] = relativePosition_.y;
		relativePosition[2] = relativePosition_.z;
		
		return ret;
	}
	
std::vector< PQP_DistanceResult > Agent::getClosestPoints(const std::vector<PQP_Model *> &obs) const
{
  std::vector<PQP_DistanceResult> ret;
  
  PQP_REAL p1[3];
  PQP_REAL p2[3];
  PQP_REAL p3[3];
  PQP_REAL p4[3];
  
  PQP_DistanceResult res;
  PQP_Model uav;
  p1[0] = position_.x() + 0.05;p1[1] = position_.y() + 0.05;p1[2]= position_.z();
  p2[0] = position_.x() + 0.05;p2[1] = position_.y() - 0.05;p2[2]= position_.z();
  p3[0] = position_.x() - 0.05;p3[1] = position_.y() + 0.05;p3[2]= position_.z();
  p4[0] = position_.x() - 0.05;p4[1] = position_.y() - 0.05;p4[2]= position_.z();
  uav.AddTri(p1, p2, p3, 0);
  uav.AddTri(p4, p2, p3, 1);
  uav.EndModel();
  
//   std::cerr << "Obs size: " << obs.size() << " ";
  for (unsigned int i = 0; i < obs.size(); i++) {
    PQP_Distance(&res, unitary, zeros, &uav, unitary, zeros, obs.at(i), 0.001, 0.001);
//     std::cerr << "distance: " << res.Distance() << " ";
    if (res.Distance() < obstacleDist_) {
      ret.push_back(res);
    }
  }
//   std::cerr << std::endl;
  
  return ret;
}

}
