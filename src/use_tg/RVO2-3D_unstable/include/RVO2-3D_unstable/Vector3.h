/*
 * Vector3.h
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

/**
 * \file    Vector3.h
 * \brief   Contains the Vector3 class.
 */
#ifndef RVO_UNSTABLE_VECTOR3_H_
#define RVO_UNSTABLE_VECTOR3_H_

#include "API.h"

#include <cmath>
#include <cstddef>
#include <ostream>

namespace RVO_UNSTABLE {
	/**
	 * \brief  Defines a three-dimensional vector.
	 */
	class Vector3 {
	public:
		/**
		 * \brief   Constructs and initializes a three-dimensional vector instance to zero.
		 */
		RVO_UNSTABLE_API inline Vector3()
		{
			val_[0] = 0.0f;
			val_[1] = 0.0f;
			val_[2] = 0.0f;
		}

		/**
		 * \brief   Constructs and initializes a three-dimensional vector from the specified three-dimensional vector.
		 * \param   vector  The three-dimensional vector containing the xyz-coordinates.
		 */
		RVO_UNSTABLE_API inline Vector3(const Vector3 &vector)
		{
			val_[0] = vector[0];
			val_[1] = vector[1];
			val_[2] = vector[2];
		}

		/**
		 * \brief   Constructs and initializes a three-dimensional vector from the specified three-element array.
		 * \param   val  The three-element array containing the xyz-coordinates.
		 */
		RVO_UNSTABLE_API inline explicit Vector3(const float val[3])
		{
			val_[0] = val[0];
			val_[1] = val[1];
			val_[2] = val[2];
		}

		/**
		 * \brief   Constructs and initializes a three-dimensional vector from the specified xyz-coordinates.
		 * \param   x  The x-coordinate of the three-dimensional vector.
		 * \param   y  The y-coordinate of the three-dimensional vector.
		 * \param   z  The z-coordinate of the three-dimensional vector.
		 */
		RVO_UNSTABLE_API inline Vector3(float x, float y, float z)
		{
			val_[0] = x;
			val_[1] = y;
			val_[2] = z;
		}

		/**
		 * \brief   Returns the x-coordinate of this three-dimensional vector.
		 * \return  The x-coordinate of the three-dimensional vector.
		 */
		RVO_UNSTABLE_API inline float x() const { return val_[0]; }

		/**
		 * \brief   Returns the y-coordinate of this three-dimensional vector.
		 * \return  The y-coordinate of the three-dimensional vector.
		 */
		RVO_UNSTABLE_API inline float y() const { return val_[1]; }

		/**
		 * \brief   Returns the z-coordinate of this three-dimensional vector.
		 * \return  The z-coordinate of the three-dimensional vector.
		 */
		RVO_UNSTABLE_API inline float z() const { return val_[2]; }

		/**
		 * \brief   Returns the specified coordinate of this three-dimensional vector.
		 * \param   i  The coordinate that should be returned (0 <= i < 3).
		 * \return  The specified coordinate of the three-dimensional vector.
		 */
		RVO_UNSTABLE_API inline float operator[](size_t i) const { return val_[i]; }

		/**
		 * \brief   Returns a reference to the specified coordinate of this three-dimensional vector.
		 * \param   i  The coordinate to which a reference should be returned (0 <= i < 3).
		 * \return  A reference to the specified coordinate of the three-dimensional vector.
		 */
		RVO_UNSTABLE_API inline float &operator[](size_t i) { return val_[i]; }

		/**
		 * \brief   Computes the negation of this three-dimensional vector.
		 * \return  The negation of this three-dimensional vector.
		 */
		RVO_UNSTABLE_API inline Vector3 operator-() const
		{
			return Vector3(-val_[0], -val_[1], -val_[2]);
		}

		/**
		 * \brief   Computes the dot product of this three-dimensional vector with the specified three-dimensional vector.
		 * \param   vector  The three-dimensional vector with which the dot product should be computed.
		 * \return  The dot product of this three-dimensional vector with a specified three-dimensional vector.
		 */
		RVO_UNSTABLE_API inline float operator*(const Vector3 &vector) const
		{
			return val_[0] * vector[0] + val_[1] * vector[1] + val_[2] * vector[2];
		}

		/**
		 * \brief   Computes the scalar multiplication of this three-dimensional vector with the specified scalar value.
		 * \param   scalar  The scalar value with which the scalar multiplication should be computed.
		 * \return  The scalar multiplication of this three-dimensional vector with a specified scalar value.
		 */
		RVO_UNSTABLE_API inline Vector3 operator*(float scalar) const
		{
			return Vector3(val_[0] * scalar, val_[1] * scalar, val_[2] * scalar);
		}

		/**
		 * \brief   Computes the scalar division of this three-dimensional vector with the specified scalar value.
		 * \param   scalar  The scalar value with which the scalar division should be computed.
		 * \return  The scalar division of this three-dimensional vector with a specified scalar value.
		 */
		RVO_UNSTABLE_API inline Vector3 operator/(float scalar) const
		{
			const float invScalar = 1.0f / scalar;

			return Vector3(val_[0] * invScalar, val_[1] * invScalar, val_[2] * invScalar);
		}

		/**
		 * \brief   Computes the vector sum of this three-dimensional vector with the specified three-dimensional vector.
		 * \param   vector  The three-dimensional vector with which the vector sum should be computed.
		 * \return 	The vector sum of this three-dimensional vector with a specified three-dimensional vector.
		 */
		RVO_UNSTABLE_API inline Vector3 operator+(const Vector3 &vector) const
		{
			return Vector3(val_[0] + vector[0], val_[1] + vector[1], val_[2] + vector[2]);
		}

		/**
		 * \brief   Computes the vector difference of this three-dimensional vector with the specified three-dimensional vector.
		 * \param   vector  The three-dimensional vector with which the vector difference should be computed.
		 * \return  The vector difference of this three-dimensional vector with a specified three-dimensional vector.
		 */
		RVO_UNSTABLE_API inline Vector3 operator-(const Vector3 &vector) const
		{
			return Vector3(val_[0] - vector[0], val_[1] - vector[1], val_[2] - vector[2]);
		}

		/**
		 * \brief   Tests this three-dimensional vector for equality with the specified three-dimensional vector.
		 * \param   vector  The three-dimensional vector with which to test for equality.
		 * \return  True if the three-dimensional vectors are equal.
		 */
		RVO_UNSTABLE_API inline bool operator==(const Vector3 &vector) const
		{
			return val_[0] == vector[0] && val_[1] == vector[1] && val_[2] == vector[2];
		}

		/**
		 * \brief   Tests this three-dimensional vector for inequality with the specified three-dimensional vector.
		 * \param   vector  The three-dimensional vector with which to test for inequality.
		 * \return  True if the three-dimensional vectors are not equal.
		 */
		RVO_UNSTABLE_API inline bool operator!=(const Vector3 &vector) const
		{
			return val_[0] != vector[0] || val_[1] != vector[1] || val_[2] != vector[2];
		}

		/**
		 * \brief   Sets the value of this three-dimensional vector to the scalar multiplication of itself with the specified scalar value.
		 * \param   scalar  The scalar value with which the scalar multiplication should be computed.
		 * \return  A reference to this three-dimensional vector.
		 */
		RVO_UNSTABLE_API inline Vector3 &operator*=(float scalar)
		{
			val_[0] *= scalar;
			val_[1] *= scalar;
			val_[2] *= scalar;

			return *this;
		}

		/**
		 * \brief   Sets the value of this three-dimensional vector to the scalar division of itself with the specified scalar value.
		 * \param   scalar  The scalar value with which the scalar division should be computed.
		 * \return  A reference to this three-dimensional vector.
		 */
		RVO_UNSTABLE_API inline Vector3 &operator/=(float scalar)
		{
			const float invScalar = 1.0f / scalar;

			val_[0] *= invScalar;
			val_[1] *= invScalar;
			val_[2] *= invScalar;

			return *this;
		}

		/**
		 * \brief   Sets the value of this three-dimensional vector to the vector
		 *             sum of itself with the specified three-dimensional vector.
		 * \param   vector  The three-dimensional vector with which the vector sum should be computed.
		 * \return  A reference to this three-dimensional vector.
		 */
		RVO_UNSTABLE_API inline Vector3 &operator+=(const Vector3 &vector)
		{
			val_[0] += vector[0];
			val_[1] += vector[1];
			val_[2] += vector[2];

			return *this;
		}

		/**
		 * \brief   Sets the value of this three-dimensional vector to the vector difference of itself with the specified three-dimensional vector.
		 * \param   vector  The three-dimensional vector with which the vector difference should be computed.
		 * \return  A reference to this three-dimensional vector.
		 */
		RVO_UNSTABLE_API inline Vector3 &operator-=(const Vector3 &vector)
		{
			val_[0] -= vector[0];
			val_[1] -= vector[1];
			val_[2] -= vector[2];

			return *this;
		}

	private:
		float val_[3];
	};


	/**
	 * \relates  Vector3
	 * \brief    Computes the scalar multiplication of the specified three-dimensional vector with the specified scalar value.
	 * \param    scalar  The scalar value with which the scalar multiplication should be computed.
	 * \param    vector  The three-dimensional vector with which the scalar multiplication should be computed.
	 * \return   The scalar multiplication of the three-dimensional vector with the scalar value.
	 */
	inline Vector3 operator*(float scalar, const Vector3 &vector)
	{
		return Vector3(scalar * vector[0], scalar * vector[1], scalar * vector[2]);
	}

	/**
	 * \relates  Vector3
	 * \brief    Computes the cross product of the specified three-dimensional vectors.
	 * \param    vector1  The first vector with which the cross product should be computed.
	 * \param    vector2  The second vector with which the cross product should be computed.
	 * \return   The cross product of the two specified vectors.
	 */
	inline Vector3 cross(const Vector3 &vector1, const Vector3 &vector2)
	{
		return Vector3(vector1[1] * vector2[2] - vector1[2] * vector2[1], vector1[2] * vector2[0] - vector1[0] * vector2[2], vector1[0] * vector2[1] - vector1[1] * vector2[0]);
	}

	/**
	 * \relates  Vector3
	 * \brief    Inserts the specified three-dimensional vector into the specified output stream.
	 * \param    os      The output stream into which the three-dimensional vector should be inserted.
	 * \param    vector  The three-dimensional vector which to insert into the output stream.
	 * \return   A reference to the output stream.
	 */
	inline std::ostream &operator<<(std::ostream &os, const Vector3 &vector)
	{
		os << "(" << vector[0] << "," << vector[1] << "," << vector[2] << ")";

		return os;
	}

	/**
	 * \relates  Vector3
	 * \brief    Computes the length of a specified three-dimensional vector.
	 * \param    vector  The three-dimensional vector whose length is to be computed.
	 * \return   The length of the three-dimensional vector.
	 */
	inline float abs(const Vector3 &vector)
	{
		return std::sqrt(vector * vector);
	}

	/**
	 * \relates  Vector3
	 * \brief    Computes the squared length of a specified three-dimensional vector.
	 * \param    vector  The three-dimensional vector whose squared length is to be computed.
	 * \return   The squared length of the three-dimensional vector.
	 */
	inline float absSq(const Vector3 &vector)
	{
		return vector * vector;
	}

	/**
	 * \relates  Vector3
	 * \brief    Computes the normalization of the specified three-dimensional vector.
	 * \param    vector  The three-dimensional vector whose normalization is to be computed.
	 * \return   The normalization of the three-dimensional vector.
	 */
	inline Vector3 normalize(const Vector3 &vector)
	{
		return vector / abs(vector);
	}
}

#endif
