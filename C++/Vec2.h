/*!
 * Rapid trajectory generation for quadrocopters
 *
 * Copyright 2014 by Mark W. Mueller <mwm@mwm.im>
 *
 * This code is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This code is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the code.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#include <assert.h>
#include <math.h>
#include <limits>

//! 3D vector class
/*!
 * A 3D vector class, to make passing arguments easier, and allow easy addition etc. of vectors.
 * Could be readily replaced with arrays of doubles, but this would complicate some of the code.
 */
class Vec2
{
public:
	double x,y; //!< the three components of the vector

	Vec2(void):x(std::numeric_limits<double>::quiet_NaN()),y(std::numeric_limits<double>::quiet_NaN()){};//!<Initialises all members to NaN
	Vec2(double xin,double yin):x(xin),y(yin){};//!< Initialise vector

	//!Getter function, index 0 <-> x, 1 <-> y, 2 <-> z
	inline double operator[](int i) const
	{
		switch(i)
		{
		case 0: return x;
		case 1: return y;
		break;
		}
		//we're doing something wrong if we get here
		assert(0);
		return std::numeric_limits<double>::quiet_NaN();
	}

	//!Setter function, index 0 <-> x, 1 <-> y, 2 <-> z
	inline double & operator[](int i)
	{
		switch(i)
		{
		case 0: return x;
		case 1: return y;
		break;
		}
		//we're doing something wrong if we get here
		assert(0);
		//fail loudly:
		x = y = std::numeric_limits<double>::quiet_NaN();
		return x;
	}

	//!Calculate the dot product of two vectors
	inline double Dot(const Vec2 rhs) const {return x*rhs.x + y*rhs.y;};

	//!Calculate the Euclidean norm of the vector, squared (= sum of squared elements).
	inline double GetNorm2Squared(void) const {return this->Dot(*this);};

	//!Calculate the Euclidean norm of the vector.
	inline double GetNorm2(void) const {return sqrt(GetNorm2Squared());};

	//!Get the unit vector pointing along the same direction as this vector. Will fail for zero vectors.
	Vec2 GetUnitVector(void) const {return (*this)/this->GetNorm2();};

	inline Vec2 operator+(const Vec2 rhs)const {return Vec2(x+rhs.x,y+rhs.y);};
	inline Vec2 operator-(const Vec2 rhs)const {return Vec2(x-rhs.x,y-rhs.y);};
	inline Vec2 operator/(const double rhs)const {return Vec2(x/rhs,y/rhs);};
};

//Some operator overloading:
inline Vec2 operator*(const double lhs, const Vec2 rhs) {return Vec2(lhs*rhs.x, lhs*rhs.y);};
inline Vec2 operator*(const Vec2 lhs, const double rhs) {return Vec2(lhs.x*rhs, lhs.y*rhs);};
inline Vec2 operator/(const Vec2 lhs, const double rhs) {return Vec2(lhs.x/rhs, lhs.y/rhs);};
