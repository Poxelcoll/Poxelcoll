/* DataTypes.hpp */

/* Copyright (C) 2012 Jens W.-Møller
 * All rights reserved.
 *
 * This file is part of Poxelcoll.
 *
 * Poxelcoll is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Poxelcoll is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Poxelcoll.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef POXELCOLL_DATATYPES_HPP_
#define POXELCOLL_DATATYPES_HPP_

#include <math.h>
#include <sstream>

namespace poxelcoll {

/** \ingroup poxelcoll
 *
 * A point with two coordinates. Can also be used to represent vectors.
 */
class P {
private:
	double x; //Do not change.
	double y; //Do not change.
public:
	P(double aX, double aY);
public:
	struct Comparer {
	  bool operator() (const poxelcoll::P & lhs, const poxelcoll::P & rhs) const
	  {
		  return lhs.gX() < rhs.gX() || (lhs.gX() == rhs.gX() && lhs.gY() < rhs.gY());
	  }
	};
public:

	double gX() const;
	double gY() const;

	/** Vector addition.
	 *
	 * @param that other vector
	 * @return the sum vector
	 */
	const P plus(P that) const;
	/** Vector subtraction.
	 *
	 * @param that other vector
	 * @return this vector minus that vector
	 */
	const P minus(P that) const;
	/** Scaling.
	 *
	 * @param scale factor
	 * @return scaled point/vector
	 */
	const P multi(double scale) const;

	/** The negative of the coordinates.
	 *
	 * @return the negative point/vector.
	 */
	const P unaryMinus() const;

	/** Cross-product.
	 *
	 * @param that other vector
	 * @return this vector cross-product that vector
	 */
	const double cross(P that) const;

	/** Dot-product.
	 *
	 * @param that other vector
	 * @return this vector dot that vector
	 */
	const double dot(P that) const;
	/** Inverse scale. Undefined if inverseScale == 0.0.
	 *
	 * @param inverseScale non-zero scale
	 * @return inversely scale point/vector
	 */
	const P divide(double inverseScale) const;

	/** The length of the vector/distance from the point to origo.
	 *
	 * @return the length of the vector
	 */
	const double norm() const;

	/** @return unsafe normalization of vector */
	const P normaUnsafe() const;

	const bool equal(P that) const;

	const std::string toString() const;
};

/** \ingroup poxelcoll
 *
 * 2-dimensional integer point/vector.
 */
class IP { //Integer Point.
private:
	int x; //Do not change.
	int y; //Do not change.

public:
	IP(int aX, int aY);
public:

	struct Comparer {
	  bool operator() (const poxelcoll::IP & lhs, const poxelcoll::IP & rhs) const
	  {
		  return lhs.gX() < rhs.gX() || (lhs.gX() == rhs.gX() && lhs.gY() < rhs.gY());
	  }
	};
public:

	int gX() const;
	int gY() const;

	/** Point/vector addition.
	 *
	 * @param that other vector
	 * @return this vector plus that vector
	 */
	const IP plus(IP that) const;

	/** Point/vector subtraction.
	 *
	 * @param that other vector
	 * @return this vector minus that vector
	 */
	const IP minus(IP that) const;

	/** @return convert to double-precision point */
	const P toP() const;
};

/** \ingroup poxelcoll
 *
 * An axis-aligned bounding box.
 *
 * Invariant: Both of pMin's coordinates must be smaller or equal than the corresponding coordinates in pMax.
 */
class BoundingBox { //Points are allowed to be the same, or have the same x-values or y-values.
public:
	const P pMin;
	const P pMax;
public:

	BoundingBox(const P & aPMin, const P & aPMax);

	/** Whether or not this axis-aligned bounding box intersects another bounding box.
	 *
	 * @param that other axis-aligned bounding box
	 * @return whether the boxes intersects
	 */
	const bool intersects(BoundingBox that) const;

	const std::string toString() const;
};

/** \ingroup poxelcoll
 *
 * A collision pair indicates that two collision objects with strictly different ids
 * has collided.
 *
 * Invariant: id1 is always strictly smaller than id2.
 */
class CollisionPair {
public:
	const int id1;
	const int id2;
public:

	/** Creation of collision pair. This method is the preferred way of creating collision pairs.
	 *
	 * @param id1 first id
	 * @param id2 second id
	 * @return the collision pair, or none if the ids are equivalent
	 */
	static CollisionPair* createNull(int id1, int id2) {
		if (id1 == id2) {
			return 0;
		} else {
			if (id1 < id2) {
				return new CollisionPair(id1, id2);
			} else {
				return new CollisionPair(id2, id1);
			}
		}
	}
private:
	CollisionPair(int aId1, int aId2);
};

}

#endif /* POXELCOLL_DATATYPES_HPP_ */
