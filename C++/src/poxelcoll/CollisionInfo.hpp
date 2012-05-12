/* CollisionInfo.hpp */

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

#ifndef POXELCOLL_COLLISIONINFO_HPP_
#define POXELCOLL_COLLISIONINFO_HPP_

#include "mask/Mask.hpp"

namespace poxelcoll {

/** \ingroup poxelcollgeometrymatrix
 *
 * Collision info represents a collision object, including the mask, a position, an angle, scaling, and an id.
 *
 * The order of transformation is: origin, scaling, rotation, position.
 *
 * Angle is in radians, position is in pixels, and the scaling factors are percentages (where 1.0 == 100%).
 */
class CollisionInfo {
private:
	const std::shared_ptr<const Mask> mask;
	const P position;
	const double angle;
	const double scaleX;
	const double scaleY;
	const int id;

public:
	CollisionInfo(const std::shared_ptr<const Mask> aMask, const P aPosition, const double aAngle,
			const double aScaleX, const double aScaleY, const int aId);

public:

	const std::shared_ptr<const Mask> gMask() const;

	const P gPosition() const;

	const double gAngle() const;

	const double gScaleX() const;

	const double gScaleY() const;

	const int gId() const;
};

}

#endif /* POXELCOLL_COLLISIONINFO_HPP_ */
