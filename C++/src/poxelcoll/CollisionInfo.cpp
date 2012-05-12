/* CollisionInfo.cpp */

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

#include "CollisionInfo.hpp"

namespace poxelcoll {

CollisionInfo::CollisionInfo(const std::shared_ptr<const Mask> aMask, const P aPosition, const double aAngle,
		const double aScaleX, const double aScaleY, const int aId):
			mask(aMask), position(aPosition), angle(aAngle),
			scaleX(aScaleX), scaleY(aScaleY), id(aId) {
}

const std::shared_ptr<const Mask> CollisionInfo::gMask() const {
	return mask;
}

const P CollisionInfo::gPosition() const {
	return position;
}

const double CollisionInfo::gAngle() const {
	return angle;
}

const double CollisionInfo::gScaleX() const {
	return scaleX;
}

const double CollisionInfo::gScaleY() const {
	return scaleY;
}

const int CollisionInfo::gId() const {
	return id;
}

}
