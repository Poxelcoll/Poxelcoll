/* Mask.cpp */

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

#include "Mask.hpp"

namespace poxelcoll {

Mask::Mask(const P origin, const BoundingBox boundingBox,
		const std::shared_ptr<const NonemptyConvexCCWPolygon> convexHull,
		const std::shared_ptr<const BinaryImage> binaryImageNull) :
		myOrigin(origin), myBoundingBox(boundingBox), myConvexHull(
				convexHull), myBinaryImageNull(binaryImageNull) {
}

const P Mask::origin() const {
	return myOrigin;
}

const BoundingBox Mask::boundingBox() const {
	return myBoundingBox;
}

const std::shared_ptr<const NonemptyConvexCCWPolygon> Mask::convexHull() const {
	return myConvexHull;
}

const std::shared_ptr<const BinaryImage> Mask::binaryImageNull() const {
	return myBinaryImageNull;
}

const bool Mask::isPolygonFull() const {
	return myBinaryImageNull.get() == 0;
}

}
