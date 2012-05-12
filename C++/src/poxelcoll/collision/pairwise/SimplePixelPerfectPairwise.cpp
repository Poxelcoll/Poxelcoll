/* SimplePixelPerfectPairwise.cpp */

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

#include "SimplePixelPerfectPairwise.hpp"
#include "../pixelperfect/PixelPerfect.hpp"
#include "../../geometry/matrix/Transformation.hpp"
#include "../../geometry/convexccwpolygon/PolygonIntersection.hpp"

namespace poxelcoll {

const bool SimplePixelPerfectPairwise::testForCollision(
		const std::shared_ptr<const CollisionInfo> collInfo1,
		const std::shared_ptr<const CollisionInfo> collInfo2) const {

	const auto mask1 = (*collInfo1).gMask();
	const auto mask2 = (*collInfo2).gMask();

	const auto transformationMatrix1 = Transformation::getTransformationMatrix(collInfo1);
	const auto transformationMatrix2 = Transformation::getTransformationMatrix(collInfo2);

	const auto inv1Null = (*transformationMatrix1).inverseNull(); //NOTE: Handle null.
	const auto inv2Null = (*transformationMatrix2).inverseNull(); //NOTE: Handle null.

	if (inv1Null == 0 || inv2Null == 0) { //Handling if any of the matrices are null.
		return false; //If the inverse is not well-defined, there is no collision (no inverse == line without width or similar).
	}
	else { //None of the matrices are null.

		//There is a well-defined inverse, continue.

		const std::shared_ptr<const Matrix> inv1(inv1Null);
		const std::shared_ptr<const Matrix> inv2(inv2Null);

		const auto transConHull1 = assumingValidConvexPolygonPointsTransformToCCWEvenIfCW(
				(*transformationMatrix1).transformPoints(*(*(*mask1).convexHull()).points())
		);
		const auto transConHull2 = assumingValidConvexPolygonPointsTransformToCCWEvenIfCW(
				(*transformationMatrix2).transformPoints(*(*(*mask2).convexHull()).points())
		);
		const auto approxBoundingBox1 = Transformation::approximateBoundingBox(transformationMatrix1, (*mask1).boundingBox());
		const auto approxBoundingBox2 = Transformation::approximateBoundingBox(transformationMatrix2, (*mask2).boundingBox());

		//If both full, check for intersection.
		//If not both full, find the intersection.

		const auto otherIntersection = PolygonIntersection::intersection(
				transConHull1, transConHull2,
				(*mask1).isPolygonFull(), (*mask2).isPolygonFull(),
				std::shared_ptr<const BoundingBox>(new BoundingBox(approxBoundingBox1)),
				std::shared_ptr<const BoundingBox>(new BoundingBox(approxBoundingBox2))
		);

		if (otherIntersection.getIsRight()) { //NOTE: Is right.
			const auto collisionIntersection = otherIntersection.getRight();

			const auto testFunction = generalTestFunction(mask1, mask2, inv1, inv2);

			//Given the intersection, test the pixels by taking a pixel in the intersection polygon,
			//and using the inverse transformation matrices to get the corresponding point in the
			//binary image (or if full, just true).

			const auto collisionIntersectionType = (*collisionIntersection).getType();

			switch (collisionIntersectionType) {
			case ConvexCCWType::PointT : {

				const auto a = (*collisionIntersection).getAPoint();

				return PixelPerfect::collisionTest(a, testFunction);
			}
			case ConvexCCWType::LineT : {

				const auto a = (*collisionIntersection).getALine();

				return PixelPerfect::collisionTest(a, testFunction);
			}
			case ConvexCCWType::PolygonT : {

				const auto a = (*collisionIntersection).getAPolygon();

				return PixelPerfect::collisionTest(a, testFunction);
			}
			case ConvexCCWType::EmptyT : {
				return false;
			}
			default: {
				std::cerr << "Did not match any of the ConvexCCWTypes.";
				throw 1;
			}
			}
		}
		else { //NOTE: Is left.
			const auto hasIntersection = *otherIntersection.getLeft();
			return hasIntersection;
		}
	}
}

}


