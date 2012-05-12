/* Transformation.hpp */

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

#ifndef POXELCOLL_GEOMETRY_MATRIX_TRANSFORMATION_HPP_
#define POXELCOLL_GEOMETRY_MATRIX_TRANSFORMATION_HPP_

#include "Matrix.hpp"

using namespace poxelcoll::functional;

namespace poxelcoll {

/** \ingroup poxelcollgeometrymatrix
 *
 * Supports operations for the default matrix implementation.
 */
class Transformation {

public:

	/** Given the info of a collision object, derive a transformation matrix
	 * from it.
	 *
	 * @param collInfo the collision info of a collision object
	 * @return a transformation matrix that handles origin, translation, scaling and rotation
	 */
	static const std::shared_ptr<const Matrix> getTransformationMatrix(
			const std::shared_ptr<const CollisionInfo> collInfo) {

		//NOTE: Please keep the below out-commented code. It helps act as documentation.

//    val posTransl = {
//      val pos = collInfo.position
//      factory.createMatrixArray(Array(1.0, 0.0, pos.x.toDouble, 0.0, 1.0, pos.y.toDouble, 0.0, 0.0, 1.0))
//    }
//
//    val rot = {
//      val ang = collInfo.angle
//      val cosA = math.cos(ang)
//      val sinA = math.sin(ang)
//      val ang90 = ang + math.Pi/2
//      val cosA90 = math.cos(ang90)
//      val sinA90 = math.sin(ang90)
//      factory.createMatrixArray(Array(cosA, sinA, 0.0, cosA90, sinA90, 0.0, 0.0, 0.0, 1.0))
//    }
//
//    val scaling = {
//      factory.createMatrixArray(Array(collInfo.scaleX, 0.0, 0.0, 0.0, collInfo.scaleY, 0.0, 0.0, 0.0, 1.0))
//    }
//
//    val originTransl = {
//      val origin = collInfo.mask.origin
//      factory.createMatrixArray(Array(1.0, 0.0, -origin.x.toDouble, 0.0, 1.0, -origin.y.toDouble, 0.0, 0.0, 1.0.toDouble))
//    }
//
//    posTransl *** rot *** scaling *** originTransl

//NOTE: If any bugs are found in the below code, please fix them in the above out-commented code too.

		if ((*collInfo).gAngle() != 0.0 || (*collInfo).gScaleX() != 1.0
				|| (*collInfo).gScaleY() != 1) {

			const auto pos = (*collInfo).gPosition();
			const double posX = pos.gX();
			const double posY = pos.gY();

			const auto ang = (*collInfo).gAngle();
			const auto ang90 = ang + M_PI / 2.0;
			const auto cosA = cos(ang);
			const auto sinA = sin(ang);
			const auto cosA90 = cos(ang90);
			const auto sinA90 = sin(ang90);

			const auto scaleX = (*collInfo).gScaleX();
			const auto scaleY = (*collInfo).gScaleY();

			const auto origin = (*(*collInfo).gMask()).origin();
			const double originX = origin.gX();
			const double originY = origin.gY();

			const auto array = boost::array<double, Matrix::dataSize>(
					{
							{ cosA * scaleX, scaleY * sinA, -cosA * originX
									* scaleX - originY * scaleY * sinA + posX,
									cosA90 * scaleX, scaleY * sinA90, -cosA90
											* originX * scaleX
											- originY * scaleY * sinA90 + posY,
									0, 0, 1 } });

			return std::shared_ptr<const Matrix>(
					std::move(Matrix::createMatrixArray(array)));
		} else {
			const auto origin = (*(*collInfo).gMask()).origin();
			const double originX = origin.gX();
			const double originY = origin.gY();
			const auto pos = (*collInfo).gPosition();
			const double posX = pos.gX();
			const double posY = pos.gY();

			const auto array = boost::array<double, Matrix::dataSize>( { { 1.0,
					0.0, -originX + posX, 0.0, 1.0, -originY + posY, 0.0, 0.0,
					1.0 } });

			return std::shared_ptr<const Matrix>(
					std::move(Matrix::createMatrixArray(array)));
		}

		/*
		 * [ cosA*scaleX          scaleY*sinA       -cosA*originX*scaleX - originY*scaleY*sinA + posX     ]
		 * [ cosA90*scaleX        scaleY*sinA90     -cosA90*originX*scaleX - originY*scaleY*sinA90 + posY ]
		 * [ 0                    0                 1                                                     ]
		 */
	}

	/** Given a transformation matrix and an axis-aligned bounding box,
	 * find the axis-aligned bounding box of the transformed axis-aligned bounding box.
	 *
	 * @param transformationMatrix the transformation matrix
	 * @param boundingBox the axis-aligned bounding box
	 * @return the axis-aligned bounding box of the transformed given axis-aligned bounding box.
	 */
	static const BoundingBox approximateBoundingBox(
			const std::shared_ptr<const Matrix> transformationMatrix,
			const BoundingBox& boundingBox) {

		const auto pMin = boundingBox.pMin;
		const auto pMax = boundingBox.pMax;
		const auto p1 = pMin;
		const auto p2 = pMax;
		const auto p3 = P(pMin.gX(), pMax.gY());
		const auto p4 = P(pMax.gX(), pMin.gY());

		const auto approximateBoundingBoxPoints =
				(*transformationMatrix).transformPoints(std::vector<P>( { { p1,
						p2, p3, p4 } }));

		const auto getX = [](const P p) {return p.gX();};
		const auto getY = [](const P p) {return p.gY();};
		const auto xs =
				map<std::vector<P>, std::vector<double>, decltype(getX)>(
						*approximateBoundingBoxPoints, getX);
		const auto ys =
				map<std::vector<P>, std::vector<double>, decltype(getY)>(
						*approximateBoundingBoxPoints, getY);

		return BoundingBox(P(minDefault(*xs, 0.0), minDefault(*ys, 0.0)), //NOTE: Default is never used since never empty.
		P(maxDefault(*xs, 0.0), maxDefault(*ys, 0.0)) //NOTE: Default is never used since never empty.
				);
	}
};

}

#endif /* POXELCOLL_GEOMETRY_MATRIX_TRANSFORMATION_HPP_ */
