/* SimplePixelPerfectPairwise.hpp */

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

#ifndef SIMPLEPIXELPERFECTPAIRWISE_HPP_
#define SIMPLEPIXELPERFECTPAIRWISE_HPP_

#include <memory>
#include <vector>

#include "Pairwise.hpp"
#include "../../DataTypes.hpp"
#include "../../CollisionInfo.hpp"
#include "../../mask/Mask.hpp"
#include "../../geometry/convexccwpolygon/DataTypes.hpp"
#include "../../geometry/matrix/Matrix.hpp"

namespace poxelcoll {

/** \ingroup poxelcollcollisionpairwise
  *
  * The simple pairwise collision detection takes 2 pairs and determines if they collide.
  *
  * The method takes everything into account, including transformation (rotation, translation, scaling),
  * binary images, filled objects, etc.
  *
  * Strictly over-approximating bounding boxes are used to speed up collision detection
  * by excluding collision objects that do not overlap.
  *
  * '''Method'''
  *
  * The implementation first checks the approximate bounding box found by
  * transforming the bounding box according to the collision objects transformation data,
  * and then finding the axis-aligned bounding box of the transformed bounding box.
  * This is efficient, but not very precise.
  * If they still collide, the detection goes on, else it stops with false.
  * Then the convex hulls of the collision objects is transformed in linear time of the points on the hulls themselves.
  * The intersection of the convex hulls are then found, again in linear time of the points on the hulls themselves.
  *
  * If the intersection is found to be empty, the objects do not collide.
  * Else, all the points that overlaps the intersection is found:
  * Now, for each of these points, the point is transformed back to each of the
  * coordinate systems of the original collision objects, and the binary
  * images of each object is checked. If both are filled, a collision is
  * decided to have occurred, and the algorithm stops with true.
  * If this collision test fails for all points overlapping with the intersection,
  * it is decided that there is no collision.
  *
  * In general, the above method stops as soon as a colliding pixel has been found.
  * Furthermore, if both of the collision objects are filled (ie. they have no binary image),
  * the method stops the moment it has been decided whether or not there is an intersection.
  *
  * This method is generally very performant if the collision objects (including their
  * binary images) are well approximated by their convex hulls.
  * For instance, two triangles that are transformed and scale will generally
  * be checked very quickly, because either their convex hulls will find no
  * intersection quickly (meaning there is no intersection, and thus no need to go on),
  * or else the intersection will consist of on-pixels, and the pixel-perfect detection
  * will stop quickly.
  * Conversely, if collision objects are not well approximated by their convex hulls,
  * collision detection will generally not be so performant, since the convex hulls
  * may have a large intersection where there is few or no on-pixels.
  *
  * '''Pixel-perfect collision detection, precision and scaling'''
  *
  * For the pixel-perfect collision detection,
  * a scaling-invariant method is used. This means that collision detection will generally
  * only be precise and performant as long as the collision objects are not scaled.
  * If the objects are scaled up, they will not loose precision, but they may become less
  * performant. Conversely, if the objects are scaled down, they will loose precision,
  * and may become more performant. The performance and precision of mixed scaling
  * (such as one object scaling up and the other scaling down,
  * or scaling up along one axis and down along the other)
  * is generally difficult to predict, but can be assumed in general to be less
  * performant and precise than if no scaling is present.
  */
class SimplePixelPerfectPairwise : public virtual Pairwise {

private:

	  /** Given a set of points that form a valid convex polygon, that is either clockwise or counter-clockwise,
	    * return a counter-clockwise convex polygon.
	    *
	    * @param points CW or CCW convex points
	    * @return CCW convex polygon
	    */
	static const std::shared_ptr<const ConvexCCWPolygon> assumingValidConvexPolygonPointsTransformToCCWEvenIfCW(
			const std::shared_ptr<const std::vector<P>> points
				) {

		const auto size = (*points).size();

		if (size == 0) {
			return Empty::getEmpty();
		}
		else if (size == 1) {
			return std::shared_ptr<const ConvexCCWPolygon>(new Point((*points).front()));
		}
		else if (size == 2) {
			const auto head = (*points).front();
			const auto last = (*points).back();
			return Line::create(head, last);
		}
		else { //size >= 3.

			//Get the first 3 points, and check their direction.

			const auto p1 = (*points).front();
			const auto p2 = *(++(*points).begin());
			const auto p3 = *(++++(*points).begin());

			const auto v1 = p2.minus(p1);
			const auto v2 = p3.minus(p1);

			const auto v1XV2 = v1.cross(v2);

			if (v1XV2 == 0) {
				std::cerr << "A valid convex polygon will never have 3 points on the same line in the convex hull." << std::endl;
				throw 1;
			}
			else if (v1XV2 > 0) {

				//The polygon is CCW, do nothing.
				return Polygon::createUtterlyUnsafelyNotChecked(points);
			}
			else { // v1 X v2 < 0.0.

				//The polygon is CW, reverse in other to get CCW.

				const auto pointsReverse = new std::vector<P>(*points);
				std::reverse(pointsReverse->begin(), pointsReverse->end());

				return Polygon::createUtterlyUnsafelyNotChecked(std::shared_ptr<const std::vector<P>>(pointsReverse));
			}
		}
	}

	  /** Check whether a point is contained in a binary image.
	    *
	    * @param image a binary image
	    * @param v a point which has a superfluous third coordinate, and which coordinates may be outside the images dimension
	    * @return whether the image contains the point
	    */
	static const bool checkImage(const std::shared_ptr<const Mask> image, const P3 v) {

		const auto binaryImageNull = (*image).binaryImageNull(); //NOTE: Handle potential null.

		if (binaryImageNull.get() == 0) {//binaryImageNull is null.
			return true;
		}
		else { //binaryImageNull is not null.

			const auto point = IP((int)round(v.gX()), (int)round(v.gY()));
			const auto x = point.gX();
			const auto y = point.gY();

			return x >= 0 && x < (int)(*binaryImageNull).width() &&
				   y >= 0 && y < (int)(*binaryImageNull).height() &&
				   (*binaryImageNull).hasPoint(x, y);
		}
	}

	  /** For 2 images and 2 transformation matrices, give a function that test whether a given point is contained in both images.
	    *
	    * The transformation matrices is used to map from some coordinate system to that of the images.
	    *
	    * @param image1 first binary image to test whether a point is contained in
	    * @param image2 second binary image to test whether a point is contained in
	    * @param inv1 transform given points to the coordinate system of the first image
	    * @param inv2 transform given points to the coordinate system of the second image
	    * @param point whether a point is contained in the given function
	    * @return first currying yields a test function that yields if a point is contained in both images using the given transformation matrices
	    */
	static const std::function<bool(IP)> generalTestFunction(const std::shared_ptr<const Mask> image1, const std::shared_ptr<const Mask> image2,
			const std::shared_ptr<const Matrix> inv1, const std::shared_ptr<const Matrix> inv2) {

		const auto fun = [inv1, inv2, image1, image2](const IP point){

			const auto vector = P3(point.gX(), point.gY(), 1.0);

			const auto imageVector1 = (*inv1).vectorMult(vector);
			const auto imageVector2 = (*inv2).vectorMult(vector);

			return checkImage(image1, imageVector1) && checkImage(image2, imageVector2);
		};

		return fun;
	}

public:

	const bool testForCollision(
			const std::shared_ptr<const CollisionInfo> collInfo1,
			const std::shared_ptr<const CollisionInfo> collInfo2) const;
};

}

#endif /* SIMPLEPIXELPERFECTPAIRWISE_HPP_ */
