/* IntersectionFromCollisionSegments.hpp */

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

#ifndef POXELCOLL_GEOMETRY_CONVEXCCWPOLYGON_INTERSECTIONFROMCOLLISIONSEGMENTS_HPP_
#define POXELCOLL_GEOMETRY_CONVEXCCWPOLYGON_INTERSECTIONFROMCOLLISIONSEGMENTS_HPP_

#include <memory>
#include <vector>

#include "DataTypes.hpp"
#include "../../functional/IMList.hpp"
#include "../../functional/IMReverseList.hpp"

using namespace poxelcoll::functional;

namespace poxelcoll {

/** \ingroup poxelcollgeometryconvexccwpolygon
 *
 * Given a sequence of collision segments between two convex polygons in CCW-order, finds the intersection between the two polygons.
 *
 * This is done in linear time in the number of points of the polygons.
 *
 * This collision segments finder is part of a robust variation of the algorithm found here:
 * http://www-cgrl.cs.mcgill.ca/~godfried/teaching/cg-projects/97/Plante/CompGeomProject-EPlante/algorithm.html
 * This handles merging the previously found intersections of the first and second polygon.
 *
 * If the given sequence is empty, the intersection may be non-empty,
 * for instance if one of the polygons is fully inside the other.
 * To check for that, it is checked whether one of the polygons has
 * a point inside the other, and that polygon is then returned.
 * If no polygon is inside the other, the result is empty.
 *
 * If the given sequence is non-empty, the intersection is non-empty, and it is found
 * by following the collision segments in CCW-order.
 *
 * ==Status==
 *
 * The current implementation as of 0.1 is meant to be geometrically robust,
 * but gives no guarantees in regards to being numerically robust.
 * The consequences of the lack of numerical robustness is unknown,
 * but may range from imprecision to undefined behaviour.
 * The numerical robustness may be improved in the future.
 *
 * @param collisionSegments intersections between the two polygons, given in CCW-order
 * @param poly1Points points of the first convex CCW polygon
 * @param poly2Points points of the second convex CCW polygon
 */
class IntersectionFromCollisionSegments {

private:
	//Member variables.

	const std::shared_ptr<const std::vector<CollisionSegment>> collisionSegments;
	const std::shared_ptr<const std::vector<P>> poly1Points;
	const std::shared_ptr<const std::vector<P>> poly2Points;
	/** Number of points in polygon 1. */
	const int size1;
	/** Number of points in polygon 2. */
	const int size2;

public:
	//Constructor.

	IntersectionFromCollisionSegments(
			const std::shared_ptr<const std::vector<CollisionSegment>> aCollisionSegments,
			const std::shared_ptr<const std::vector<P>> aPoly1Points,
			const std::shared_ptr<const std::vector<P>> aPoly2Points);

private:
	//Private functions.

	/** Given an index in a polygon of a given size, find the circular next index.
	 *
	 * @param a valid index of a polygon
	 * @param size the number of points in the convex polygon
	 * @return the next circular index
	 */
	const int next(const int a, const int size) const;

	/** Given an index in a polygon of a given size, find the circular previous index.
	 *
	 * @param a valid index of a polygon
	 * @param size the number of points in the convex polygon
	 * @return the previous circular index
	 */
	const int prev(const int a, const int size) const;

	/** Find the next circular index for polygon 1.
	 *
	 * @param i valid index for polygon 1
	 * @return the next circular index
	 */
	const int next1(const int i) const;

	/** Find the next circular index for polygon 2.
	 *
	 * @param i valid index for polygon 2
	 * @return the next circular index
	 */
	const int next2(const int i) const;

	//If the same direction, or if one/both is zero-vector. Note that zero-vectors should never occur.
	/** Returns whether the given vectors are in the same direction.
	 *
	 * Zero-vectors are always in the same direction, but zero-vectors should never be given as arguments.
	 *
	 * @param v1 first vector
	 * @param v2 second vector
	 * @return whether in the same direction, assuming no zero-vectors.
	 */
	const bool sameDir(const P v1, const P v2) const;

	//If opposite direction. Zero-vectors are defined as not opposite. Note that zero-vectors should never occur.
	/** Returns whether the given vectors are in the opposite direction.
	 *
	 * Zero-vectors are never in the same direction, but zero-vectors should never be given as arguments.
	 *
	 * @param v1 first vector
	 * @param v2 second vector
	 * @return whether in the opposite direction, assuming no zero-vectors.
	 */
	const bool oppositeDir(const P v1, const P v2) const;

	/** Given a list of vectors, checks that the vectors are in clock-wise order.
	 *
	 * If given any zero-vectors, the vectors are not in clock-wise order.
	 *
	 * 'Clock-wise order' is defined as, for each vector, no vector are in the same
	 * direction as any other vector, and that the vector immediately cyclicly before it and
	 * immediately after it is the same in the list as if the vectors were put on a clock,
	 * and collected from one vector and clock-wise around.
	 *
	 * @param vs a list of vectors
	 * @return whether the vectors are in clock-wise order, as defined above.
	 */
	const bool cwOrder(std::shared_ptr<const std::list<P>> vs) const;

	//Should only be used on overlapping, in-same-direction line pieces, where the heads does not overlap.
	/** Given two directed line-segments returns whether the head of the first is ahead of the second.
	 *
	 * It is assumed that the directed line-segments collides, and that they are in the same direction.
	 *
	 * @param i1 index indicating the directed line-segment ]poly1(i1), poly1(next1(i1))]
	 * @param i2 index indicating the directed line-segment ]poly2(i2), poly2(next2(i2))]
	 * @return whether the head of the first directed line-segment is ahead of the head of the second directed line-segment.
	 */
	const bool ahead(const int i1, const int i2) const;

	/** Determines whether the given point is inside the polygon indicated by the given point sequence.
	 *
	 * @param points the point sequence representing the polygon
	 * @param nextFun a function that given an index into the sequence, gives the cyclically next index
	 * @param point the point to check for
	 * @return whether the point is inside the polygon
	 */
	template<class F>
	const bool pointInside(const std::shared_ptr<const std::vector<P>> points,
			const F & nextFun, const P point) const;

	/** Keep collecting points for the first polygon until there are no more points, or the next collision segment is reached and then continue constructing the intersection.
	 *
	 * @param collisionSegments the collision segments remaining
	 * @param res the resulting intersection so far
	 * @param i1 the current index of polygon 1 to investigate
	 * @param i2 the current index of polygon 2 to investigate
	 * @param lastSegment the segment that indicates at which point the construction of the result should stop. Is the original head of the collision segments
	 * @return the final intersection represented as a list of points
	 */
	const std::shared_ptr<const IMReverseList<const P>> F1(
			const std::shared_ptr<const IMList<const CollisionSegment>> collisionSegments,
			const std::shared_ptr<const IMReverseList<const P>> res,
			const int i1, const int i2,
			const std::shared_ptr<const CollisionSegment> lastSegmentNull) const;

	/** Keep collecting points for the second polygon until there are no more points, or the next collision segment is reached and then continue constructing the intersection.
	 *
	 * @param collisionSegments the collision segments remaining
	 * @param res the resulting intersection so far
	 * @param i1 the current index of polygon 1 to investigate
	 * @param i2 the current index of polygon 2 to investigate
	 * @param lastSegment the segment that indicates at which point the construction of the result should stop. Is the original head of the collision segments
	 * @return the final intersection represented as a list of points
	 */
	const std::shared_ptr<const IMReverseList<const P>> F2(
			const std::shared_ptr<const IMList<const CollisionSegment>> collisionSegments,
			const std::shared_ptr<const IMReverseList<const P>> res,
			const int i1, const int i2,
			const std::shared_ptr<const CollisionSegment> lastSegmentNull) const;

	/** Construct the intersection, by investigating the current collision segment and the relative configuration of the directed line-segments at it.
	 *
	 * @param collisionSegments the collision segments remaining
	 * @param res the resulting intersection so far
	 * @param lastSegment the segment that indicates at which point the construction of the result should stop. Is the original head of the collision segments
	 * @return the intersection
	 */
	const std::shared_ptr<const IMReverseList<const P>> constructIntersection(
			const std::shared_ptr<const IMList<const CollisionSegment>> collisionSegments,
			const std::shared_ptr<const IMReverseList<const P>> res,
			const std::shared_ptr<const CollisionSegment> lastSegmentNull) const;

public:
	//Public functions.

	std::shared_ptr<const ConvexCCWPolygon> getIntersectionFromCollisionSegments() const;
};

}

#endif /* POXELCOLL_GEOMETRY_CONVEXCCWPOLYGON_INTERSECTIONFROMCOLLISIONSEGMENTS_HPP_ */
