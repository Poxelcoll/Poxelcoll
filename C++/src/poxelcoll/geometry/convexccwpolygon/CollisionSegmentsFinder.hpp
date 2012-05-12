/* CollisionSegmentsFinder.hpp */

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

#ifndef POXELCOLL_GEOMETRY_CONVEXCCWPOLYGON_COLLISIONSEGMENTSFINDER_HPP_
#define POXELCOLL_GEOMETRY_CONVEXCCWPOLYGON_COLLISIONSEGMENTSFINDER_HPP_

#include <memory>
#include <vector>

#include "../../DataTypes.hpp"
#include "GeneralFunctions.hpp"
#include "DataTypes.hpp"

namespace poxelcoll {

/** \ingroup poxelcollgeometryconvexccwpolygon
 *
 * The dir trait stands for direction, and has three values.
 *
 * The direction is used in the rotating callipers method in CollisionSegmentsFinder.
 * For a given pair of callipers, the direction is the relative position of the
 * second calliper to the first.
 * For instance, if the second calliper is to the right (as seen from the counter-clockwise (CCW) view),
 * then the direction is RightDir.
 */
enum class Dir {
	/** The left direction. */
	LeftDir,
	/** The right direction. */
	RightDir,
	/** The same direction. */
	SameDir
};

/** \ingroup poxelcollgeometryconvexccwpolygon
 *
 * The collision segments finder finds all the collision segments between two convex polygons in CCW-order.
 *
 * This is done in linear time in the number of points of the polygons.
 * The general method used is rotating callipers.
 *
 * This collision segments finder is part of a robust variation of the algorithm found here:
 * http://www-cgrl.cs.mcgill.ca/~godfried/teaching/cg-projects/97/Plante/CompGeomProject-EPlante/algorithm.html
 * This handles finding all the intersections of the first and second polygon, including but not limited to
 * those intersections that lies in pockets.
 *
 * The collision segments are found in CCW-order, meaning that the segments collisions follow the intersection.
 *
 * Note the definition of a collision segment: for a given index in one of the counter-clockwise (CCW) convex polygons,
 * form a line segment from the current point to the next point, and include all points on the line segment except
 * the current point. Call this the directed line segment. For any given two indices in two polygons,
 * these two indices collide if and only if their directed line segments overlap.
 *
 * ==Status==
 *
 * The current implementation as of 0.1 is meant to be geometrically robust,
 * but gives no guarantees in regards to being numerically robust.
 * The consequences of the lack of numerical robustness is unknown,
 * but may range from imprecision to undefined behaviour.
 * The numerical robustness may be improved in the future.
 *
 * @param poly1Points points of the first convex CCW polygon
 * @param poly2Points points of the second convex CCW polygon
 * @param originIndex1 origin inex of the first polygon
 * @param originIndex2 origin inex of the second polygon
 */
class CollisionSegmentsFinder {

private:
	const std::shared_ptr<const std::vector<P>> poly1Points;
	const std::shared_ptr<const std::vector<P>> poly2Points;
	const int originIndex1;
	const int originIndex2;

	/** Number of points in polygon 1. */
	const std::vector<P>::size_type size1;
//	/** Number of points in polygon 2. */
	const std::vector<P>::size_type size2;

public:
	CollisionSegmentsFinder(
			const std::shared_ptr<const std::vector<P>> aPoly1Points,
			const std::shared_ptr<const std::vector<P>> aPoly2Points,
			const int aOriginIndex1, const int aOriginIndex2);

public:

private:

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

	/** Assuming that the indices fits with rotating callipers, find the next rotating callipers indices.
	 *
	 * @param i1 index in polygon 1
	 * @param i2 index in polygon 2
	 * @param currentDir the current direction
	 * @return the next indices when moving the callipers one step
	 */
	//Find the coming index when finding the collision segments.
	const std::pair<int, int> findComingIndex(const int i1, const int i2,
			const Dir currentDir) const;

	/** The cross left finder is used to find crosses, assuming that the second polygon was previously to the left.
	 *
	 * If the second polygon was not previously to the left, the behaviour is undefined.
	 *
	 * A cross is understood as the cross found when using rotating callipers, as described here:
	 * http://www-cgrl.cs.mcgill.ca/~godfried/teaching/cg-projects/97/Plante/CompGeomProject-EPlante/algorithm.html
	 *
	 * @param startIndex1 the start index for polygon 1 after cross has been detected
	 * @param startIndex2 the start index for polygon 2 after cross has been detected
	 * @param s1 the size of polygon 1
	 * @param s2 the size of polygon 2
	 * @param p1Points points of polygon 1
	 * @param p2Points points of polygon 2
	 * @param getColli function that given 2 indices, returns the collision segments for the directed line segments at those indices
	 */
	template<class F>
	class CrossLeftFinder {
	private:
		const int startIndex1;
		const int startIndex2;
		const int s1;
		const int s2;
		const std::shared_ptr<const std::vector<P>> p1Points;
		const std::shared_ptr<const std::vector<P>> p2Points;
		const F getColli;

	public:
		CrossLeftFinder(

		const int aStartIndex1, const int aStartIndex2, const int aS1,
				const int aS2,
				const std::shared_ptr<const std::vector<P>> aP1Points,
				const std::shared_ptr<const std::vector<P>> aP2Points,
				const F aGetColli) :
				startIndex1(aStartIndex1), startIndex2(aStartIndex2), s1(aS1), s2(
						aS2), p1Points(aP1Points), p2Points(aP2Points), getColli(
						aGetColli) {
		}

	public:
		/** Find the cross, assuming that the given argument polygon 2 was previously to the left of polygon 1.
		 *
		 * @return the cross indicated by the start indices, or nothing if there is no cross
		 */
		std::shared_ptr<const std::vector<CollisionSegment>> getCrossLeftNull() const {

			return getCrossLeftInner(startIndex1, startIndex2);
		}
		;
	private:

		/** Given an index in a polygon of a given size, find the circular next index.
		 *
		 * @param a valid index of a polygon
		 * @param size the number of points in the convex polygon
		 * @return the next circular index
		 */
		const int next(const int a, const int size) const {
			return (a + 1) % size;
		}

		/** Given an index in a polygon of a given size, find the circular previous index.
		 *
		 * @param a valid index of a polygon
		 * @param size the number of points in the convex polygon
		 * @return the previous circular index
		 */
		const int prev(const int a, const int size) const {
			return (a - 1 < 0) ? size - 1 : a - 1;
		}

		/** Given indices, step towards the cross, until the cross is found or it is detected that there is no cross.
		 *
		 * @param i1 the index for the first polygon
		 * @param i2 the index for the second polygon
		 * @return Some collision segments if cross found, or None
		 */
		std::shared_ptr<const std::vector<CollisionSegment>> getCrossLeftInner(
				const int i1, const int i2) const {

			//Go as far as possible, and then check.
			const auto p11 = (*p1Points)[i1];
			const auto p12 = (*p1Points)[next(i1, s1)];
			const auto p21 = (*p2Points)[i2];
			const auto p22 = (*p2Points)[prev(i2, s2)]; //NOTE: Going backwards.

			const auto v1 = p12.minus(p11);
			const auto v2 = p22.minus(p21);

			if ((v1.cross(v2)) >= 0.0) {
				//Try to move along polygon 1.
				const auto v21 = p12.minus(p21);

				if ((v2.cross(v21)) > 0.0) {
					return getCrossLeftInner(next(i1, s1), i2);
				} else {

					const auto v12 = p22.minus(p11);

					if ((v1.cross(v12)) < 0.0) {
						return getCrossLeftInner(i1, prev(i2, s2)); //NOTE: Going backwards.
					} else {
						//Test for collisions!

						//Go back once, and if something, return it.

						const auto i22 = prev(i2, s2); //NOTE: Going backwards.
						const auto collisionSegments = getColli(i1, i22);

						if ((*collisionSegments).empty()) {

							const auto i23 = prev(i22, s2);
							const auto collisionSegments2 = getColli(i1, i23);
							if ((*collisionSegments2).empty()) {
								return std::shared_ptr<
										const std::vector<CollisionSegment>>(0);
							} else {
								return collisionSegments2;
							}
						} else {
							return collisionSegments;
						}
					}
				}
			} else {
				return std::shared_ptr<const std::vector<CollisionSegment>>(0);
			}
		}
	};

	/** Given a position with a potential cross (ie. when the direction between the polygons change), find the cross.
	 *
	 * An example of changing directions is when the second polygon goes from being on the left to being on the right.
	 * Note that if there is no collision segments in a cross, there are no collision segments whatsoever,
	 * and there is thus no intersecting polygon at all.
	 *
	 * @param i1 index for polygon 1
	 * @param i2 index for polygon 2
	 * @param prevDir the previous direction the second polygon was relative to the first one
	 * @param currentDir the current direction the second polygon is relative to the first one
	 * @return the collision segments of the cross, or None if no intersection at all
	 */
	const std::shared_ptr<const std::vector<CollisionSegment>> getCrossNull(
			const int i1, const int i2, const Dir prevDir,
			const Dir currentDir) const;

	/** Find the direction of the second polygon relative to the first polygon.
	 *
	 * "direction of the second polygon relative to the first polygon"
	 * is defined from the callipers of the convex counter-clockwise (CCW) polygons.
	 * It is assumed that the indices are valid in regards to the callipers.
	 *
	 * The two callipers are defined as one line for each polygon.
	 * They go in the counter-clockwise direction, to fit with the
	 * definition of the polygons.
	 * See http://en.wikipedia.org/wiki/Rotating_calipers or google for
	 * "rotating callipers intersection" for more information on callipers and intersection.
	 *
	 * @param i1 index of polygon 1
	 * @param i2 index of polygon 2
	 * @return the direction of the second calliper relative to the first
	 */
	const Dir findDir(const int i1, const int i2) const;

	/** Go through the polygons, and find all collision segments. None may be returned if there is no collision segments at all.
	 *
	 * @param i1 the index of the calliper of the first polygon at this point
	 * @param i2 the index of the calliper of the second polygon at this point
	 * @param previousDirO the previous direction of the callipers
	 * @param prevRes the collision segments found so far
	 * @return all the collision segments between the convex polygons, if any
	 */
	const std::shared_ptr<const std::vector<CollisionSegment>> findAllCollisionSegmentsNull(
			const int i1, const int i2,
			std::shared_ptr<const Dir> previousDirNull,
			std::shared_ptr<const std::vector<CollisionSegment>> prevRes) const;

public:

	/** Find all the collision segments between the two convex polygons in CCW-order.
	 *
	 * @return the collision segments between the two convex polygons, if any.
	 *         If there are no collision segments (ie. empty collection),
	 *         it means the intersection is either empty, or one is strictly inside the other.
	 *         If none is returned, there are no intersection at all
	 */
	std::shared_ptr<const std::vector<CollisionSegment>> getCollisionSegmentsNull() const;
};

}

#endif /* POXELCOLL_GEOMETRY_CONVEXCCWPOLYGON_COLLISIONSEGMENTSFINDER_HPP_ */
