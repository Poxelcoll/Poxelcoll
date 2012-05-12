/* PolygonIntersection.hpp*/

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

#ifndef POXELCOLL_GEOMETRY_CONVEXCCWPOLYGON_POLYGONINTERSECTION_HPP_
#define POXELCOLL_GEOMETRY_CONVEXCCWPOLYGON_POLYGONINTERSECTION_HPP_

#include "CollisionSegmentsFinder.hpp"
#include "IntersectionFromCollisionSegments.hpp"
#include "../../functional/IMList.hpp"
#include "../../functional/IMReverseList.hpp"
#include "../../functional/Either.hpp"

using namespace poxelcoll::functional;

namespace poxelcoll {

/** \ingroup poxelcollgeometryconvexccwpolygon
 *
 * Supports operations for finding the intersection between two polygons.
 *
 * The efficiency for finding the intersection is intended to be linear in the
 * size of the polygons points.
 *
 * ==Status==
 *
 * The current implementation as of 0.1 is meant to be geometrically robust,
 * but gives no guarantees in regards to being numerically robust.
 * The consequences of the lack of numerical robustness is unknown,
 * but may range from imprecision to undefined behaviour.
 * The numerical robustness may be improved in the future.
 */
class PolygonIntersection {

public:

private:

	/** Extracts the bounds from the given boundingBox option if present, else derives them from the given, non-empty points.
	 *
	 * @param polyPoints non-empty points
	 * @param boundingBox bounding box option
	 * @return the bounding box of the given polygon and bounding box option, or possibly undefined behaviour if points is empty
	 */
	static const std::pair<const P, const P> getBounds(
			const std::shared_ptr<const std::vector<P>> polyPoints,
			const std::shared_ptr<const std::pair<const P, const P>> boundingBoxNull) {

		if (boundingBoxNull.get() != 0) {
			return *boundingBoxNull;
		} else {

			//Ensure that the size of the polygons is >= 1.
			if ((*polyPoints).size() < 1) {
				std::cerr << "The size of the given polygons was less than one."
						<< std::endl;
				throw 1;
			}

			const auto getX = [](const P p) {
				return p.gX();
			};
			const auto getY = [](const P p) {
				return p.gY();
			};
			const auto polyPointsX = map<std::vector<P>, std::vector<double>,
					decltype(getX)>(*polyPoints, getX);
			const auto polyPointsY = map<std::vector<P>, std::vector<double>,
					decltype(getY)>(*polyPoints, getY);

			//Assuming that the size is >= 1.
			const auto pMinX = minDefault(*polyPointsX, 0.0);
			const auto pMinY = minDefault(*polyPointsY, 0.0);
			const auto pMaxX = maxDefault(*polyPointsX, 0.0);
			const auto pMaxY = maxDefault(*polyPointsY, 0.0);

			const auto pMin = P(pMinX, pMinY);
			const auto pMax = P(pMaxX, pMaxY);

			return std::pair<const P, const P>(pMin, pMax);
		}
	}

	/** A function that returns the leftmost point-index of the two, and if equally leftmost, the uppermost.
	 *
	 * Behaviour is undefined if the point-index pair has the same point. This should never happen.
	 *
	 * @return a function that finds the leftmost, upper point-index. The index corresponsd to the given point
	 */
	static const std::pair<const P, const int> chooseLeftmostUpperPoint(
			const std::pair<const P, const int> oldPointIndex,
			const std::pair<const P, const int> newPointIndex) {

		if (newPointIndex.first.gX() > oldPointIndex.first.gX()
				|| (newPointIndex.first.gX() == oldPointIndex.first.gX()
						&& newPointIndex.first.gY() < oldPointIndex.first.gY())) {
			return oldPointIndex;
		} else {
			return newPointIndex;
		}
	}

	/** Given a non-empty point sequence, find the bounding box.
	 *
	 * Behaviour is undefined if the point sequence is empty.
	 *
	 * @param polyPoints a non-empty point sequence
	 * @return bounding box of the non-empty point sequence, or undefined if empty
	 */
	static const BoundingBox bBoxNonemptyPolygon(
			const std::shared_ptr<const std::vector<P>> polyPoints) {

		//Ensure that the size of the polygons is >= 1.
		if ((*polyPoints).size() < 1) {
			std::cerr << "The size of the given polygons was less than one."
					<< std::endl;
			throw 1;
		}

		const auto getX = [](const P p) {
			return p.gX();
		};
		const auto getY = [](const P p) {
			return p.gY();
		};
		const auto polyPointsX = map<std::vector<P>, std::vector<double>,
				decltype(getX)>(*polyPoints, getX);
		const auto polyPointsY = map<std::vector<P>, std::vector<double>,
				decltype(getY)>(*polyPoints, getY);

		//Assuming that the size is >= 1.
		const auto pMinX = minDefault(*polyPointsX, 0.0);
		const auto pMinY = minDefault(*polyPointsY, 0.0);
		const auto pMaxX = maxDefault(*polyPointsX, 0.0);
		const auto pMaxY = maxDefault(*polyPointsY, 0.0);

		const auto pMin = P(pMinX, pMinY);
		const auto pMax = P(pMaxX, pMaxY);

		return BoundingBox(pMin, pMax);
	}

	/** Finds the intersection between a point and a polygon.
	 *
	 * @param point the point
	 * @param poly the polygon
	 * @return the intersection
	 */
	static const std::shared_ptr<const EmptyPoint> handlePointPolygon(
			const std::shared_ptr<const Point> point,
			const std::shared_ptr<const Polygon> poly) {

		auto polyPlusHeadVector = addAll(*(*poly).points(), std::vector<P>( { {
				(*poly).myP1 } }));
		const auto iden = [](const P & p) {return p;};
		const auto polyPlusHeadList = map<std::vector<P>, std::list<P>,
				decltype(iden)>(*polyPlusHeadVector, iden);

		const auto polyPlusHead = IMList<const P>::constructFrom(
				*polyPlusHeadList);

		typedef const std::shared_ptr<const IMList<const P>> GoThroughPolyArg;

		std::function<bool(GoThroughPolyArg)> goThroughPoly;
		goThroughPoly =
				[point, goThroughPoly](GoThroughPolyArg polys) -> bool {

					const auto size = (*polys).size();

					if (size >= 2) {

						const auto x1 = *( *polys ).headNull();
						const auto x2 = *( *(*polys).tailNull() ).headNull();
						const auto xs = ( *(*polys).tailNull() ).tailNull();

						const auto v1 = x2.minus(x1);
						const auto v2 = (*point).myPoint.minus(x1);

						return (v1.cross(v2) >= 0.0) && goThroughPoly(IMList<const P>::prepend(x2, xs));
					}
					else {
						return true;
					}
				};

		if (goThroughPoly(polyPlusHead)) {
			return point;
		} else {
			return Empty::getEmpty();
		}
	}

	/** Finds the intersection between a line and a polygon.
	 *
	 * @param line the line
	 * @param poly the polygon
	 * @return the intersection
	 */
	static Either<const bool, const ConvexCCWPolygon> handleLinePoly(
			const std::shared_ptr<const Line> line,
			const std::shared_ptr<const Polygon> poly) {

		typedef const std::shared_ptr<const EmptyPointLine> SHEmptyPointLine;

		typedef const std::shared_ptr<const IMList<const P>> IMListP;
		typedef const std::shared_ptr<const IMReverseList<SHEmptyPointLine>> IMReverseListEmptyPointLine;

		//Collide all line-segment pairs, and return the results.

		const auto p11 = (*line).myP1;
		const auto p12 = (*line).myP2;

		auto polygonListPlusHeadTemp = *(*poly).points();
		polygonListPlusHeadTemp.push_back(polygonListPlusHeadTemp.front()); //NOTE: Add head.
		const IMListP polygonListPlusHead = IMList<const P>::constructFrom(
				polygonListPlusHeadTemp);

		const std::function<
				IMReverseListEmptyPointLine(IMListP,
						IMReverseListEmptyPointLine)> collideAll =
				[p11, p12, &collideAll](
						IMListP polyList,
						IMReverseListEmptyPointLine res)
				-> IMReverseListEmptyPointLine {

					if ((*polyList).size() >= 2) {
						const auto x1 = *(*polyList).headNull();
						const auto x2 = *(*(*polyList).tailNull()).headNull();
						const auto xs = (*(*polyList).tailNull()).tailNull();

						const auto p21 = x1;
						const auto p22 = x2;

						const auto collisionResult = GeneralFunctions::handleLineLine(p11, p12, p21, p22);

						const auto collisionResultType = (*collisionResult).getType();

						if (collisionResultType == ConvexCCWType::EmptyT) {
							return collideAll(IMList<const P>::prepend(x2, (*xs).tailNull()), res);
						}
						else if (collisionResultType == ConvexCCWType::PointT) {
							const auto pRes = (*collisionResult).getAPoint();
							return collideAll(
									IMList<const P>::prepend(x2, (*xs).tailNull()),
									IMReverseList<const std::shared_ptr<const EmptyPointLine>>::append(res, pRes)
							);
						}
						else { //Line, since Polygon not possible.
							const auto lineRes = (*collisionResult).getALine();
							return IMReverseList<SHEmptyPointLine>::append(IMReverseList<SHEmptyPointLine>::nil(), lineRes);
						}
					}
					else {
						return res;
					}
				};

		const auto part1 = handlePointPolygon(
				std::shared_ptr<const Point>(new Point(p11)), poly);
		const auto part2 = handlePointPolygon(
				std::shared_ptr<const Point>(new Point(p12)), poly);

		const auto insideEnds =
				IMReverseList<const std::shared_ptr<const EmptyPointLine>>::append(
						IMReverseList<
								const std::shared_ptr<const EmptyPointLine>>::create(
								part1), part2);

		const IMReverseListEmptyPointLine collisions =
				collideAll(polygonListPlusHead,
						IMReverseList<
								const std::shared_ptr<const EmptyPointLine>>::nil());

		const auto finalResultCalc =
				[collisions, insideEnds]() -> std::shared_ptr<const EmptyPointLine> {

					const auto size = (*collisions).size();

					if (size == 1 && (*(*(*collisions).lastNull())).getType() == ConvexCCWType::LineT) {
						const auto a = *(*collisions).lastNull();
						return a;
					}
					else {

						//Make a set out of the points, thereby removing duplicates.

						const auto temp1 = IMReverseList<SHEmptyPointLine>::constructTo<std::list<std::shared_ptr<const EmptyPointLine>>>(collisions);
						const auto temp2 = IMReverseList<SHEmptyPointLine>::constructTo<std::list<std::shared_ptr<const EmptyPointLine>>>(insideEnds);
						const auto finalFinalRes1 = addAll(*temp1, *temp2);

						const auto filterPoints = [](const std::shared_ptr<const EmptyPointLine> poly) {
							const auto type = (*poly).getType();
							return type == ConvexCCWType::PointT;
						};
						const auto finalFinalResOnlyPoints = filter(*finalFinalRes1, filterPoints);

						const auto toPoint = [](const std::shared_ptr<const EmptyPointLine> point) {
							return (*(*point).getAPoint()).myPoint;
						};
						const auto finalFinalResPs = map<std::list<std::shared_ptr<const EmptyPointLine>>, std::list<P>, decltype(toPoint)>
						(*finalFinalResOnlyPoints, toPoint);

						const auto idenP = [](const P p) {return p;};
						const auto finalFinalResSet = map<std::list<P>, std::set<P, P::Comparer>, decltype(idenP)>(*finalFinalResPs, idenP);

						const auto setSize = (*finalFinalResSet).size();
						if (setSize == 0) {
							return Empty::getEmpty();
						}
						else if (setSize == 1) {
							return std::shared_ptr<const Point>(new Point(*(*finalFinalResSet).begin()));
						}
						else { //Take the two first.
							return Line::create(
									*(*finalFinalResSet).begin(),
									*(++(*finalFinalResSet).begin())
							);
						}
					}
				};
		const auto finalResult = finalResultCalc();

		return Either<const bool, const ConvexCCWPolygon>::createRight(
				finalResult);
	}

public:

	/** Finds the intersection between two polygons, that may be full or not-full.
	 *
	 * @param poly1 the first polygon
	 * @param poly2 the second polygon
	 * @param poly1Full whether the first polygon is full
	 * @param poly2Full whether the second polygon is full
	 * @param poly1ApproxBoundingBox the optional bounding box of the first polygon, to avoid possible recalculation
	 * @param poly2ApproxBoundingBox the optional bounding box of the second polygon, to avoid possible recalculation
	 * @return the intersection of the polygons
	 */
	static const Either<const bool, const ConvexCCWPolygon> intersection(
			const std::shared_ptr<const ConvexCCWPolygon> poly1,
			const std::shared_ptr<const ConvexCCWPolygon> poly2,
			const bool poly1Full, const bool poly2Full,
			const std::shared_ptr<const BoundingBox> poly1ApproxBoundingBoxNull =
					std::shared_ptr<const BoundingBox>(0),
			const std::shared_ptr<const BoundingBox> poly2ApproxBoundingBoxNull =
					std::shared_ptr<const BoundingBox>(0)) {

			const
		auto poly1Points = (*poly1).points();
		const auto poly2Points = (*poly2).points();

		const auto boundingBoxesIntersectCalc =
				[poly1, poly2, poly1Points, poly2Points, poly1ApproxBoundingBoxNull, poly2ApproxBoundingBoxNull]() {

					//Ensure that neither of the polygons are empty.
					if ((*poly1).getType() == ConvexCCWType::EmptyT || (*poly2).getType() == ConvexCCWType::EmptyT) {
						return false;
					}
					else {
						//Match with the approximate bounding box if existing, and if not or no approximate, check actual bounding box.

						const auto poly1ApproxBoundingBoxNullIsNull = poly1ApproxBoundingBoxNull.get() == 0;
						const auto poly2ApproxBoundingBoxNullIsNull = poly2ApproxBoundingBoxNull.get() == 0;

						if (poly1ApproxBoundingBoxNullIsNull && poly2ApproxBoundingBoxNullIsNull) {
							return bBoxNonemptyPolygon(poly1Points).intersects(bBoxNonemptyPolygon(poly2Points));
						}
						else if (!poly1ApproxBoundingBoxNullIsNull) {
							const auto approx1 = *poly1ApproxBoundingBoxNull;
							const auto boundingBox2 = bBoxNonemptyPolygon(poly2Points);
							return
							(approx1.intersects(boundingBox2)) &&
							(bBoxNonemptyPolygon(poly1Points).intersects(boundingBox2));
						}
						else if (!poly2ApproxBoundingBoxNullIsNull) {
							const auto approx2 = *poly2ApproxBoundingBoxNull;
							const auto boundingBox1 = bBoxNonemptyPolygon(poly1Points);
							return
							(boundingBox1.intersects(approx2)) &&
							(boundingBox1.intersects(bBoxNonemptyPolygon(poly2Points)));
						}
						else {
							const auto approx1 = *poly1ApproxBoundingBoxNull;
							const auto approx2 = *poly2ApproxBoundingBoxNull;
							return
							(approx1.intersects(approx2)) &&
							(bBoxNonemptyPolygon(poly1Points).intersects(bBoxNonemptyPolygon(poly2Points)));
						}
					}
				};
		const auto boundingBoxesIntersect = boundingBoxesIntersectCalc();

		if (!boundingBoxesIntersect) {
			return Either<const bool, const ConvexCCWPolygon>::createLeft(
					std::shared_ptr<const bool>(new bool(false)));
		} else {

			const auto poly1Type = (*poly1).getType();
			const auto poly2Type = (*poly2).getType();

			if (poly1Type == ConvexCCWType::PolygonT
					&& poly2Type == ConvexCCWType::PolygonT) {

				const auto originIndex1OriginIndex2Calc =
						[poly1Points, poly2Points]() {

							typedef std::vector<P>::size_type size_type_vec;
							const auto range1 = until<size_type_vec>(0, (*poly1Points).size(), 1);
							const auto range2 = until<size_type_vec>(0, (*poly2Points).size(), 1);

							const auto zipped1 = zip<decltype(*poly1Points), decltype(*range1), std::pair<P, size_type_vec>>(*poly1Points, *range1);
							const auto zipped2 = zip<decltype(*poly2Points), decltype(*range2), std::pair<P, size_type_vec>>(*poly2Points, *range2);

							const auto folded1 = foldLeft(*zipped1, std::pair<P, size_type_vec>((*poly1Points).front(), 0), chooseLeftmostUpperPoint);
							const auto folded2 = foldLeft(*zipped2, std::pair<P, size_type_vec>((*poly2Points).front(), 0), chooseLeftmostUpperPoint);

							const auto originIndex1 = (*folded1).second;
							const auto originIndex2 = (*folded2).second;

							return std::pair<int, int>(originIndex1, originIndex2);
						};
				const auto originIndex1OriginIndex2 =
						originIndex1OriginIndex2Calc();
				const auto originIndex1 = originIndex1OriginIndex2.first;
				const auto originIndex2 = originIndex1OriginIndex2.second;

				const auto collisionSegmentsFinder = CollisionSegmentsFinder(
						poly1Points, poly2Points, originIndex1, originIndex2);

				const auto collisionSegmentsNull =
						collisionSegmentsFinder.getCollisionSegmentsNull(); //NOTE: Handle potential null.

				const auto intersectionConstructionResultCalc =
						[collisionSegmentsNull, poly1Points, poly2Points]
						() -> std::shared_ptr<const ConvexCCWPolygon> {

							if (collisionSegmentsNull.get() == 0) { //NOTE: Handling case null.
								//No need to check for one polygon inside the other,
								//since none means that there is no polygon intersection at all.
								return Empty::getEmpty();
							}
							else { //NOTE: Handling case non-null.

								const auto intersectionFinder = IntersectionFromCollisionSegments(collisionSegmentsNull, poly1Points, poly2Points);
								const auto result = intersectionFinder.getIntersectionFromCollisionSegments();
								return result;
							}
						};
				const auto intersectionConstructionResult =
						intersectionConstructionResultCalc();

				return Either<const bool, const ConvexCCWPolygon>::createRight(
						intersectionConstructionResult);

			} else if (poly1Type == ConvexCCWType::LineT
					&& poly2Type == ConvexCCWType::PolygonT) {

				const auto line = (*poly1).getALine();
				const auto poly = (*poly2).getAPolygon();

				return handleLinePoly(line, poly);

			} else if (poly1Type == ConvexCCWType::PolygonT
					&& poly2Type == ConvexCCWType::LineT) {

				const auto poly = (*poly1).getAPolygon();
				const auto line = (*poly2).getALine();

				return handleLinePoly(line, poly);

			} else if (poly1Type == ConvexCCWType::PointT
					&& poly2Type == ConvexCCWType::PolygonT) {

				const auto point = (*poly1).getAPoint();
				const auto poly = (*poly2).getAPolygon();

				return Either<const bool, const ConvexCCWPolygon>::createRight(
						handlePointPolygon(point, poly));

			} else if (poly1Type == ConvexCCWType::PolygonT
					&& poly2Type == ConvexCCWType::PointT) {

				const auto poly = (*poly1).getAPolygon();
				const auto point = (*poly2).getAPoint();

				return Either<const bool, const ConvexCCWPolygon>::createRight(
						handlePointPolygon(point, poly));

			} else if (poly1Type == ConvexCCWType::LineT
					&& poly2Type == ConvexCCWType::LineT) {

				const auto line1 = (*poly1).getALine();
				const auto line2 = (*poly2).getALine();

				const auto p11 = (*line1).myP1;
				const auto p12 = (*line1).myP2;
				const auto p21 = (*line2).myP1;
				const auto p22 = (*line2).myP2;

				return Either<const bool, const ConvexCCWPolygon>::createRight(
						GeneralFunctions::handleLineLine(p11, p12, p21, p22));

			} else if (poly1Type == ConvexCCWType::LineT
					&& poly2Type == ConvexCCWType::PointT) {

				const auto line = (*poly1).getALine();
				const auto point = (*poly2).getAPoint();

				return Either<const bool, const ConvexCCWPolygon>::createRight(
						GeneralFunctions::handlePointLine(*point, *line));

			} else if (poly1Type == ConvexCCWType::PointT
					&& poly2Type == ConvexCCWType::LineT) {

				const auto point = (*poly1).getAPoint();
				const auto line = (*poly2).getALine();

				return Either<const bool, const ConvexCCWPolygon>::createRight(
						GeneralFunctions::handlePointLine(*point, *line));

			} else if (poly1Type == ConvexCCWType::PointT
					&& poly2Type == ConvexCCWType::PointT) {

				const auto a = (*(*poly1).getAPoint()).myPoint;
				const auto b = (*(*poly2).getAPoint()).myPoint;

				if (a.equal(b)) {
					return Either<const bool, const ConvexCCWPolygon>::createRight(
							poly1);
				} else {
					return Either<const bool, const ConvexCCWPolygon>::createRight(
							Empty::getEmpty());
				}

			} else { //One of the arguments must be empty at this point.

				return Either<const bool, const ConvexCCWPolygon>::createRight(
						Empty::getEmpty());
			}
		}
	}
};

}

#endif /* POXELCOLL_GEOMETRY_CONVEXCCWPOLYGON_POLYGONINTERSECTION_HPP_ */
