/* GeneralFunctions.hpp */

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

#ifndef POXELCOLL_GEOMETRY_CONVEXCCWPOLYGON_GENERALFUNCTIONS_HPP_
#define POXELCOLL_GEOMETRY_CONVEXCCWPOLYGON_GENERALFUNCTIONS_HPP_

#include <cmath>

#include "DataTypes.hpp"
#include "../../functional/Functional.hpp"

using namespace poxelcoll::functional;

namespace poxelcoll {

/** \ingroup poxelcollgeometryconvexccwpolygon
 *
 * General functions for handling intersection between different polygon primitives.
 */
class GeneralFunctions {

public:

	//This finds the collision between 2 line segments, if any collision exists.
	//The line segments are denoted by the two points
	//poly(i), poly(next(i)).
	//The line segments are directed.
	//This means that the first point, poly(i), counts as nothing in regards to collisions.
	//The second point, poly(next(i)), is called the "head".
	  /** Finds the collision between 2 directed line segments, if any exists.
	    *
	    *
	    * A directed line segment is defined as the vector consisting of the points {first, last},
	    * where first = poly*Points(i*) and last = poly*Points(next*(i*)),
	    * and where * is the number 1 or 2.
	    * The first point is not considered part of the directed line segment.
	    * Thus, if the directed line segments only overlap in one or two of the first points,
	    * there is no collision overall.
	    *
	    * If there is more than one collision, the overlapping last point(s) are used for the
	    * collision point.
	    *
	    * @param i1 index of the first point for the first polygon
	    * @param i2 index of the first point for the second polygon
	    * @param poly1Points the points of the first polygon
	    * @param poly2Points the points of the second polygon
	    * @return if the directed line segments indicated by the indices overlap, the corresponding collision segment, else an empty list
	    */
	static const std::shared_ptr<const std::vector<CollisionSegment>> getCollisionDirectedLineSegment(
			const int i1, const int i2,
			const std::shared_ptr<const std::vector<P>> poly1Points,
			const std::shared_ptr<const std::vector<P>> poly2Points) {

		const auto next = [](const int a, const int size) {
			return (a + 1) % size;
		};

		const auto size1 = (*poly1Points).size();
		const auto size2 = (*poly2Points).size();

		const auto next1 = [size1, next](const int i) {return next(i, size1);};
		const auto next2 = [size2, next](const int i) {return next(i, size2);};

		const auto p11 = (*poly1Points)[i1];
		const auto p12 = (*poly1Points)[next1(i1)];
		const auto p21 = (*poly2Points)[i2];
		const auto p22 = (*poly2Points)[next2(i2)];

		const auto l1 = p12.minus(p11);
		const auto l2 = p22.minus(p21);

		//If the lines not only crosses, but overlap, then only the heads.
		//Else, normal cross.
		//Remember that ends does not count.

		//Line segment intersection.
		//Treat the lines as vector lines.

		const auto denominator = l2.cross(l1); //l1.y * l2.x - l1.x * l2.y

		if (denominator == 0.0) {

			//No single point of intersection.

			//Finding the distance between a point of one line segment,
			//and the line of the other line segment.

			const auto perpenL2 = P(-l2.gY(), l2.gX()).divide(l2.norm());
			const auto v3 = p21.minus(p11);

			const auto dist = abs(perpenL2.dot(v3));

			if (dist == 0.0) {

				//Find relative position of heads.
				const auto u1Calc = [l1, p11, p22]() {
					if (l1.gX() != 0.0) {
						return (p22.gX() - p11.gX()) / l1.gX();
					}
					else {
						return (p22.gY() - p11.gY()) / l1.gY();
					}
				};
				const auto u1 = u1Calc(); //Position of head 2.

				const auto u2Calc = [l2, p12, p21]() {
					if (l2.gX() != 0.0) {
						return (p12.gX() - p21.gX()) / l2.gX();
					}
					else {
						return (p12.gY() - p21.gY()) / l2.gY();
					}
				};
				const auto u2 = u2Calc(); //Position of head 1.

				if ((u1 > 0.0 && u1 <= 1.0) || (u2 > 0.0 && u2 <= 1.0)) {

					//At least 1 heads overlap.
					const auto crossHeadPointCalc =
							[u1, p11, l1, p21, l2, u2]() {
								if (u1 > 0.0 && u1 <= 1.0) {return p11.plus(l1.multi(u1));}
								else {return p21.plus(l2.multi(u2));}
							};
					const auto crossHeadPoint = crossHeadPointCalc();

					const auto resVector = std::vector<CollisionSegment>( { {
							CollisionSegment(i1, i2, crossHeadPoint) } });
					return std::shared_ptr<const std::vector<CollisionSegment>>(
							new std::vector<CollisionSegment>(resVector));
				} else {
					return std::shared_ptr<const std::vector<CollisionSegment>>(
							new std::vector<CollisionSegment>());
				}
			} else {
				return std::shared_ptr<const std::vector<CollisionSegment>>(
						new std::vector<CollisionSegment>());
			}
		}
		else {

			const auto u1 = (-p21.gX() * l2.gY() + p11.gX() * l2.gY() + (p21.gY() - p11.gY()) * l2.gX()) / denominator;
			const auto u2 = (-p21.gX() * l1.gY() + p11.gX() * l1.gY() + (p21.gY() - p11.gY()) * l1.gX()) / denominator;

			if (u1 > 0.0 && u1 <= 1.0 && u2 > 0.0 && u2 <= 1.0) {
				return std::shared_ptr<const std::vector<CollisionSegment>>(
					new std::vector<CollisionSegment>({{
						CollisionSegment(
							i1, i2, p11.plus((l1.multi(u1)))
						)
					}})
				);
			}
			else {
				return std::shared_ptr<const std::vector<CollisionSegment>>(
						new std::vector<CollisionSegment>());
			}
		}
	}

	  /** Finds the intersection between two line segments.
	    *
	    * A(n) (undirected) line segment includes the first point, the last point,
	    * and all the points between them. The first point and the last point is
	    * never equal.
	    *
	    * @param p11 the first point of the first line segment, not equal p12
	    * @param p12 the last point of the first line segment, not equal p11
	    * @param p21 the first point of the second line segment, not equal p22
	    * @param p22 the last point of the second line segment, not equal p21
	    * @return the intersection of the line segments
	    */
	static const std::shared_ptr<const EmptyPointLine> handleLineLine(const P p11,
			const P p12, const P p21, const P p22) {

		const auto l1 = p12.minus(p11);
		const auto l2 = p22.minus(p21);

		//Line segment intersection.
		//Treat the lines as vector lines.

		const auto denominator = l2.cross(l1);

		if (denominator == 0.0) {
			//Possibly no single point of intersection.

			//Finding the distance between a point of one line segment,
			//and the line of the other line segment.

			const auto perpenL2 = P(-l2.gY(), l2.gX()).divide(l2.norm());
			const auto v3 = p21.minus(p11);

			const auto dist = abs(perpenL2.dot(v3));

			if (dist == 0.0) {

				//Find relative position of points.

				const auto calcU11U12 = [l1, p11, p21, p22]() {
					if (l1.gX() != 0.0) {
						return std::pair<double, double>(
								(p21.gX() - p11.gX()) / l1.gX(),
								(p22.gX() - p11.gX()) / l1.gX()
						);
					}
					else {
						return std::pair<double, double>(
								(p21.gY() - p11.gY()) / l1.gY(),
								(p22.gY() - p11.gY()) / l1.gY()
						);
					}
				};
				const auto u11u12 = calcU11U12(); //Position of l2-points on l1.
				const auto u11 = u11u12.first;
				const auto u12 = u11u12.second;

				const auto calcU21U22 = [l2, p11, p12, p21]() {
					if (l2.gX() != 0.0) {
						return std::pair<double, double>(
								(p11.gX() - p21.gX()) / l2.gX(),
								(p12.gX() - p21.gX()) / l2.gX()
						);
					}
					else {
						return std::pair<double, double>(
								(p11.gY() - p21.gY()) / l2.gY(),
								(p12.gY() - p21.gY()) / l2.gY()
						);
					}
				};
				const auto u21u22 = calcU21U22(); //Position of l1-points on l2.
				const auto u21 = u21u22.first;
				const auto u22 = u21u22.second;

				const auto pointsRelative = std::vector<std::pair<double, P>>(
						{ { std::pair<double, P>(u11, p21),
								std::pair<double, P>(u12, p22), std::pair<
										double, P>(u21, p11), std::pair<double,
										P>(u22, p12) } });
				const auto filterFunction =
						[](const std::pair<double, P> & partPoint) {
							return partPoint.first >= 0.0 && partPoint.first <= 1.0;
						};
				const auto overlappingPoints1 = filter(pointsRelative,
						filterFunction);
				const auto mappingFunction =
						[](const std::pair<double, P> & partPoint) {return partPoint.second;};
				const auto overlappingPoints2 = map<
						std::vector<std::pair<double, P>>, std::vector<P>,
						decltype(mappingFunction)>(*overlappingPoints1,
						mappingFunction);

				const auto overLappingPointsSize = (*overlappingPoints2).size();

				if (overLappingPointsSize == 2 || overLappingPointsSize == 3
						|| overLappingPointsSize == 4) {

					const auto sorterFunction = [](const P a, const P b) {
						const auto f = a.gX() - b.gX();
						if (f != 0.0) {return f > 0.0;}
						else {
							const auto f2 = a.gY() - b.gY();
							if (f2 != 0.0) {return f2 > 0.0;}
							else {return false;}
						}
					};
					const auto sortedOverlappingPoints = sortByToVector(
							*overlappingPoints2, sorterFunction);

					const auto first = (*sortedOverlappingPoints).front();
					const auto last = (*sortedOverlappingPoints).back();
					return Line::create(first, last);
				} else if (overLappingPointsSize == 1) {
					return std::shared_ptr<EmptyPoint>(
							new Point((*overlappingPoints2).front()));
				} else {
					return Empty::getEmpty();
				}
			} else {
				return Empty::getEmpty();
			}
		} else {

			const auto u1 = (-p21.gX() * l2.gY() + p11.gX() * l2.gY()
					+ (p21.gY() - p11.gY()) * l2.gX()) / denominator;
			const auto u2 = (-p21.gX() * l1.gY() + p11.gX() * l1.gY()
					+ (p21.gY() - p11.gY()) * l1.gX()) / denominator;

			if (u1 >= 0.0 && u1 <= 1.0 && u2 >= 0.0 && u2 <= 1.0) {
				return std::shared_ptr<EmptyPoint>(
						new Point(p11.plus(l1.multi(u1))));
			}
			else {
				return Empty::getEmpty();
			}
		}
	}

	  /** Finds the intersection between a point and a line segment.
	    *
	    * @param point a point
	    * @param line a line
	    * @return the intersection
	    */
	static const std::shared_ptr<const EmptyPoint> handlePointLine(const Point & point,
			const Line & line) {

		const auto p11 = line.myP1;
		const auto p12 = line.myP2;
		const auto p21 = point.myPoint;

		const auto v1 = p12.minus(p11);
		const auto v2 = p21.minus(p11);

		if ((v1.cross(v2)) == 0.0) {

			const auto uCalc = [v1, p11, p21]() {
				if (v1.gX() != 0.0) {
					return (p21.gX() - p11.gX() / v1.gX());
				}
				else {
					return (p21.gY() - p11.gY() / v1.gY());
				}
			};
			const auto u = uCalc();

			if (u >= 0.0 && u <= 1.0) {
				return std::shared_ptr<const EmptyPoint>(new Point(point));
			} else {
				return Empty::getEmpty();
			}
		} else {
			return Empty::getEmpty();
		}
	}
};

}

#endif /* POXELCOLL_GEOMETRY_CONVEXCCWPOLYGON_GENERALFUNCTIONS_HPP_ */
