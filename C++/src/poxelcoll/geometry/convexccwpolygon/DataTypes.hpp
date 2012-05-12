/* DataTypes.hpp */

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

#ifndef POXELCOLL_GEOMETRY_CONVEXCCWPOLYGON_DATATYPES_HPP_
#define POXELCOLL_GEOMETRY_CONVEXCCWPOLYGON_DATATYPES_HPP_

#include <memory>
#include <sstream>

#include "../../DataTypes.hpp"
#include "../../functional/Functional.hpp"
#include "../../functional/IMList.hpp"
#include "../../functional/IMReverseList.hpp"

using namespace poxelcoll::functional;

namespace poxelcoll {

enum class ConvexCCWType {
	EmptyT,
	PointT,
	LineT,
	PolygonT
};

class Empty;
class Point;
class Line;
class Polygon;

/** \ingroup poxelcollgeometryconvexccwpolygon
 *
 * This is a simple ("simple" as in the trait "simple" for polygons),
 * convex, counter-clockwise polygon meant to represent a convex hull.
 *
 * There is no duplicated points, nor any collinearity.
 *
 * The polygon may be empty.
 */
class ConvexCCWPolygon {

public:
	virtual ~ConvexCCWPolygon() {
	}

	/** The points as an indexed sequence. */
	virtual const std::shared_ptr<const std::vector<P>> points() const = 0;

	/** Translate the points by a vector represented as a point.
	 *
	 * @param the vector to translate with
	 * @return the translated polygon
	 */
	virtual const std::shared_ptr<const ConvexCCWPolygon> translate(
			P p) const = 0;

	virtual const ConvexCCWType getType() const = 0;

	virtual const std::shared_ptr<const Empty> getAEmpty() const = 0;
	virtual const std::shared_ptr<const Point> getAPoint() const = 0;
	virtual const std::shared_ptr<const Line> getALine() const = 0;
	virtual const std::shared_ptr<const Polygon> getAPolygon() const = 0;

	//Debug method.
	virtual const std::string toString() const = 0;

};

/** \ingroup poxelcollgeometryconvexccwpolygon
 *
 *  A true subset of the convex polygon.
 */
class EmptyPointLine: public virtual ConvexCCWPolygon {
public:
	virtual ~EmptyPointLine() {
	}
};

/** \ingroup poxelcollgeometryconvexccwpolygon
 *
 * A true subset of the convex polygon.
 */
class EmptyPoint: public virtual EmptyPointLine {
public:
	virtual ~EmptyPoint() {
	}
};
/** \ingroup poxelcollgeometryconvexccwpolygon
 *
 * A special subset of the convex polygon, this type accepts no empty
 * polygons, and therefore provides more operations than the more general convex polygon.
 */
class NonemptyConvexCCWPolygon: public virtual ConvexCCWPolygon {
public:
	virtual ~NonemptyConvexCCWPolygon() {
	}

	/** The middle point of the polygon, defined as the average of all the points of the polygon.
	 * Always well-defined because the polygon is never empty.
	 */
	virtual const P middlePoint() const = 0;
};

/** \ingroup poxelcollgeometryconvexccwpolygon
 *
 * The empty convex CCW-polygon.
 */
class Empty: public virtual EmptyPoint {

private:
	Empty();
public:

	static std::shared_ptr<const Empty> getEmpty() {

		static const auto emptyPoint = std::shared_ptr<const Empty>(new Empty());

		return emptyPoint;
	}

	const std::shared_ptr<const std::vector<P>> points() const;

	const std::shared_ptr<const ConvexCCWPolygon> translate(P p) const;

	const ConvexCCWType getType() const;

	const std::shared_ptr<const Empty> getAEmpty() const;

	const std::shared_ptr<const Point> getAPoint() const;

	const std::shared_ptr<const Line> getALine() const;

	const std::shared_ptr<const Polygon> getAPolygon() const;

	const std::string toString() const;
};

/** \ingroup poxelcollgeometryconvexccwpolygon
 *
 * A Point represents a convex CCW-polygon consisting of just one point.
 */
class Point: public virtual EmptyPoint, public virtual NonemptyConvexCCWPolygon {
public:
	const P myPoint;
private:
	const std::shared_ptr<const std::vector<P>> myPoints;

public:

	Point(const P point) :
			myPoint(point), myPoints(
					std::shared_ptr<const std::vector<P>>(
							new std::vector<P>(1, point))) {
	}

	const std::shared_ptr<const std::vector<P>> points() const {
		return myPoints;
	}

	const P middlePoint() const {
		return myPoint;
	}

	const std::shared_ptr<const ConvexCCWPolygon> translate(P p) const {
		return std::shared_ptr<const ConvexCCWPolygon>(
				new Point(P(myPoint.gX() + p.gX(), myPoint.gY() + p.gY())));
	}

	const ConvexCCWType getType() const {return ConvexCCWType::PointT;}

	const std::shared_ptr<const Empty> getAEmpty() const {
		std::cerr << "Tried to get an empty from a non-empty." << std::endl;
		throw 1;
	};

	const std::shared_ptr<const Point> getAPoint() const {
		return std::shared_ptr<const Point>(new Point(*this));
	};

	const std::shared_ptr<const Line> getALine() const {
		std::cerr << "Tried to get a line from a non-line." << std::endl;
		throw 1;
	};

	const std::shared_ptr<const Polygon> getAPolygon() const {
		std::cerr << "Tried to get a polygon from a non-polygon." << std::endl;
		throw 1;
	};

	const std::string toString() const {
		std::stringstream ss;
		ss << "Point(" << myPoint.toString() << ")";

		return ss.str();
	}
};

/** \ingroup poxelcollgeometryconvexccwpolygon
 *
 * A line with two strictly different points.
 */
class Line: public virtual EmptyPointLine,
		public virtual NonemptyConvexCCWPolygon {

public:
	const P myP1;
	const P myP2;

private:

	const std::shared_ptr<const std::vector<P>> myPoints;
	const P myMiddlePoint;

	Line(const P p1, const P p2);
public:

	const std::shared_ptr<const std::vector<P>> points() const;

	const P middlePoint() const;

	const std::shared_ptr<const ConvexCCWPolygon> translate(P p) const;

	static const std::shared_ptr<const EmptyPointLine> create(P p1, P p2) {
		if (p1.equal(p2)) {
			return std::shared_ptr<const EmptyPointLine>(new Point(p1));
		} else {
			return std::shared_ptr<const EmptyPointLine>(new Line(p1, p2));
		}
	}

	static const std::shared_ptr<const Line> createUtterlyUnsafelyNotChecked(
			P p1, P p2) {
		return std::shared_ptr<const Line>(new Line(p1, p2));
	}

	const ConvexCCWType getType() const;

	const std::shared_ptr<const Empty> getAEmpty() const;

	const std::shared_ptr<const Point> getAPoint() const;

	const std::shared_ptr<const Line> getALine() const;

	const std::shared_ptr<const Polygon> getAPolygon() const;

	const std::string toString() const;
};

/** \ingroup poxelcollgeometryconvexccwpolygon
 *
 * A simple, convex, CCW polygon.
 *
 * The polygon has no duplicate points, it has at least 3 points,
 * there is no collinearity between its points at all,
 * it has a non-zero area, etc.
 */
class Polygon: public virtual NonemptyConvexCCWPolygon {

private:

	Polygon(const P p1, const P p2, const P p3,
			const std::shared_ptr<const std::vector<P>> rest);

public:
	const P myP1;
	const P myP2;
	const P myP3;
	const std::shared_ptr<const std::vector<P>> myRest;
private:
	std::shared_ptr<const std::vector<P>> myPoints; //NOTE: Do not change.
	P myMiddlePoint;

public:

	const std::shared_ptr<const std::vector<P>> points() const;

	static const double getX(P p) {
		return p.gX();
	}
	static const double getY(P p) {
		return p.gY();
	}

	const P middlePoint() const;

	const std::shared_ptr<const ConvexCCWPolygon> translate(P p) const;

	/** Utterly unsafely way to create a polygon, only use is for speed and
	 * when it is ABSOLUTELY certain that the points constitute a valid polygon.
	 *
	 * @param p1 first point
	 * @param p2 second point
	 * @param p3 third point
	 * @param rest the rest of the points
	 * @return valid polygon if points are valid, else undefined
	 */
	static const std::shared_ptr<const NonemptyConvexCCWPolygon> createUtterlyUnsafelyNotChecked(
			P p1, P p2, P p3, std::shared_ptr<const std::vector<P>> rest) {
		return std::shared_ptr<const NonemptyConvexCCWPolygon>(
				new Polygon(p1, p2, p3, rest));
	}

	static const std::shared_ptr<const NonemptyConvexCCWPolygon> createUtterlyUnsafelyNotChecked(
			std::shared_ptr<const std::vector<P>> points) {

		const auto p1 = (*points).front();
		const auto p2 = *(++(*points).begin());
		const auto p3 = *(++++(*points).begin());
		auto rest = new std::vector<P>(*points);
		std::reverse(rest->begin(), rest->end());
		rest->pop_back();
		rest->pop_back();
		rest->pop_back();
		std::reverse(rest->begin(), rest->end());

		return std::shared_ptr<const NonemptyConvexCCWPolygon>(new Polygon(
				p1, p2, p3, std::shared_ptr<const std::vector<P>>(rest)
		));
	}

	const ConvexCCWType getType() const;

	const std::shared_ptr<const Empty> getAEmpty() const;

	const std::shared_ptr<const Point> getAPoint() const;

	const std::shared_ptr<const Line> getALine() const;

	const std::shared_ptr<const Polygon> getAPolygon() const;

	const std::string toString() const;
};

/** \ingroup poxelcollgeometryconvexccwpolygon
 *
 * A collision segment represents a collision between two directed line segments
 * of different polygons, and their collision point.
 *
 * If the directed line segments collide in more than one point,
 * the collision point is one of the heads that overlap.
 * If two heads overlap in two different points, this can generally raise issues,
 * but since it is clear that if two heads overlap in two different points,
 * the intersection of the two polygons is a line,
 * and no more work is needed.
 *
 * A directed line segment from a given index into a polygon is understod
 * as the line segment from the point of the given index to the point
 * of the next index, excluding the point of the given index.
 * For instance, if index points to p1 = P(3, 0), and next index points to
 * p1Next = P(0, 4), the directed line segments includes all the points
 * from p1 to p1Next, p1Next, but not p1 itself.
 */
class CollisionSegment {
private:
	int index1;
	int index2;
	P collisionPoint;
public:
	CollisionSegment(const int aIndex1, const int aIndex2, const P aCollisionPoint);

	const int gIndex1() const;
	const int gIndex2() const;
	const P gCollisionPoint() const;
};

}

#endif /* POXELCOLL_GEOMETRY_CONVEXCCWPOLYGON_DATATYPES_HPP_ */
