/* DataTypes.cpp */

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

#include "DataTypes.hpp"

namespace poxelcoll {

Empty::Empty() {
}

const std::shared_ptr<const std::vector<P>> Empty::points() const {
	static const auto empty = std::shared_ptr<const std::vector<P>>();

	return empty;
}

const std::shared_ptr<const ConvexCCWPolygon> Empty::translate(P p) const {
	return getEmpty();
}

const ConvexCCWType Empty::getType() const {return ConvexCCWType::EmptyT;}

const std::shared_ptr<const Empty> Empty::getAEmpty() const {
	return getEmpty();
};

const std::shared_ptr<const Point> Empty::getAPoint() const {
	std::cerr << "Tried to get a point from a non-point." << std::endl;
	throw 1;
};

const std::shared_ptr<const Line> Empty::getALine() const {
	std::cerr << "Tried to get a line from a non-line." << std::endl;
	throw 1;
};

const std::shared_ptr<const Polygon> Empty::getAPolygon() const {
	std::cerr << "Tried to get a polygon from a non-polygon." << std::endl;
	throw 1;
};

const std::string Empty::toString() const {return "Empty()";}

Line::Line(const P p1, const P p2) :
		myP1(p1), myP2(p2), myPoints(std::shared_ptr<const std::vector<P>>(new std::vector<P>({{p1, p2}}))),
		myMiddlePoint(P((myP1.gX() + myP2.gY()) / 2.0, (myP1.gY() + myP2.gY()) / 2.0)) {
}


const std::shared_ptr<const std::vector<P>> Line::points() const {
	return myPoints;
}

const P Line::middlePoint() const {
	return myMiddlePoint;
}

const std::shared_ptr<const ConvexCCWPolygon> Line::translate(P p) const {
	return std::shared_ptr<const ConvexCCWPolygon>(
			new Line(myP1.plus(p), myP2.plus(p)));
}

const ConvexCCWType Line::getType() const {return ConvexCCWType::LineT;}

const std::shared_ptr<const Empty> Line::getAEmpty() const {
	std::cerr << "Tried to get an empty from a non-empty." << std::endl;
	throw 1;
};

const std::shared_ptr<const Point> Line::getAPoint() const {
	std::cerr << "Tried to get a point from a non-point." << std::endl;
	throw 1;
};

const std::shared_ptr<const Line> Line::getALine() const {
	return std::shared_ptr<const Line>(new Line(*this));
};

const std::shared_ptr<const Polygon> Line::getAPolygon() const {
	std::cerr << "Tried to get a polygon from a non-polygon." << std::endl;
	throw 1;
};

const std::string Line::toString() const {
	std::stringstream ss;
	ss << "Line(" << myP1.toString() << ", " << myP2.toString() << ")";
	return ss.str();
}


Polygon::Polygon(const P p1, const P p2, const P p3,
		const std::shared_ptr<const std::vector<P>> rest) :
		myP1(p1), myP2(p2), myP3(p3), myRest(rest), myMiddlePoint(P(0,0)) {

	myPoints = [myP1, myP2, myP3, myRest]() {
		std::vector<P> intermediate;
		intermediate.push_back(myP1);
		intermediate.push_back(myP2);
		intermediate.push_back(myP3);

		return std::shared_ptr<const std::vector<P>>(std::move(addAll(intermediate, *myRest)));
	}();


	typedef std::vector<P> vP;
	typedef std::vector<double> vD;
	myMiddlePoint = [this]() {
		const auto pois = points();
		const auto xValue = sum<vD, double>(*map<vP, vD, decltype(getX)>(*pois, getX));
		const auto yValue = sum<vD, double>(*map<vP, vD, decltype(getY)>(*pois, getY));
		const auto length = (*pois).size();
		return P(xValue / length, yValue / length);
	}();
}

const std::shared_ptr<const std::vector<P>> Polygon::points() const {
	return myPoints;
}

const P Polygon::middlePoint() const { //The middle is the average.
	return myMiddlePoint;
}

const std::shared_ptr<const ConvexCCWPolygon> Polygon::translate(P p) const {
	auto f = [p](const P p1) {return p.plus(p1);};
	return std::shared_ptr<const ConvexCCWPolygon>(
			new Polygon(myP1.plus(p), myP2.plus(p), myP3.plus(p),
					std::shared_ptr<std::vector<P>>(
							map<std::vector<P>, std::vector<P>, decltype(f)>(
									*myRest, f))));
}

const ConvexCCWType Polygon::getType() const {return ConvexCCWType::PolygonT;}

const std::shared_ptr<const Empty> Polygon::getAEmpty() const {
	std::cerr << "Tried to get an empty from a non-empty." << std::endl;
	throw 1;
};

const std::shared_ptr<const Point> Polygon::getAPoint() const {
	std::cerr << "Tried to get a point from a non-point." << std::endl;
	throw 1;
};

const std::shared_ptr<const Line> Polygon::getALine() const {
	std::cerr << "Tried to get a line from a non-line." << std::endl;
	throw 1;
};

const std::shared_ptr<const Polygon> Polygon::getAPolygon() const {
	return std::shared_ptr<const Polygon>(new Polygon(*this));
};

const std::string Polygon::toString() const {
	std::stringstream ss;
	ss << "Polygon(" << myP1.toString() << ", " << myP2.toString() << ", " << myP3.toString() << ", ";
	const auto endI = (*myRest).end();
	for (auto i = (*myRest).begin(); i != endI; i++) {
		ss << (*i).toString();
		if (i != endI) {
			ss << ", ";
		}
	}
	ss << ")";
	return ss.str();
}

CollisionSegment::CollisionSegment(const int aIndex1, const int aIndex2, const P aCollisionPoint) :
		index1(aIndex1), index2(aIndex2), collisionPoint(aCollisionPoint) {
}

const int CollisionSegment::gIndex1() const {
	return index1;
}
const int CollisionSegment::gIndex2() const {
	return index2;
}
const P CollisionSegment::gCollisionPoint() const {
	return collisionPoint;
}

}
