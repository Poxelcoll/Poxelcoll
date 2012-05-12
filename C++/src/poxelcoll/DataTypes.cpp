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

P::P(double aX, double aY) :
		x(aX), y(aY) {
}

double P::gX() const {
	return x;
}
double P::gY() const {
	return y;
}

const P P::plus(P that) const {
	return P(x + that.x, y + that.y);
}

const P P::minus(P that) const {
	return P(x - that.x, y - that.y);
}

const P P::multi(double scale) const {
	return P(x * scale, y * scale);
}

const P P::unaryMinus() const {
	return P(-x, -y);
}

const double P::cross(P that) const {
	return x * that.y - y * that.x;
}

const double P::dot(P that) const {
	return x * that.x + y * that.y;
}

const P P::divide(double inverseScale) const {
	return P(x / inverseScale, y / inverseScale);
}

const double P::norm() const {
	return sqrt(x * x + y * y);
}

const P P::normaUnsafe() const {
	const auto norm1 = norm();
	return P(x / norm1, y / norm1);
}

const bool P::equal(P that) const {
	return x == that.x && y == that.y;
}

const std::string P::toString() const {
	std::stringstream ss;
	ss << "P(" << x << ", " << y << ")";
	return ss.str();
}

IP::IP(int aX, int aY) :
		x(aX), y(aY) {
}

int IP::gX() const {
	return x;
}
int IP::gY() const {
	return y;
}

const IP IP::plus(IP that) const {
	return IP(x + that.x, y + that.y);
}

const IP IP::minus(IP that) const {
	return IP(x - that.x, y - that.y);
}

const P IP::toP() const {
	return P(x, y);
}

BoundingBox::BoundingBox(const P & aPMin, const P & aPMax): pMin(aPMin), pMax(aPMax) {
}

const bool BoundingBox::intersects(BoundingBox that) const {
	return pMin.gX() <= that.pMax.gX() && that.pMin.gX() <= pMax.gX()
			&& pMin.gY() <= that.pMax.gY() && that.pMin.gY() <= pMax.gY();
}

const std::string BoundingBox::toString() const {
	std::stringstream ss;
	ss << "BoundingBox(" << pMin.toString() << ", " << pMax.toString() << ")";
	return ss.str();
}

CollisionPair::CollisionPair(int aId1, int aId2) :
		id1(aId1), id2(aId2) {
}

}
