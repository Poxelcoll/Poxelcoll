/* Matrix.cpp */

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

#include <sstream>

#include "Matrix.hpp"

namespace poxelcoll {

P3::P3(double aX, double aY, double aZ) :
			x(aX), y(aY), z(aZ) {
	}

double P3::gX() const {
	return x;
}
double P3::gY() const {
	return y;
}
double P3::gZ() const {
	return z;
}

Matrix::Matrix(const boost::array<double, dataSize>& aData) :
		data(aData) {
}

const std::shared_ptr<const Matrix> Matrix::matrixMult(const Matrix& that) const {

	const auto d = data;
	const auto e = that.data;

	const boost::array<double, dataSize> result3 = { { d[0] * e[0]
			+ d[1] * e[3] + d[2] * e[6], d[0] * e[1] + d[1] * e[4]
			+ d[2] * e[7], d[0] * e[2] + d[1] * e[5] + d[2] * e[8], d[3]
			* e[0] + d[4] * e[3] + d[5] * e[6], d[3] * e[1] + d[4] * e[4]
			+ d[5] * e[7], d[3] * e[2] + d[4] * e[5] + d[5] * e[8], d[6]
			* e[0] + d[7] * e[3] + d[8] * e[6], d[6] * e[1] + d[7] * e[4]
			+ d[8] * e[7], d[6] * e[2] + d[7] * e[5] + d[8] * e[8] } };

	return std::shared_ptr<const Matrix>(new Matrix(result3));
}

const P3 Matrix::vectorMult(const P3 p) const {

	const auto d = data;

	return P3(d[0] * p.gX() + d[1] * p.gY() + d[2] * p.gZ(),
			d[3] * p.gX() + d[4] * p.gY() + d[5] * p.gZ(),
			d[6] * p.gX() + d[7] * p.gY() + d[8] * p.gZ());
}

const std::shared_ptr<const std::vector<P>> Matrix::transformPoints(
		const std::vector<P>& points) const {

	auto d = data;

	auto tra = [d](const P p) {return P(
				d[0] * p.gX() + d[1] * p.gY() + d[2],
				d[3] * p.gX() + d[4] * p.gY() + d[5]);};

	return std::shared_ptr<const std::vector<P>>(
			map<std::vector<P>, std::vector<P>, decltype(tra)>(points, tra));
}

const Matrix* Matrix::inverseNull() const {

	//Unmaintainable but efficient implementation of 3-by-3 matrix inversion.
	//See matrix inversion on wikipedia for details.

	auto da = data;
	double a = da[0];
	double b = da[1];
	double c = da[2];
	double d = da[3];
	double e = da[4];
	double f = da[5];
	double g = da[6];
	double h = da[7];
	double k = da[8];

	auto det = a * (e * k - f * h) + b * (f * g - k * d)
			+ c * (d * h - e * g);

	if (det != 0.0) {

		double a1 = e * k - f * h;
		double b1 = f * g - d * k;
		double c1 = d * h - e * g;
		double d1 = c * h - b * k;
		double e1 = a * k - c * g;
		double f1 = g * b - a * h;
		double g1 = b * f - c * e;
		double h1 = c * d - a * f;
		double k1 = a * e - b * d;

		const boost::array<double, dataSize> res = { { a1 / det, d1 / det,
				g1 / det, b1 / det, e1 / det, h1 / det, c1 / det, f1 / det,
				k1 / det } };

		return new Matrix(res);
	} else {
		return 0;
	}
}

const bool Matrix::hasInverse() const { //TODO: This could be cached.

	auto da = data;
	double a = da[0];
	double b = da[1];
	double c = da[2];
	double d = da[3];
	double e = da[4];
	double f = da[5];
	double g = da[6];
	double h = da[7];
	double k = da[8];

	auto det = a * (e * k - f * h) + b * (f * g - k * d)
			+ c * (d * h - e * g);

	return det != 0.0;
}

const std::string Matrix::toString() const {
	const auto tra = [](double value) {
		std::ostringstream s;
		s << value << ", ";
		return s.str();
	};
	auto dataVectorStrings = map<boost::array<double, dataSize>,
			std::vector<std::string>, decltype(tra)>(data, tra);

	auto reduce =
			[](const std::string s1, const std::string s2) {return s1 + s2;};
	auto dataString = reduceDefault<std::vector<std::string>, std::string,
			decltype(reduce)>(*dataVectorStrings, "", reduce);

	return dataString;
}

}

