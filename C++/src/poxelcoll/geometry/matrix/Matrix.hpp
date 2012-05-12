/* Matrix.hpp */

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

#ifndef POXELCOLL_GEOMETRY_MATRIX_MASK_HPP_
#define POXELCOLL_GEOMETRY_MATRIX_MASK_HPP_

#include <memory>
#include <boost/array.hpp>

#include "../../DataTypes.hpp"
#include "../../functional/Functional.hpp"

using namespace poxelcoll::functional;

namespace poxelcoll {

/** \ingroup poxelcollgeometrymatrix
 *
 * A point with 3 coordinates.
 */
class P3 {
private:
	double x; //Do not change.
	double y; //Do not change.
	double z; //Do not change.
public:
	P3(double aX, double aY, double aZ);
public:

	double gX() const;
	double gY() const;
	double gZ() const;
};

/** \ingroup poxelcollgeometrymatrix
 *
 * A 3-by-3 matrix of doubles.
 */
class Matrix {
public:
	static const unsigned int dataSize = 9;
private:
	const boost::array<double, dataSize> data;

public:

	Matrix(const boost::array<double, dataSize>& aData);

	/** Multiply this matrix with a matrix of the same implementation.
	 *
	 * @param that another matrix of the same implementation
	 * @return the result of the multiplication
	 */
	const std::shared_ptr<const Matrix> matrixMult(const Matrix& that) const;

	/** Multiply this matrix with a vector, like M * v.
	 *
	 * @param p the vector
	 * @return the resulting vector of the multiplication
	 */
	const P3 vectorMult(const P3 p) const;

	/** Multiply a sequence of points by treating each point P(x, y) as P3(x, y, z),
	 * but possibly more efficiently than using * and mapping each point to P3.
	 *
	 * @param points the sequence of points to be transformed
	 * @return the transformed points
	 */
	const std::shared_ptr<const std::vector<P>> transformPoints(
			const std::vector<P>& points) const;

	/** The inverse of this matrix, or none if it doesn't have one.
	 *
	 * @return Some inverse matrix if it has one, else None
	 */
	const Matrix* inverseNull() const;

	/** Whether the matrix has an inverse. */
	const bool hasInverse() const;

	const std::string toString() const;

	static std::unique_ptr<Matrix> createMatrixArray(
			const boost::array<double, dataSize> & data) {
		return std::unique_ptr<Matrix>(new Matrix(data));
	}
};

}

#endif /* POXELCOLL_GEOMETRY_MATRIX_MASK_HPP_ */
