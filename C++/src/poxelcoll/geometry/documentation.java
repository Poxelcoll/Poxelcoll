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

/** \defgroup poxelcollgeometry poxelcoll_geometry
  * \ingroup poxelcoll
  * 
  * Geometry contains packages related to computational geometry, including convex hulls, polygons and matrices.
  *
  *
  * The convex hull and polygon package supports finding the convex hull from a point set
  * and finding the intersection between two convex hulls represented as polygons efficiently.
  *
  * The matrix package supports a simple 3-by-3 matrix and operations on it.
  *
  * Two important properties for computational geometry are
  * geometric stability and numerical stability.
  * Geometric stability regards handling all possible cases
  * and situations correctly, assuming that all operations
  * on numbers are performed with infinite precision.
  * As an example of an algorithm that does not have
  * geometric stability is an intersection-finding
  * algorithm that generally works well when
  * the intersection itself is a polygon, but fails
  * when ie. the intersection is a point or a line.
  *
  * The algorithms provided are in general believed to have geometric
  * stability. For instance, the convex hull calculation
  * always offers the correct answer (despite duplicate
  * points, colinearity, 0 points, etc), and the
  * convex hull intersection always offers the correct answer
  * (despite one hull containing the other, overlapping lines,
  * overlapping points, intersection being a point,
  * intersection being a line, etc.).
  *
  * Numerical stability is about the effect numbers without
  * infinite precision has on the computation.
  * Numerical instability comes from issues such as
  * lacking precision, rounding errors, comparision
  * between floating points, etc.
  * An example is the computation of the cross product
  * of two vector that is supposed to be positive,
  * but ends up being 0 or negative due to imprecision.
  * Even if a package has geometric stability
  * and is otherwise correct, numerical instability
  * can lead to all sorts of errors.
  * For an example of a library that guarantees numerical
  * stability (by having infinite precision), see CGAL.
  * For CGAL's discussion of numerical stability,
  * see http://www.cgal.org/philosophy.html .
  *
  * While the packages are guaranteed to strive towards having geometric stability
  * (ie. no issues regarding geometric stability are known to exists),
  * no guarantees are given regarding numerical stability.
  * The effects of numerical stability are unknown, and may
  * include everything from no effect, to imprecision,
  * to crashes/infinite loops/wrong results.
  */
