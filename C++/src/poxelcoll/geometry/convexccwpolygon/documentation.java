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

/** \defgroup poxelcollgeometryconvexccwpolygon poxelcoll_geometry_convexccwpolygon
  * \ingroup poxelcollgeometry
  * 
  * This package deals with convex hulls, polygons and polygon intersection.
  *
  * The main data type is the convex, counter-clockwise polygon: ConvexCCWPolygon.
  * This data type covers all possibilities of convex hulls.
  * The concrete types of ConvexCCWPolygon is Empty, Point, Line and Polygon.
  * Different, more constrained intermediate types exists, such as NonemptyConvexCCWPolygon,
  * which only covers Point, Line and Polygon.
  * The different concrete types are meant to improve both the geometric robustness as well
  * as type-safety.
  *
  * The package is meant to have geometric stability, but not necessarily numerical stability.
  */
