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

/** \defgroup poxelcoll poxelcoll
  * 
  * The top-level package, which contains the sub-packages which provides the functionality of the library, as well as the general data types.
  *
  * ==Overview over the different packages==
  *
  * The binaryimage package contains classes that for representing binary images.
  * Binary images are used for representing collision bitmasks, and are useful
  * for pixel-perfect collision detection.
  *
  * The mask package contains classes for representing and generating binary images.
  * Masks are the general representation of a collision primitive.
  * They support two primitives as of version 0.1, namely convex polygons and binary images.
  * Whether or not the binary image is present changes the meaning of the primitive;
  * if it not present, it means the convex polygon is the primitive;
  * if it is present, it means the convex polygon is used as an over-approximation
  * for the binary image to speed up collision detection.
  *
  * The geometry package contains packages that deals with the computational geometry part
  * of the library. This includes polygons, convex hulls, polygon intersection, convex hull
  * from points, matrices, matrix generation and others. The geometry package has in general
  * geometric stability, but not numerical stability.
  *
  * The collision package is the main package that supports the actual collision detection,
  * and uses the other packages to do this. As of version 0.1, the only collision phase supported
  * is the narrow phase, which is basically collision detection between pairs of objects.
  * Over-approximations such as axis-aligned bounding boxes and polygon intersection is
  * used to increase the efficiency of this phase and quickly prune those collisions that cannot happen.
  * The other part of this package is the part that deals with pixel-perfect collision detection.
  * This part also does not necessarily have numerical stability.
  */
