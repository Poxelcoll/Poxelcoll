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

/** \defgroup poxelcollcollision poxelcoll_collision
  * \ingroup poxelcoll
  * 
  * The library supports the narrow phase of collision detection.
  *
  * The narrow phase of collision detection is defined as the parts
  * of collision detection that deals with detecting collisions between
  * two objects. This stands in contrast to the broad phase of collision detection,
  * which seeks to decrease the number of potential collisions between many objects
  * by efficiently filtering out objects that obviously do not collide
  * (ie. small objects that are far away from each other).
  * While this library as of version 0.1 offers no support for broad phase collision
  * detection, there exists other libraries that handles this task,
  * and which potentially can be used with this library.
  *
  * The narrow phase of collision detection generally consists of parts that quickly
  * filter out non-collisions quickly, as well as parts that more precisely and slowly
  * determines whether there is a collision.
  *
  * Two primitives are supported: Convex hulls represented by polygons, and images represented
  * by binary images (useful for pixel-perfect collision detection).
  * For the binary images, the convex hull is used as an (over-)approximation to speed up collision detection,
  * only checking the intersection of the convex hulls of the two colliding objects.
  * Since the convex hull never under-approximates the binary image,
  * it is only necessary to check the intersection of the convex hulls, since
  * anything outside the intersection will never yield a collision.
  *
  * Collision detection are supported between all types of primitives.
  * This means that collision detection for convex hull vs. convex hull, convex hull vs. binary image,
  * and binary image vs. binary image is supported.
  *
  * The standard implementation supports transformation of the two primitives.
  * By 'transformation' is meant translation, rotation and scaling along two axis.
  * This includes the binary images.
  * The transformation for binary images is generally implemented by transforming
  * the convex hull, finding the intersection, creating an image out of the intersection,
  * and back-transforming for each pixel into each object to see if the objects
  * overlap at some pixel. If the convex hulls are precise approximations
  * (ie. the binary image covers at least 95% of the area of the convex hull),
  * this should generally be somewhat efficient.
  */
