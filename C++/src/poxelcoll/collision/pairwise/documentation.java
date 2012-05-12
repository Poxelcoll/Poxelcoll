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

/** \defgroup poxelcollcollisionpairwise poxelcoll_collision_pairwise
  * \ingroup poxelcollcollision
  * 
  * The pairwise collision detection handles collisions between 2 independent objects.
  *
  * In general there are 2 main issues to handle for pairwise collision detection,
  * namely precision and performance. Getting high precision and performance
  * simultaneously is generally impossible. For that reason, different
  * implementations that provide different tradeoffs between the two
  * can be provided.
  *
  * '''Current status and improvements'''
  *
  * The status (as of version 0.1) is that 1 pairwise collision detection implementation is supported.
  *
  * The simple pixel-perfect pairwise collision detection implementation
  * supports transformed pixel-perfect collision relatively efficiently.
  * It also handles filled polygons as well, and uses bounding boxes
  * for quick pruning.
  * One issue is that its performance and precision can be adversely
  * affected by scaling.
  *
  * Apart from implementation optimization, this implementation should
  * support most needs. Current issues include numerical instability.
  * The exact effect of numerical instability in the implementation
  * is currently unknown, but future work may address the issue.
  * A potential future implementation could deal with the effect that
  * scaling has on performance and precision, but there are no
  * concrete plans for implementing this, and without any requests or desire
  * for implementing it, it may never be implemented.
  */
