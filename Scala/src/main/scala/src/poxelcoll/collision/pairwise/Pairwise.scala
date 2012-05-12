package poxelcoll.collision.pairwise

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

import poxelcoll.CollisionInfo

/** The pairwise detects collisions between 2 collision objects.
  *
  * Pairwise collision detection has varying levels of precision and performance,
  * depending on the implementation used. Generally, high precision and performance cannot
  * be achieved simultaneously, so different implementations offer different trade-offs
  * between them.
  */
trait Pairwise {

  /** Given two collision objects, determine whether there is a collision between them.
    *
    * The precision and performance depends on the concrete implementation.
    *
    * @param collInfo1 first collision object
    * @param collInfo2 second collision object
    * @return whether there is a collision or not between the two objects
    */
  def testForCollision(collInfo1:CollisionInfo, collInfo2:CollisionInfo):Boolean
}
