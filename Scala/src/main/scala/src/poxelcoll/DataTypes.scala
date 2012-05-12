package poxelcoll

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

import poxelcoll.mask.Mask

/** A point with two coordinates. Can also be used to represent vectors. */
sealed case class P(x:Double, y:Double) { //Point.
  /** Vector addition.
    *
    * @param that other vector
    * @return the sum vector
    */
  def +(that:P) = P(x + that.x, y + that.y)
  /** Vector subtraction.
   *
    * @param that other vector
    * @return this vector minus that vector
    */
  def -(that:P) = P(x - that.x, y - that.y)
  /** Scaling.
    *
    * @param scale factor
    * @return scaled point/vector
    */
  def *(scale:Double) = P(x*scale, y*scale)
  /** The negative of the coordinates.
    *
    * @return the negative point/vector.
    */
  def unary_- = P(-x, -y)
  /** Cross-product.
    *
    * @param that other vector
    * @return this vector cross-product that vector
    */
  def X(that:P) = x * that.y - y * that.x
  /** Dot-product.
    *
    * @param that other vector
    * @return this vector dot that vector
    */
  def dot(that:P) = x*that.x + y*that.y
  /** Inverse scale. Undefined if inverseScale == 0.0.
    *
    * @param inverseScale non-zero scale
    * @return inversely scale point/vector
    */
  def /(inverseScale:Double) = P(x/inverseScale, y/inverseScale)
  /** The length of the vector/distance from the point to origo.
    *
    * @return the length of the vector
    */
  def norm = math.sqrt(x*x + y*y)
  /** @return some normalized vector if length not 0, else none */
  def norma = {
    val norm1 = norm
    if (norm1 == 0.0) {
      None
    }
    else {
      Some(P(x / norm1, y / norm1))
    }
  }
  /** @return unsafe normalization of vector */
  def normaUnsafe = {
    val norm1 = norm
    P(x / norm1, y / norm1)
  }
}

/** 2-dimensional integer point/vector. */
final case class IP(x:Int, y:Int) { //Integer Point.
  /** Point/vector addition.
    *
    * @param that other vector
    * @return this vector plus that vector
    */
  def +(that:IP) = IP(x + that.x, y + that.y)

  /** Point/vector subtraction.
    *
    * @param that other vector
    * @return this vector minus that vector
    */
  def -(that:IP) = IP(x - that.x, y - that.y)

  /** @return convert to double-precision point */
  def toP = P(x, y)
}

/** Collision info represents a collision object, including the mask, a position, an angle, scaling, and an id.
  *
  * The order of transformation is: origin, scaling, rotation, position.
  *
  * Angle is in radians, position is in pixels, and the scaling factors are percentages (where 1.0 == 100%).
  */
final case class CollisionInfo(mask:Mask, position:P, angle:Double, scaleX:Double, scaleY:Double, id:Int)

/** An axis-aligned bounding box.
  *
  * Invariant: Both of pMin's coordinates must be smaller or equal than the corresponding coordinates in pMax.
  */
final case class BoundingBox(pMin:P, pMax:P) { //Points are allowed to be the same, or have the same x-values or y-values.
  /** Given a mapping function, transform this bounding box.
    *
    * @param fun mapping function
    * @return result of mapping this bounding box
    */
  def map[A](fun:BoundingBox => A):A = {
    fun(this)
  }

  /** Whether or not this axis-aligned bounding box intersects another bounding box.
    *
    * @param that other axis-aligned bounding box
    * @return whether the boxes intersects
    */
  final def intersects(that:BoundingBox) = {
    this.pMin.x <= that.pMax.x &&
    that.pMin.x <= this.pMax.x &&
    this.pMin.y <= that.pMax.y &&
    that.pMin.y <= this.pMax.y
  }
}

/** A collision pair indicates that two collision objects with strictly different ids
  * has collided.
  *
  * Invariant: id1 is always strictly smaller than id2.
  */
final case class CollisionPair private(val id1:Int, val id2:Int) //id1 is always strictly lower than id2.

/** Collision pair factory methods. */
object CollisionPair {
  /** Safe creation of collision pair. This method is the preferred way of creating collision pairs.
    *
    * @param id1 first id
    * @param id2 second id
    * @return the collision pair, or none if the ids are equivalent
    */
  final def createO(id1:Int, id2:Int) = {
    if (id1 == id2) {
      None
    }
    else {
      if (id1 < id2) Some(CollisionPair(id1, id2))
      else Some(CollisionPair(id2, id1))
    }
  }

  /** INVARIANT: id1, id2 not equal, second id2 strictly larger
    *
    * This method should only be used with great care, and post-optimization
    * and profiling indicates that it is safe to use.
    *
    * @param id1 smaller, not equal to id2
    * @param id2 larger, not equal to id1
    * @return collision pair, possibly invalid if bad arguments
    */
  final def createUtterlyUnsafelyDoCarefullyObeyInvariant(id1:Int, id2:Int) = { //NOTE: Dangerous!
    CollisionPair(id1, id2)
  }
}
