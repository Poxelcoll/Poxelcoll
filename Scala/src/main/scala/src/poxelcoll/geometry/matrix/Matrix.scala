package poxelcoll.geometry.matrix

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

import poxelcoll.P

/** A point with 3 coordinates.  */
final case class P3(x:Double, y:Double, z:Double)

/** A 3-by-3 matrix of doubles.
  *
  * @param <A> The types the matrix implementation supports operations with
  */
trait Matrix[A <: Matrix[A]] {
  /** Multiply this matrix with a matrix of the same implementation.
    *
    * @param that another matrix of the same implementation
    * @return the result of the multiplication
    */
  def ***(that:A):A

  /** Multiply this matrix with a vector, like M * v.
    *
    * @param p the vector
    * @return the resulting vector of the multiplication
    */
  def *(p:P3):P3

  /** Multiply a sequence of points by treating each point P(x, y) as P3(x, y, z),
    * but possibly more efficiently than using * and mapping each point to P3.
    *
    * @param points the sequence of points to be transformed
    * @return the transformed points
    */
  def transformPoints(points:IndexedSeq[P]):IndexedSeq[P]

  /** The inverse of this matrix, or none if it doesn't have one.
    *
    * @return Some inverse matrix if it has one, else None
    */
  def inverseO:Option[A]

  /** Whether the matrix has an inverse. */
  def hasInverse:Boolean
}

/** A matrix factory interface.
  *
  * @param <A> the type of the implementation the matrix factory produces
  */
trait MatrixFactory[A <: Matrix[A]] {
  /**
   * @param data 3-by-3 matrix, a sequence of rows.
   * @return matrix
   */
  def createMatrix(data:IndexedSeq[IndexedSeq[Double]]):A

  /**
   * @param data 3-by-3 matrix, a sequence of rows.
   * @return matrix
   */
  def createMatrix2(data:Seq[Seq[Double]]):A = {
    createMatrix(data.map(_.toIndexedSeq).toIndexedSeq)
  }

  /**
   * @param data 3-by-3 matrix, an array consisting of 3 rows concatenated.
   * @return matrix
   */
  def createMatrixArray(data:Array[Double]):A
}

/** A simple, possibly efficient and tested matrix implementation.
  *
  * Only supports operations with other SimpleMatrix's.
  */
class SimpleMatrix(val data:Array[Double]) extends Matrix[SimpleMatrix] {

  def ***(that:SimpleMatrix) = {

    val d = data
    val e = that.data

    val result2 = Array(
      d(0)*e(0) + d(1)*e(3) + d(2)*e(6),
      d(0)*e(1) + d(1)*e(4) + d(2)*e(7),
      d(0)*e(2) + d(1)*e(5) + d(2)*e(8),
      d(3)*e(0) + d(4)*e(3) + d(5)*e(6),
      d(3)*e(1) + d(4)*e(4) + d(5)*e(7),
      d(3)*e(2) + d(4)*e(5) + d(5)*e(8),
      d(6)*e(0) + d(7)*e(3) + d(8)*e(6),
      d(6)*e(1) + d(7)*e(4) + d(8)*e(7),
      d(6)*e(2) + d(7)*e(5) + d(8)*e(8)
    )

    new SimpleMatrix(result2)
  }

  def *(p:P3) = {

    val d = data

    P3(
      d(0) * p.x + d(1) * p.y + d(2) * p.z,
      d(3) * p.x + d(4) * p.y + d(5) * p.z,
      d(6) * p.x + d(7) * p.y + d(8) * p.z
    )
  }

  def transformPoints(points:IndexedSeq[P]) = {

    val d = data

    points.map(p => {
      P(
        d(0) * p.x + d(1) * p.y + d(2),
        d(3) * p.x + d(4) * p.y + d(5)
      )
    })
  }

  lazy val inverseO = {

    //Unmaintainable but efficient implementation of 3-by-3 matrix inversion.
    //See matrix inversion on wikipedia for details.

    val da = data
    val (a, b, c, d, e, f, g, h, k) =
      (
        da(0),
        da(1),
        da(2),
        da(3),
        da(4),
        da(5),
        da(6),
        da(7),
        da(8)
      )

    val det = a*(e*k - f*h) + b*(f*g - k*d) + c*(d*h - e*g)

    if (det != 0.0) {
      val (a1, b1, c1, d1, e1, f1, g1, h1, k1) =
        (
          e*k - f*h,
          f*g - d*k,
          d*h - e*g,
          c*h - b*k,
          a*k - c*g,
          g*b - a*h,
          b*f - c*e,
          c*d - a*f,
          a*e - b*d
        )

      Some(new SimpleMatrix(Array(
        a1 / det, d1 / det, g1 / det,
        b1 / det, e1 / det, h1 / det,
        c1 / det, f1 / det, k1 / det
      )))
    }
    else {
      None
    }
  }

  lazy val hasInverse = {

    val da = data
    val (a, b, c, d, e, f, g, h, k) =
      (
        da(0),
        da(1),
        da(2),
        da(3),
        da(4),
        da(5),
        da(6),
        da(7),
        da(8)
      )

    val det = a*(e*k - f*h) + b*(f*g - k*d) + c*(d*h - e*g)

    det != 0.0
  }

  override def toString = {
    data.map(_.toString).reduce(_+", "+_)
  }
}

/** Factory for SimpleMatrix. */
object SimpleMatrix extends MatrixFactory[SimpleMatrix] {
  def createMatrix(data:IndexedSeq[IndexedSeq[Double]]):SimpleMatrix = {
    if (data.size != 3 || data.exists(_.size != 3)) {
      throw new IllegalArgumentException("Given data is not 3 by 3: " + data)
    }
    else {
      new SimpleMatrix(data.flatten.toArray)
    }
  }
  def createMatrixArray(data:Array[Double]) = {
    if (data.size != 9) {
      throw new IllegalArgumentException("Given data is not 3 by 3: " + data)
    }
    else {
      new SimpleMatrix(data)
    }
  }
}
