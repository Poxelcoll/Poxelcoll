package poxelcoll.geometry.convexccwpolygon

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

/** This is a simple, convex, counter-clockwise polygon meant to represent a convex hull.
  *
  * There is no duplicated points, nor any collinearity.
  *
  * The polygon may be empty.
  */
sealed abstract class ConvexCCWPolygon {
  /** The points as an indexed sequence. */
  val points:IndexedSeq[P]
  /** Translate the points by a vector represented as a point.
    *
    * @param the vector to translate with
    * @return the translated polygon
    */
  def translate(p:P):ConvexCCWPolygon
}
/** A true subset of the convex polygon. */
sealed trait EmptyPointLine extends ConvexCCWPolygon
/** A true subset of the convex polygon. */
sealed trait EmptyPoint extends EmptyPointLine
/** A special subset of the convex polygon, this type accepts no empty
  * polygons, and therefore provides more operations than the more general convex polygon.
  */
sealed trait NonemptyConvexCCWPolygon extends ConvexCCWPolygon {
  /** The middle point of the polygon, defined as the average of all the points of the polygon.
    * Always well-defined because the polygon is never empty.
    */
  val middlePoint:P
  def translate(p:P):NonemptyConvexCCWPolygon
}

/** The empty convex CCW-polygon. */
final case object Empty extends ConvexCCWPolygon with EmptyPointLine with EmptyPoint {
  val points = IndexedSeq.empty
  def translate(p:P) = this
}

/** A Point represents a convex CCW-polygon consisting of just one point. */
final case class Point(p:P) extends ConvexCCWPolygon with EmptyPointLine with EmptyPoint with NonemptyConvexCCWPolygon {
  lazy val points = IndexedSeq(p)
  lazy val middlePoint = p
  def translate(aP:P) = Point(p + aP)
}

/** A line with two strictly different points. */
final case class Line private(p1:P, p2:P) extends ConvexCCWPolygon with EmptyPointLine with NonemptyConvexCCWPolygon {
  lazy val points = IndexedSeq(p1, p2)
  lazy val middlePoint = P((p1.x + p2.x) / 2, (p1.y + p2.y) / 2)
  def translate(aP:P) = Line(p1 + aP, p2 + aP)
}
/** A simple, convex, CCW polygon.
  *
  * The polygon has no duplicate points, it has at least 3 points,
  * there is no collinearity between its points at all,
  * it has a non-zero area, etc.
  */
final case class Polygon private (p1:P, p2:P, p3:P, rest:Seq[P] with Immutable) extends ConvexCCWPolygon with NonemptyConvexCCWPolygon {
  lazy val points = IndexedSeq(p1, p2, p3) ++ rest
  lazy val middlePoint = P(points.map(_.x).sum / points.length, points.map(_.y).sum / points.length) //The middle is the average.
  def translate(aP:P) = Polygon(p1 + aP, p2 + aP, p3 + aP, scala.collection.immutable.Seq.empty ++ rest.map(_ + aP))
}

object Line {
  def create(p1:P, p2:P) = {
    if (p1 == p2) Point(p1)
    else Line(p1, p2)
  }
  def createUtterlyUnsafelyNotChecked(p1:P, p2:P) = {
    Line(p1, p2)
  }
}

object Polygon {
  /** Preferred method for constructing a polygon. Throws an exception if anything is wrong with the input.
    *
    * The input points must be given in CCW order. This includes the first 3 points.
    * The points must represent a valid polygon, as defined in the polygon interface.
    *
    * @param p1 first point
    * @param p2 second point
    * @param p3 third point
    * @param rest the rest of the points
    * @return valid polygon or exception if invalid
    */
  def create(p1:P, p2:P, p3:P, rest:Seq[P] with Immutable) = {

    //Check that all points are unique.

    val set = Set(p1, p2, p3) ++ rest
    if (set.size != rest.size + 3) {
      throw new IllegalArgumentException("The points are not all unique: " + p1 + ", " + p2 + ", " + p3 + ", " + rest)
    }
    else {

      //Check that all points go counter-clockwise.

      val seq = IndexedSeq(p1, p2, p3) ++ rest

      val allCounterClockwise = (0 until seq.length).forall(a => {
        val p11 = seq(a)
        val p12 = seq((a+1) % seq.length)
        val p13 = seq((a+2) % seq.length)
        val v1 = p12 - p11
        val v2 = p13 - p12
        (v1 X v2) > 0.0
      })

      if (!allCounterClockwise) {
        throw new IllegalArgumentException("The polygon given is not strictly counter-clockwise: " + p1 + ", " + p2 + ", " + p3 + ", " + rest)
      }
      else {

        //Check that the total angle sum is not over 360 degrees (and do it in a possibly numerically stable way).

        val v1 = p2 - p1

        val sumIs360 = (2 until seq.length - 1).forall(a => {
          val p11 = seq(a)
          val p12 = seq((a+1) % seq.length)
          val p13 = seq((a+2) % seq.length)
          val v2 = p12 - p11
          val v3 = p13 - p12
          (v1 X v2) >= 0.0 || (v1 X v3) < 0.0
        })

        if (!sumIs360) {
          throw new IllegalArgumentException("The total angle sum of the polygon is not 360: " + p1 + ", " + p2 + ", " + p3 + ", " + rest)
        }
        else {
          Polygon(p1, p2, p3, rest)
        }
      }
    }
  }

  /** Utterly unsafely way to create a polygon, only use is for speed and
    * when it is ABSOLUTELY certain that the points constitute a valid polygon.
    *
    * @param p1 first point
    * @param p2 second point
    * @param p3 third point
    * @param rest the rest of the points
    * @return valid polygon if points are valid, else undefined
    */
  def createUtterlyUnsafelyNotChecked(p1:P, p2:P, p3:P, rest:Seq[P] with Immutable) = {
    Polygon(p1, p2, p3, rest)
  }

  /** Utterly unsafely way to create a polygon, only use is for speed and
    * when it is ABSOLUTELY certain that the points constitute a valid polygon.
    *
    * @param all all the points of the polygon
    * @return valid polygon if points are valid, else undefined
    */
  def createUtterlyUnsafelyNotChecked(all:Seq[P] with Immutable) = all.toList match {
    case (a::b::c::d) => {
      Polygon(a, b, c, d)
    }
    case _ => throw new IllegalArgumentException("Not enough elements in argument: " + all)
  }
}

/** A collision segment represents a collision between two directed line segments
  * of different polygons, and their collision point.
  *
  * If the directed line segments collide in more than one point,
  * the collision point is one of the heads that overlap.
  * If two heads overlap in two different points, this can generally raise issues,
  * but since it is clear that if two heads overlap in two different points,
  * the intersection of the two polygons is a line,
  * and no more work is needed.
  *
  * A directed line segment from a given index into a polygon is understod
  * as the line segment from the point of the given index to the point
  * of the next index, excluding the point of the given index.
  * For instance, if index points to p1 = P(3, 0), and next index points to
  * p1Next = P(0, 4), the directed line segments includes all the points
  * from p1 to p1Next, p1Next, but not p1 itself.
  */
sealed case class CollisionSegment(index1:Int, index2:Int, collisionPoint:P)
