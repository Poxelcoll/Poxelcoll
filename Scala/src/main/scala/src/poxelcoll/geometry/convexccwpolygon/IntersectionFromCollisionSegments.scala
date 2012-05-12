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

/** Given a sequence of collision segments between two convex polygons in CCW-order, finds the intersection between the two polygons.
  *
  * This is done in linear time in the number of points of the polygons.
  *
  * This collision segments finder is part of a robust variation of the algorithm found here:
  * http://www-cgrl.cs.mcgill.ca/~godfried/teaching/cg-projects/97/Plante/CompGeomProject-EPlante/algorithm.html
  * This handles merging the previously found intersections of the first and second polygon.
  *
  * If the given sequence is empty, the intersection may be non-empty,
  * for instance if one of the polygons is fully inside the other.
  * To check for that, it is checked whether one of the polygons has
  * a point inside the other, and that polygon is then returned.
  * If no polygon is inside the other, the result is empty.
  *
  * If the given sequence is non-empty, the intersection is non-empty, and it is found
  * by following the collision segments in CCW-order.
  *
  * ==Status==
  *
  * The current implementation as of 0.1 is meant to be geometrically robust,
  * but gives no guarantees in regards to being numerically robust.
  * The consequences of the lack of numerical robustness is unknown,
  * but may range from imprecision to undefined behaviour.
  * The numerical robustness may be improved in the future.
  *
  * @param collisionSegments intersections between the two polygons, given in CCW-order
  * @param poly1Points points of the first convex CCW polygon
  * @param poly2Points points of the second convex CCW polygon
  */
final class IntersectionFromCollisionSegments(collisionSegments:CollisionSegments,
      poly1Points:IndexedSeq[P], poly2Points:IndexedSeq[P]) {

  import GeneralFunctions._

  /** Number of points in polygon 1. */
  private[this] val size1 = poly1Points.size
  /** Number of points in polygon 2. */
  private[this] val size2 = poly2Points.size

  /** Given an index in a polygon of a given size, find the circular next index.
    *
    * @param a valid index of a polygon
    * @param size the number of points in the convex polygon
    * @return the next circular index
    */
  private[this] def next(a:Int, size:Int) = (a + 1) % size

  /** Given an index in a polygon of a given size, find the circular previous index.
    *
    * @param a valid index of a polygon
    * @param size the number of points in the convex polygon
    * @return the previous circular index
    */
  private[this] def prev(a:Int, size:Int) =  if (a - 1 < 0) size - 1 else a - 1

   /** Find the next circular index for polygon 1.
    *
    * @param i valid index for polygon 1
    * @return the next circular index
    */
  private[this] def next1(i:Int) = next(i, size1)

  /** Find the next circular index for polygon 2.
    *
    * @param i valid index for polygon 2
    * @return the next circular index
    */
  private[this] def next2(i:Int) = next(i, size2)

  /** Returns whether the given vectors are in the same direction.
    *
    * Zero-vectors are always in the same direction, but zero-vectors should never be given as arguments.
    *
    * @param v1 first vector
    * @param v2 second vector
    * @return whether in the same direction, assuming no zero-vectors.
    */
  private[this] def sameDir(v1:P, v2:P) = {//If the same direction, or if one/both is zero-vector. Note that zero-vectors should never occur.
    ((v1 X v2) == 0.0) && (v1 dot v2) >= 0.0
  }

  /** Returns whether the given vectors are in the opposite direction.
    *
    * Zero-vectors are never in the same direction, but zero-vectors should never be given as arguments.
    *
    * @param v1 first vector
    * @param v2 second vector
    * @return whether in the opposite direction, assuming no zero-vectors.
    */
  private[this] def oppositeDir(v1:P, v2:P) = {//If opposite direction. Zero-vectors are defined as not opposite. Note that zero-vectors should never occur.
    ((v1 X v2) == 0.0) && (v1 dot v2) < 0.0
  }

  /** Given a list of vectors, checks that the vectors are in clock-wise order.
    *
    * If given any zero-vectors, the vectors are not in clock-wise order.
    *
    * 'Clock-wise order' is defined as, for each vector, no vector are in the same
    * direction as any other vector, and that the vector immediately cyclicly before it and
    * immediately after it is the same in the list as if the vectors were put on a clock,
    * and collected from one vector and clock-wise around.
    *
    * @param vs a list of vectors
    * @return whether the vectors are in clock-wise order, as defined above.
    */
  private[this] def cwOrder(vs:List[P]) = vs match {

    case Nil => true
    case all@(x::xs) => {
      if (all.exists(a => a.norm == 0.0)) {//NOTE: Checking for no zero-vectors.
        false
      }
      else {
        val xNorma = x.normaUnsafe//NOTE: Assuming no zero-vectors.
        val transformed = xs.map{
          a => (xNorma X a.normaUnsafe, xNorma dot a.normaUnsafe)//NOTE: Assuming no zero-vectors.
        }

        def check(crossDots:List[(Double, Double)]):Boolean = crossDots match {
          case Nil => true
          case y::Nil => true
          case y1::y2::ys => {
            y1._1 match {
              case 0.0 => y2._1 match {
                case 0.0 => false
                case _ if y2._1 > 0.0 => check(y2::ys)
                case _ => false
              }

              case _ if y1._1 > 0.0 => y2._1 match {
                case 0.0 => false
                case _ if y2._1 > 0.0 => if (y1._2 < y2._2) check(y2::ys) else false
                case _ => false
              }

              case _ => y2._1 match {
                case 0.0 => check(y2::ys)
                case _ if y2._1 > 0.0 => check(y2::ys)
                case _ => if (y1._2 > y2._2) check(y2::ys) else false
              }
            }
          }
        }

        transformed.forall( a => !(a._1 == 0.0 && a._2 >= 0.0) ) &&//Check that no vector is in the direction of x.
          check(transformed)
      }
    }
  }

  /** Given two directed line-segments returns whether the head of the first is ahead of the second.
    *
    * It is assumed that the directed line-segments collides, and that they are in the same direction.
    *
    * @param i1 index indicating the directed line-segment ]poly1(i1), poly1(next1(i1))]
    * @param i2 index indicating the directed line-segment ]poly2(i2), poly2(next2(i2))]
    * @return whether the head of the first directed line-segment is ahead of the head of the second directed line-segment.
    */
  private[this] def ahead(i1:Int, i2:Int) = { //Should only be used on overlapping, in-same-direction line pieces, where the heads does not overlap.

    val p11 = poly1Points(i1)
    val p12 = poly1Points(next(i1, size1))

    val p22 = poly2Points(next(i2, size2))

    val v1 = p12 - p11
    val v2 = p22 - p12

    (v1 dot v2) < 0.0
  }

  /** Determines whether the given point is inside the polygon indicated by the given point sequence.
    *
    * @param points the point sequence representing the polygon
    * @param nextFun a function that given an index into the sequence, gives the cyclically next index
    * @param point the point to check for
    * @return whether the point is inside the polygon
    */
  private[this] def pointInside(points:IndexedSeq[P], nextFun:Int => Int, point:P) = {
    (0 until points.size).forall(i => {
      val nextI = nextFun(i)
      val p1 = points(i)
      val p2 = points(nextI)
      val v1 = p2 - p1
      val v2 = point - p1
      (v1 X v2) >= 0.0
    })
  }

  /** Keep collecting points for the first polygon until there are no more points, or the next collision segment is reached and then continue constructing the intersection.
    *
    * @param collisionSegments the collision segments remaining
    * @param res the resulting intersection so far
    * @param i1 the current index of polygon 1 to investigate
    * @param i2 the current index of polygon 2 to investigate
    * @param lastSegment the segment that indicates at which point the construction of the result should stop. Is the original head of the collision segments
    * @return the final intersection represented as a list of points
    */
  private[this] def F1(collisionSegments:CollisionSegments, res:Vector[P], i1:Int, i2:Int, lastSegment:Option[CollisionSegment]):Vector[P] = collisionSegments match {
    case x::xs => {
      if (x.index1 == i1) {
        constructIntersection(collisionSegments, res, lastSegment)
      }
      else {
        val nextI1 = next(i1, size1)
        val p12 = poly1Points(nextI1)
        val newRes = {
          res match {
            case _ if res.size > 0 => {
              if (res.last == p12) res
              else res :+ p12
            }
            case _ /* res.size == 0 */ => Vector(p12)
          }
        }
        F1(collisionSegments, newRes, nextI1, i2, lastSegment)
      }
    }
    case Nil => lastSegment match {
      case None => res
      case Some(last) => {
        if (last.index1 == i1) {
          res
        }
        else {
          val nextI1 = next(i1, size1)
          val p12 = poly1Points(nextI1)
          val newRes = {
            res match {
              case _ if res.size > 0 => {
                if (res.last == p12) res
                else res :+ p12
              }
              case _ /* res.size == 0 */ => Vector(p12)
            }
          }
          F1(collisionSegments, newRes, nextI1, i2, lastSegment)
        }
      }
    }
  }

  /** Keep collecting points for the second polygon until there are no more points, or the next collision segment is reached and then continue constructing the intersection.
    *
    * @param collisionSegments the collision segments remaining
    * @param res the resulting intersection so far
    * @param i1 the current index of polygon 1 to investigate
    * @param i2 the current index of polygon 2 to investigate
    * @param lastSegment the segment that indicates at which point the construction of the result should stop. Is the original head of the collision segments
    * @return the final intersection represented as a list of points
    */
  private[this] def F2(collisionSegments:CollisionSegments, res:Vector[P], i1:Int, i2:Int, lastSegment:Option[CollisionSegment]):Vector[P] = collisionSegments match {
    case x::xs => {
      if (x.index2 == i2) {
        constructIntersection(collisionSegments, res, lastSegment)
      }
      else {
        val nextI2 = next(i2, size2)
        val p22 = poly2Points(nextI2)
        val newRes = {
          res match {
            case _ if res.size > 0 => {
              if (res.last == p22) res
              else res :+ p22
            }
            case _ /* res.size == 0 */ => Vector(p22)
          }
        }
        F2(collisionSegments, newRes, i1, nextI2, lastSegment)
      }
    }
    case Nil => lastSegment match {
      case None => res
      case Some(last) => {
        if (last.index2 == i2) {
          res
        }
        else {
          val nextI2 = next(i2, size2)
          val p22 = poly2Points(nextI2)
          val newRes = {
            res match {
              case _ if res.size > 0 => {
                if (res.last == p22) res
                else res :+ p22
              }
              case _ /* res.size == 0 */ => Vector(p22)
            }
          }
          F2(collisionSegments, newRes, i1, nextI2, lastSegment)
        }
      }
    }
  }

  /** Construct the intersection, by investigating the current collision segment and the relative configuration of the directed line-segments at it.
    *
    * @param collisionSegments the collision segments remaining
    * @param res the resulting intersection so far
    * @param lastSegment the segment that indicates at which point the construction of the result should stop. Is the original head of the collision segments
    * @return the intersection
    */
  private[this] def constructIntersection(collisionSegments:CollisionSegments, res:Vector[P], lastSegment:Option[CollisionSegment]):Vector[P] = collisionSegments match {
    case Nil => {
      res
    }
    case x::xs => {

      val i1 = x.index1
      val nextI1 = next(i1, size1)
      val nextNextI1 = next(nextI1, size1)
      val i2 = x.index2
      val nextI2 = next(i2, size2)
      val nextNextI2 = next(nextI2, size2)

      val p11 = poly1Points(i1)
      val p12 = poly1Points(nextI1)
      val p13 = poly1Points(nextNextI1)

      val p21 = poly2Points(i2)
      val p22 = poly2Points(nextI2)
      val p23 = poly2Points(nextNextI2)

      val v11 = p12 - p11
      val v12 = p13 - p12

      val v21 = p22 - p21
      val v22 = p23 - p22

      if (p12 == p22) {
        if (!sameDir(v11, v21)) {
          if (cwOrder(List(-v11, v12, -v21)) && cwOrder(List(-v21, v22, -v11))) {
            Vector(p12)
          }
          else if (cwOrder(List(-v11, -v21, v22, v12)) || cwOrder(List(-v11, v22, v12, -v21))){
            F2(xs, res :+ p12, i1, i2, lastSegment)
          }
          else if (oppositeDir(v11, v22)) {
            Vector(
              p12,
              if ( (p11 - p12).norm < (p23 - p12).norm ) p11 else p23
            )
          }
          else {
            F1(xs, res :+ p12, i1, i2, lastSegment)
          }
        }
        else {
          if (cwOrder(List(-v11, v22, v12))) {
            F2(xs, res :+ p12, i1, i2, lastSegment)
          }
          else {
            F1(xs, res :+ p12, i1, i2, lastSegment)
          }
        }
      }
      else {

        if (oppositeDir(v11, v21)) {
          Vector(
            if ( (p12 - p11).norm < (p12 - p22).norm ) p11 else p22,
            if ( (p22 - p21).norm < (p22 - p12).norm ) p21 else p12
          )
        }
        else if (sameDir(v11, v21)) {
          if (ahead(i1, i2)) {
            F2(xs, res :+ p22, i1, i2, lastSegment)
          }
          else {
            F1(xs, res :+ p12, i1, i2, lastSegment)
          }
        }
        else {
          val collisionPointsInfo = getCollisionDirectedLineSegment(i1, i2, poly1Points, poly2Points)
          val collisionPointO = collisionPointsInfo match {
            case x::Nil => {
              Some(x.collisionPoint)
            }
            case _ => {
              //This should never happen.
              None
            }
          }

          collisionPointO match {
            case None => Vector.empty //This should never happen.
            case Some(collisionPoint) => {

              if (!Set(p12, p22).contains(collisionPoint)) {
                if ((v11 X v21) > 0.0) {
                  F2(xs, res :+ collisionPoint, i1, i2, lastSegment)
                }
                else {
                  F1(xs, res :+ collisionPoint, i1, i2, lastSegment)
                }
              }
              else if (Set(p12).contains(collisionPoint)) {
                if (cwOrder(List(v21, -v11, v12, -v21))) {
                  Vector(collisionPoint)
                }
                else if (cwOrder(List(-v11, v21, v12))) {
                  F2(xs, res :+ collisionPoint, i1, i2, lastSegment)
                }
                else {
                  F1(xs, res :+ collisionPoint, i1, i2, lastSegment)
                }
              }
              else {
                if (cwOrder(List(v11, -v21, v22, -v11))) {
                  Vector(collisionPoint)
                }
                else if (cwOrder(List(-v21, v11, v22))) {
                  F1(xs, res :+ collisionPoint, i1, i2, lastSegment)
                }
                else {
                  F2(xs, res :+ collisionPoint, i1, i2, lastSegment)
                }
              }
            }
          }
        }
      }
    }
  }

  /** Calculates the intersection given the parameterization.
    *
    * @return the intersection represented as a convex CCW polygon
    */
  def getIntersectionFromCollisionSegments():ConvexCCWPolygon = {

    val intersectingPolygon:List[P] = {


      if (collisionSegments.isEmpty) {

        if (pointInside(poly1Points, a => next(a, size1), poly2Points.head)) {
          poly2Points.toList
        }
        else if (pointInside(poly2Points, a => next(a, size2), poly1Points.head)) {
          poly1Points.toList
        }
        else {
          List.empty
        }
      }
      else {
        constructIntersection(
          collisionSegments,
          Vector.empty,
          Some(collisionSegments.head) //Collision segments is non-empty at this point.
        ).toList
      }
    }

    intersectingPolygon match {
      case Nil => Empty
      case x::Nil => Point(x)
      case x1::x2::Nil => Line.create(x1, x2)
      case x1::x2::x3::xs => Polygon.createUtterlyUnsafelyNotChecked(x1, x2, x3, xs)
    }
  }
}
