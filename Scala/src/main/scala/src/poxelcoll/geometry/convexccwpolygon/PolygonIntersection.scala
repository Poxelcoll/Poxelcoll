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
import poxelcoll.BoundingBox

/** Supports operations for finding the intersection between two polygons.
  *
  * The efficiency for finding the intersection is intended to be linear in the
  * size of the polygons points.
  *
  * ==Status==
  *
  * The current implementation as of 0.1 is meant to be geometrically robust,
  * but gives no guarantees in regards to being numerically robust.
  * The consequences of the lack of numerical robustness is unknown,
  * but may range from imprecision to undefined behaviour.
  * The numerical robustness may be improved in the future.
  */
object PolygonIntersection {

  import GeneralFunctions._

  /** Extracts the bounds from the given boundingBox option if present, else derives them from the given, non-empty points.
    *
    * @param polyPoints non-empty points
    * @param boundingBox bounding box option
    * @return the bounding box of the given polygon and bounding box option, or possibly undefined behaviour if points is empty
    */
  private[this] def getBounds(polyPoints:Seq[P], boundingBox:Option[(P, P)]) = boundingBox match {
    case Some(a) => a
    case None => {
      val polyPointsX = polyPoints.map(_.x)
      val polyPointsY = polyPoints.map(_.y)
      (P(polyPointsX.min, polyPointsY.min), P(polyPointsX.max, polyPointsY.max))
    }
  }

  /** A function that returns the leftmost point-index of the two, and if equally leftmost, the uppermost.
    *
    * Behaviour is undefined if the point-index pair has the same point. This should never happen.
    *
    * @return a function that finds the leftmost, upper point-index. The index corresponsd to the given point
    */
  private[this] def chooseLeftmostUpperPoint = ((oldPointIndex:(P, Int), newPointIndex:(P, Int)) =>
    if (newPointIndex._1.x > oldPointIndex._1.x || (newPointIndex._1.x == oldPointIndex._1.x && newPointIndex._1.y < oldPointIndex._1.y))
      oldPointIndex
    else newPointIndex
  )

  /** Given a non-empty point sequence, find the bounding box.
    *
    * Behaviour is undefined if the point sequence is empty.
    *
    * @param polyPoints a non-empty point sequence
    * @return bounding box of the non-empty point sequence, or undefined if empty
    */
  private[this] def bBoxNonemptyPolygon(polyPoints:IndexedSeq[P]) = {

    val polyPointsX = polyPoints.map(_.x)
    val polyPointsY = polyPoints.map(_.y)
    BoundingBox(P(polyPointsX.min, polyPointsY.min), P(polyPointsX.max, polyPointsY.max))
  }

  /** Finds the intersection between a point and a polygon.
    *
    * @param point the point
    * @param poly the polygon
    * @return the intersection
    */
  private[this] def handlePointPolygon(point:Point, poly:Polygon):EmptyPoint = {

    val polyPlusHead = (poly.p1 :: poly.p2 :: poly.p3 :: poly.rest.toList) :+ poly.p1

    def goThroughPoly(polys:List[P]):Boolean = polys match {
      case x1::x2::xs => {

        val v1 = x2 - x1
        val v2 = point.p - x1

        (v1 X v2) >= 0.0 && goThroughPoly(x2::xs)
      }
      case x::xs => true
      case Nil => true
    }

    if (goThroughPoly(polyPlusHead)) point else Empty
  }

  /** Finds the intersection between a line and a polygon.
    *
    * @param line the line
    * @param poly the polygon
    * @return the intersection
    */
  private[this] def handleLinePoly(line:Line, poly:Polygon) = {
    //Collide all line-segment pairs, and return the results.

    val p11 = line.p1
    val p12 = line.p2

    val polygonListPlusHead = (poly.p1 :: poly.p2 :: poly.p3 :: poly.rest.toList) :+ poly.p1 //NOTE: Add head.

    def collideAll(polyList:List[P], res:List[EmptyPointLine]):List[EmptyPointLine] = polyList match {
      case x1::x2::xs => {
        val p21 = x1
        val p22 = x2
        val collisionResult = handleLineLine(p11, p12, p21, p22)
        collisionResult match {
          case Empty => collideAll(x2::xs, res)
          case pRes @ Point(_) => collideAll(x2::xs, res :+ pRes)
          case lineRes @ Line(_, _) => List(lineRes)
        }
      }
      case x::xs => res
      case Nil => res
    }

    val insideEnds = {
      List(handlePointPolygon(Point(p11), poly)) ++
      List(handlePointPolygon(Point(p12), poly))
    }

    val collisions = collideAll(polygonListPlusHead, List.empty)

    val finalResult = collisions match {
      case List(a@Line(_, _)) => a
      case _ => {
        //Make a set out of the points, thereby removing duplicates.
        val matchOnlyPoint = {case b:Point => b}: PartialFunction[EmptyPointLine, Point]
        val finalFinalRes = collisions ++ insideEnds
        finalFinalRes.toSet.collect(matchOnlyPoint).take(2).toList match {
          case List(a, b) => Line.create(a.p, b.p)
          case List(a) => Point(a.p)
          case _ => Empty
        }
      }
    }
    Right(finalResult)
  }

  /** Finds the intersection between two polygons, that may be full or not-full.
    *
    * @param poly1 the first polygon
    * @param poly2 the second polygon
    * @param poly1Full whether the first polygon is full
    * @param poly2Full whether the second polygon is full
    * @param poly1ApproxBoundingBox the optional bounding box of the first polygon, to avoid possible recalculation
    * @param poly2ApproxBoundingBox the optional bounding box of the second polygon, to avoid possible recalculation
    * @return the intersection of the polygons
    */
  def intersection(poly1:ConvexCCWPolygon, poly2:ConvexCCWPolygon, poly1Full:Boolean, poly2Full:Boolean,
      poly1ApproxBoundingBox:Option[BoundingBox] = None, poly2ApproxBoundingBox:Option[BoundingBox] = None):Either[Boolean, ConvexCCWPolygon] = {

    val poly1Points = poly1.points
    val poly2Points = poly2.points

    val boundingBoxesIntersect = {
      (poly1, poly2) match { //Ensure that neither of the polygons are empty.
        case (Empty, _) | (_, Empty) => false
        case _ => {

          //Match with the approximate bounding box if existing, and if not or no approximate, check actual bounding box.
          (poly1ApproxBoundingBox, poly2ApproxBoundingBox) match {
            case (None, None) => {
              bBoxNonemptyPolygon(poly1Points) intersects bBoxNonemptyPolygon(poly2Points)
            }
            case (Some(approx1), None) => {
              val boundingBox2 = bBoxNonemptyPolygon(poly2Points)
              (approx1 intersects boundingBox2) && (bBoxNonemptyPolygon(poly1Points) intersects boundingBox2)
            }
            case (None, Some(approx2)) => {
              val boundingBox1 = bBoxNonemptyPolygon(poly1Points)
              (boundingBox1 intersects approx2) && (boundingBox1 intersects bBoxNonemptyPolygon(poly2Points))
            }
            case (Some(approx1), Some(approx2)) => {
              (approx1 intersects approx2) && (bBoxNonemptyPolygon(poly1Points) intersects bBoxNonemptyPolygon(poly2Points))
            }
          }
        }
      }
    }

    if (!boundingBoxesIntersect) {
      Left(false)
    }
    else {
      (poly1, poly2) match {
        case (po1:Polygon, po2:Polygon) => {
          val bothFull = poly1Full && poly2Full

          val (originIndex1, originIndex2) = {
            val originIndex1 = poly1Points.zip(0 until poly1Points.length).foldLeft(po1.p1, 0)(chooseLeftmostUpperPoint)._2
            val originIndex2 = poly2Points.zip(0 until poly2Points.length).foldLeft(po2.p1, 0)(chooseLeftmostUpperPoint)._2
            (originIndex1, originIndex2)
          }

          val collisionSegmentsFinder = new CollisionSegmentsFinder(poly1Points, poly2Points, originIndex1, originIndex2)

          val collisionSegmentsO = collisionSegmentsFinder.getCollisionSegments

          val intersectionConstructionResult = collisionSegmentsO match {
            case None => {
                //No need to check for one polygon inside the other,
                //since none means that there is no polygon intersection at all.
              Empty
            }
            case Some(collisionSegments) => {
              val intersectionFinder = new IntersectionFromCollisionSegments(collisionSegments, poly1Points, poly2Points)
              val result = intersectionFinder.getIntersectionFromCollisionSegments
              result
            }
          }

          Right(intersectionConstructionResult)
        }
        case (line@Line(_, _), poly@Polygon(_, _, _, _)) => {
          handleLinePoly(line, poly)
        }
        case (poly@Polygon(_, _, _, _), line@Line(_, _)) => {
          handleLinePoly(line, poly)
        }
        case (point:Point, poly:Polygon) => Right(handlePointPolygon(point, poly))
        case (poly:Polygon, point:Point) => Right(handlePointPolygon(point, poly))
        case (Line(p11, p12), Line(p21, p22)) => {
          Right(handleLineLine(p11, p12, p21, p22))
        }
        case (line:Line, point:Point) => Right(handlePointLine(point, line))
        case (point:Point, line:Line) => Right(handlePointLine(point, line))
        case (Point(a), Point(b)) => {
          Right(if (a == b) Point(a) else Empty)
        }
        case (Empty, _) | (_, Empty) => {
          Right(Empty)
        }
      }
    }
  }
}
