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

/** General functions for convex hulls. */
final object ConvexHull {

  /** For all points with the same x-coordinate, the minimum and maximum can be represented by either One point or Two points. */
  private[this] sealed trait OneTwo
  /** One point. */
  private[this] final case class One(p:P) extends OneTwo
  /** Two different points with the same x. */
  private[this] final case class Two(p1:P, p2:P) extends OneTwo

  /** One coordinate gives one point.
    *
    * @param p a point
    * @return one point
    */
  private[this] def create(p:P) = One(p)
  /** Two coordinates gives either one point or two points.
    *
    * @param p1 the first point
    * @param p2 the second point
    * @return one or two different points, or undefined if the x-coordinate is not the same
    */
  private[this] def create(p1:P, p2:P) = {
    if (p1 == p2) One(p1)
    else Two(p1, p2)
  }

  /** Get the points as pair of minimum/maximum points
    *
    * @param oneTwo points
    * @return pair of points
    */
  private[this] def extract(oneTwo:OneTwo) = oneTwo match {
    case One(p) => (p, p)
    case Two(p1, p2) => (p1, p2)
  }

  //TODO: Documentation.
  private[this] def produceReverseResultCCW(resultReverse:List[P], inputCCW:List[P]):List[P] = (resultReverse, inputCCW) match {
    case (_, Nil) => resultReverse
    case (_, p::ps) => {
      val newResult = p +: resultReverse
      val consumedResult = consume(newResult)
      produceReverseResultCCW(consumedResult, ps)
    }
  }

  private[this] def consume(result:List[P]):List[P] = result match {
    case Nil => result
    case p::Nil => result
    case p2::p1::Nil => result
    case p3::p2::p1::pRest => {

      val v1 = p2 - p1
      val v2 = p3 - p1
      if ((v1 X v2) <= 0.0) {
        consume(p3::p1::pRest)
      }
      else {
        p3::p2::p1::pRest
      }
    }
  }

  private[this] def removeConsecutiveDuplicates(ps:List[P]):List[P] = ps match {
    case Nil => ps
    case p::Nil => ps
    case p1::p2::rest => {
      if (p1 == p2) removeConsecutiveDuplicates(p1::rest)
      else p1::removeConsecutiveDuplicates(p2::rest)
    }
  }

  private[this] def cleanHeadLast(ps:List[P]):List[P] = ps match {
    case Nil => ps
    case p::Nil => ps
    case head::rest => {
      val last = ps.last
      if (head == last) rest
      else ps
    }
  }

  //This calculates the convex hull.
  //It is meant to be robust,
  //meaning that duplicate points, empty argument input,
  //colinearity, etc.,
  //does not break the calculation of the convex hull.
  //Any input is accepted.
  /** Given a set of points, find the convex hull of those points, and return a simple, convex, CCW polygon representing the hull.
    *
    * The implementation is meant to be geometrically robust, meaning it accepts any input and gives a valid polygon.
    *
    * @param points any kind of set of points, whether it be empty, has internal duplicates, colinearity, etc.
    * @return a valid simple, convex CCW polygon representing the convex hull
    */
  def calculateConvexHull(points:Seq[P]):ConvexCCWPolygon = {

    //TODO: Extension: Consider supporting a mask or similar.
    //TODO: Optimization: Simple optimization for mask: extract the upper and lower point for each column, if any.
    //Then sort that.
    //Implement simple monotone chain. Fair time complexity (O(n log n) worst case),
    //but is not optimal, especially considering the domain (binary images),
    //which tend to be dense.
    //Possible optimizations that could be considered for implementation in the future
    //include using heuristics to remove the bulk of inner points,
    //as well as other algorithms such as Chan's algorithm and others.
    //See wikipedia on convex hull algorithms.

    val pointsLength = points.length
    pointsLength match {
      case 0 => Empty
      case 1 => Point(points.head)
      case 2 => Line.create(points.head, points.last)
      case _ => {
        val groupedPoints = points.groupBy(a => a.x).toSeq.sortWith((a, b) => a._1 < b._1).map(a => (a._1, a._2.sortWith((c, d) => c.y < d.y)))
        val trimmedPoints = groupedPoints.map{case (x, ys) =>
          ys.length match {
            case 0 => throw new IllegalStateException("0 is not an acceptable length in grouping.")
            case 1 => create(ys.head)
            case _ => create(ys.head, ys.last)
          }
        }

        val lower = trimmedPoints.map(a => extract(a)).map(a => a._1).toList
        val upper = trimmedPoints.map(a => extract(a)).map(a => a._2).toList

        val lowerDone = produceReverseResultCCW(Nil, lower).reverse
        val upperDoneCCW = produceReverseResultCCW(Nil, upper.reverse).reverse

        val finalPoints = cleanHeadLast(removeConsecutiveDuplicates(lowerDone ++ upperDoneCCW))

        finalPoints match {
          case Nil => throw new IllegalStateException("The final points may not be empty at this point.")
          case p::Nil => Point(p)
          case p1::p2::Nil => Line.create(p1, p2)
          case p1::p2::p3::pRest => {
            Polygon.createUtterlyUnsafelyNotChecked(p1, p2, p3, pRest)
          }
        }
      }
    }
  }
}
