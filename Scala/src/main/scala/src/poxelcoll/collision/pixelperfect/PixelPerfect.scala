package poxelcoll.collision.pixelperfect

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

import poxelcoll.IP
import poxelcoll.P
import poxelcoll.geometry.convexccwpolygon.NonemptyConvexCCWPolygon
import poxelcoll.geometry.convexccwpolygon.Polygon
import scala.util.Sorting

/** The pixel-perfect collision detection supports collision through several functions. */
object PixelPerfect {

  //Shamelessly copy-wasted from Wikipedia.
  /*
     function line(x0, x1, y0, y1)
     boolean steep := abs(y1 - y0) > abs(x1 - x0)
     if steep then
         swap(x0, y0)
         swap(x1, y1)
     if x0 > x1 then
         swap(x0, x1)
         swap(y0, y1)
     int deltax := x1 - x0
     int deltay := abs(y1 - y0)
     int error := deltax / 2
     int ystep
     int y := y0
     if y0 < y1 then ystep := 1 else ystep := -1
     for x from x0 to x1
         if steep then plot(y,x) else plot(x,y)
         error := error - deltay
         if error < 0 then
             y := y + ystep
             error := error + deltax
   */
  /** An implementation of Bresenham's line algorithm that finds a line segment of points given two integer points.
    *
    * The arguments given to it must be different.
    *
    * @param start integer points different from "end"
    * @param end integer point different from "start"
    * @return a sequence of connected, consecutive points in a line from "start" to "end"
    */
  private[this] def bresenhamsLine(start:IP, end:IP) = {

    val steep = math.abs(end.y-start.y) > math.abs(end.x - start.x)
    val (x0, y0, x1, y1) = {

      val (x_0, y_0, x_1, y_1) = if (steep) (start.y, start.x, end.y, end.x) else (start.x, start.y, end.x, end.y)

      val (x0, x1, y0, y1) = if (x_0 > x_1) (x_1, x_0, y_1, y_0) else (x_0, x_1, y_0, y_1)

      (x0, y0, x1, y1)
    }
    val deltax = x1 - x0
    val deltay = math.abs(y1-y0)
    val ystep = if (y0 < y1) 1 else -1

    var error = deltax / 2
    var y = y0
    if (steep) {
      for (x <- x0 to x1) yield {
        val returnValue = IP(y,x)
        error -= deltay
        if (error < 0) {
          y += ystep
          error += deltax
        }
        returnValue
      }
    }
    else {
      for (x <- x0 to x1) yield {
        val returnValue = IP(x,y)
        error -= deltay
        if (error < 0) {
          y += ystep
          error += deltax
        }
        returnValue
      }
    }
  }

  /** Round and convert to integer. */
  def r(a:Double) = math.round(a).toInt

  /** Given a line represented by two (possibly identical) points and the middle point from its original convex polygon,
    * find a sequence of integer points constituting a line segment (or in some cases just a point).
    *
    * It should be noted that the line will be moved strictly away from the middle point
    * (unless the middle point is on the line).
    *
    * The current implementation is highly vulnerable to numerical stability issues.
    *
    * @param c1 point one of the line segment/point
    * @param c2 point two of the line segment/point
    * @param middle middle of the original convex polygon from which this comes
    * @return a sequence of points constituting a line from c1 to c2, but moved away from the middle
    */
  private[this] def lineToPoints(c1:P, c2:P, middle:P) = {

    //Overview: Over-approximate the convex hull by placing the line 1 or sqrt(2) pixel moved,
    //in the direction away from the middle.

    val lineAboveMiddle = {

      //Double-precision-version, such that the middle isn't rounded off to lie on the line segment.
      //or be on the wrong side of the line segment.
      //NOTE: This part is highly vulnerable to numerical stability issues.
      val (x1, y1, x2, y2, xm, ym) = (c1.x, c1.y, c2.x, c2.y, middle.x, middle.y)

      val xD = x2-x1
      val yD = y2-y1

      if (xD != 0) {
        val ym2 = (xm - x1) * yD / xD + y1
        if (ym2 > ym) 1
        else if (ym2 < ym) -1
        else 0
      }
      else { //A vertical line is above the middle iff it is to the right of the middle.
        if (x1 > xm) 1
        else if (x1 < xm) -1
        else 0
      }
    }

    val (x1, y1, x2, y2, xm, ym) = (r(c1.x), r(c1.y), r(c2.x), r(c2.y), r(middle.x), r(middle.y))

    val points = Set(IP(x1, y1), IP(x2, y2))

    val xD = x2-x1
    val yD = y2-y1

    val line = (xD, yD) match {
      case (0, 0) => Set(IP(x1, y1))
      case (0, _) => {
        val n1 = IP(x1 + lineAboveMiddle, y1)
        val n2 = IP(x2 + lineAboveMiddle, y2)
        bresenhamsLine(n1, n2) //NOTE: Call only with different coordinates.
      }
      case (_, 0) => {
        val n1 = IP(x1, y1 + lineAboveMiddle)
        val n2 = IP(x2, y2 + lineAboveMiddle)
        bresenhamsLine(n1, n2) //NOTE: Call only with different coordinates.
      }
      case (_, _) if (xD > 0 && yD > 0) || (xD < 0 && yD < 0) => {
        val n1 = IP(x1 - lineAboveMiddle, y1 + lineAboveMiddle)
        val n2 = IP(x2 - lineAboveMiddle, y2 + lineAboveMiddle)
        bresenhamsLine(n1, n2) //NOTE: Call only with different coordinates.
      }
      case (_, _) if (xD > 0 && yD < 0) || (xD < 0 && yD > 0) => {
        val n1 = IP(x1 + lineAboveMiddle, y1 + lineAboveMiddle)
        val n2 = IP(x2 + lineAboveMiddle, y2 + lineAboveMiddle)
        bresenhamsLine(n1, n2) //NOTE: Call only with different coordinates.
      }
      case _ => throw new IllegalStateException("This part should not be reachable.")
    }

    points ++ line
  }

  /** Find the point outline of a convex polygon.
    *
    * @param convexHullPolygon the convex polygon
    * @return the outline
    */
  def findOutline(convexHullPolygon:NonemptyConvexCCWPolygon):Set[IP] = {

    val middlePoint = convexHullPolygon.middlePoint

    def findOutline2(coordinates:List[P]):Set[IP] = coordinates.toList match {
      case a::b::cs => {
        lineToPoints(a,b, middlePoint) ++ findOutline2(b::cs)
      }
      case a::Nil => lineToPoints(a,a, middlePoint)
      case Nil => Set.empty
    }

    //A point is added if the convex polygon is an actual 3-points-or-more polygon,
    //such that the line from the end to the start is also included.
    val polygonWithExtraEnd = {
      convexHullPolygon match {
        case Polygon(a, b, c, d) => {
          Polygon.createUtterlyUnsafelyNotChecked(a, b, c, scala.collection.immutable.Seq(d :+ a:_*))
        }
        case a => a
      }
    }

    findOutline2(polygonWithExtraEnd.points.toList)
  }

  /** Find the point outline of a convex polygon, or stop if the test function yields true for a point on the outline.
    *
    * @param convexHullPolygon the convex polygon
    * @param testFunction a test function to test a point, for instance to test if a binary image is on or off at the given point
    * @return the outline
    */
  def findOutlineStoppage(convexHullPolygon:NonemptyConvexCCWPolygon, testFunction:IP => Boolean):Either[Set[IP], Boolean] = {

    val middlePoint = convexHullPolygon.middlePoint

    def findOutlineStoppage2(coordinates:List[P], testFunction:IP => Boolean):Either[Set[IP], Boolean] = coordinates.toList match {
      case a::b::cs => {
        val linePoints = lineToPoints(a,b, middlePoint)
        if (linePoints.exists(d => testFunction(d))) {
          Right(true)
        }
        else {
          findOutlineStoppage2(b::cs, testFunction) match {
            case Left(e) => Left(linePoints ++ e)
            case Right(e) => Right(e)
          }
        }
      }
      case a::Nil => {
        val linePoints = lineToPoints(a,a, middlePoint)
        if (linePoints.exists(d => testFunction(d))) {
          Right(true)
        }
        else {
          Left(linePoints)
        }
      }
      case Nil => {
        Left(Set.empty)
      }
    }

    //A point is added if the convex polygon is an actual 3-points-or-more polygon,
    //such that the line from the end to the start is also included.
    val polygonWithExtraEnd = {
      convexHullPolygon match {
        case Polygon(a, b, c, d) => {
          Polygon.createUtterlyUnsafelyNotChecked(a, b, c, scala.collection.immutable.Seq(d :+ a:_*))
        }
        case a => a
      }
    }

    findOutlineStoppage2(polygonWithExtraEnd.points.toList, testFunction)
  }

  /** Given some set of points forming an outline, fill that outline.
    *
    * If the outline is not connected, the filling is not well-defined.
    *
    * @param outline set of points forming an outline.
    * @return if outline is connected, a filled outline, else not well-defined
    */
  def fillOutline(outline:Set[IP]) = {
    val horizontalLines = outline.groupBy(_.y).map{case (a,b) => a -> Sorting.stableSort(b.map(_.x).toSeq)}
    horizontalLines.map{case (a,b) => b match {
      case Array(c) => a -> b
      case Array() => throw new IllegalStateException("The array may not be empty at this point.")
      case _ => {
        a -> (b.head to b.last).toArray
      }
    }}
  }

  /** Given some set of points forming an outline, fill the outline, and test if the test function holds for any point.
    *
    * The test function is generally applied before the whole outline is formed,
    * and if it becomes true at any point, the function returns true.
    *
    * If the outline is not connected, the filling is not well-defined.
    *
    * @param outline set of points forming an outline.
    * @param testFunction a test function to test a point, for instance to test if a binary image is on or off at the given point
    * @return whether or not the test function holds for any point in the filled outline.
    *         The testing may not test everything if the outline is not well-defined
    */
  def fillOutlineStoppage(outline:Set[IP], testFunction: IP => Boolean):Boolean = {

    val horizontalLines = outline.groupBy(_.y).map{case (a,b) => a -> Sorting.stableSort(b.map(_.x).toSeq)}
    horizontalLines.exists{case (y,xs) =>
      (xs.head to xs.last).exists(x => testFunction(IP(x, y)))
    }
  }

  /** Given an area defined by a non-empty convex polygon, test if any of the points in it yields true.
    *
    * The method guarantees correct handling of pixels in regards to that pixels are defined
    * as areas, and that the index (x, y) in a binary image refers to the area [x, x+1], [y, y+1].
    *
    * @param nonemptyConvexPolygon the area to test for
    * @param testFunction the test function
    * @return whether any point in the area yields true for the test function
    */
  def collisionTest(nonemptyConvexPolygon:NonemptyConvexCCWPolygon, testFunction:IP => Boolean):Boolean = {

    //NOTE: Translate the polygon a little backwards, such that the pixel test will be correct.
    val correctedPolygon = nonemptyConvexPolygon.translate(P(-0.5, -0.5))

    if (nonemptyConvexPolygon.points.size < 1)
      false
    else {
      val outlineResult = findOutlineStoppage(correctedPolygon, testFunction)

      outlineResult match {
        case Right(bool) => bool
        case Left(outline) => fillOutlineStoppage(outline, testFunction)
      }
    }
  }
}
