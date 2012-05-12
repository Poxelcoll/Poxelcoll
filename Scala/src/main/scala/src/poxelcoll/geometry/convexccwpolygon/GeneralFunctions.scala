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

/** General functions for handling intersection between different polygon primitives. */
object GeneralFunctions {

  //This finds the collision between 2 line segments, if any collision exists.
  //The line segments are denoted by the two points
  //poly(i), poly(next(i)).
  //The line segments are directed.
  //This means that the first point, poly(i), counts as nothing in regards to collisions.
  //The second point, poly(next(i)), is called the "head".
  /** Finds the collision between 2 directed line segments, if any exists.
    *
    *
    * A directed line segment is defined as the vector consisting of the points {first, last},
    * where first = poly*Points(i*) and last = poly*Points(next*(i*)),
    * and where * is the number 1 or 2.
    * The first point is not considered part of the directed line segment.
    * Thus, if the directed line segments only overlap in one or two of the first points,
    * there is no collision overall.
    *
    * If there is more than one collision, the overlapping last point(s) are used for the
    * collision point.
    *
    * @param i1 index of the first point for the first polygon
    * @param i2 index of the first point for the second polygon
    * @param poly1Points the points of the first polygon
    * @param poly2Points the points of the second polygon
    * @return if the directed line segments indicated by the indices overlap, the corresponding collision segment, else an empty list
    */
  def getCollisionDirectedLineSegment(i1:Int, i2:Int, poly1Points:IndexedSeq[P], poly2Points:IndexedSeq[P]):CollisionSegments = {

    def next(a:Int, size:Int) = (a + 1) % size
    def prev(a:Int, size:Int) =  if (a - 1 < 0) size - 1 else a - 1

    val size1 = poly1Points.size
    val size2 = poly2Points.size

    def next1(i:Int) = next(i, size1)
    def next2(i:Int) = next(i, size2)

    val p11 = poly1Points(i1)
    val p12 = poly1Points(next1(i1))
    val p21 = poly2Points(i2)
    val p22 = poly2Points(next2(i2))

    val l1 = p12 - p11
    val l2 = p22 - p21

    //If the lines not only crosses, but overlap, then only the heads.
    //Else, normal cross.
    //Remember that ends does not count.

    //Line segment intersection.
    //Treat the lines as vector lines.

    val denominator = l1.y * l2.x - l1.x * l2.y
    denominator match {
      case 0.0 => {
        //No single point of intersection.

        //Finding the distance between a point of one line segment,
        //and the line of the other line segment.

        val perpenL2 = P(-l2.y, l2.x) / l2.norm
        val v3 = p21 - p11

        val dist = math.abs(perpenL2 dot v3)

        dist match {
          case 0.0 => {
            //Find relative position of heads.
            val u1 = {//Position of head 2.
              if (l1.x != 0.0) {
                (p22.x - p11.x) / l1.x
              }
              else {
                (p22.y - p11.y) / l1.y
              }
            }
            val u2 = {//Position of head 1.
              if (l2.x != 0.0) {
                (p12.x - p21.x) / l2.x
              }
              else {
                (p12.y - p21.y) / l2.y
              }
            }
            if ( (u1 > 0.0 && u1 <= 1.0) || (u2 > 0.0 && u2 <= 1.0) ) {
              //At least 1 heads overlap.
              val crossHeadPoint = {
                if (u1 > 0.0 && u1 <= 1.0) p11 + (l1 * u1)
                else p21 + (l2 * u2)
              }
              List(CollisionSegment(i1, i2, crossHeadPoint))
            }
            else {
              List.empty
            }
          }
          case _ => {
            List.empty
          }
        }
      }
      case _ => {
        val u1 = (- p21.x * l2.y + p11.x * l2.y + (p21.y - p11.y) * l2.x) / denominator
        val u2 = (- p21.x * l1.y + p11.x * l1.y + (p21.y - p11.y) * l1.x) / denominator

        if (u1 > 0.0 && u1 <= 1.0 && u2 > 0.0 && u2 <= 1.0) {
          List(CollisionSegment(i1, i2, p11 + (l1 * u1)))
        }
        else {
          List.empty
        }
      }
    }
  }

  /** Finds the intersection between two line segments.
    *
    * A(n) (undirected) line segment includes the first point, the last point,
    * and all the points between them. The first point and the last point is
    * never equal.
    *
    * @param p11 the first point of the first line segment, not equal p12
    * @param p12 the last point of the first line segment, not equal p11
    * @param p21 the first point of the second line segment, not equal p22
    * @param p22 the last point of the second line segment, not equal p21
    * @return the intersection of the line segments
    */
  def handleLineLine(p11:P, p12:P, p21:P, p22:P):EmptyPointLine = {

    val l1 = p12 - p11
    val l2 = p22 - p21

    //Line segment intersection.
    //Treat the lines as vector lines.

    val denominator = l2 X l1

    denominator match {
      case 0.0 => {
        //Possibly no single point of intersection.

        //Finding the distance between a point of one line segment,
        //and the line of the other line segment.

        val perpenL2 = P(-l2.y, l2.x) / l2.norm
        val v3 = p21 - p11

        val dist = math.abs(perpenL2 dot v3)

        dist match {
          case 0.0 => {
            //Find relative position of points.
            val (u11, u12) = {//Position of l2-points on l1.
              if (l1.x != 0.0) {
                (
                  (p21.x - p11.x) / l1.x,
                  (p22.x - p11.x) / l1.x
                )
              }
              else {
                (
                  (p21.y - p11.y) / l1.y,
                  (p22.y - p11.y) / l1.y
                )
              }
            }
            val (u21, u22) = {//Position of l1-points on l2.
              if (l2.x != 0.0) {
                (
                  (p11.x - p21.x) / l2.x,
                  (p12.x - p21.x) / l2.x
                )
              }
              else {
                (
                  (p11.y - p21.y) / l2.y,
                  (p12.y - p21.y) / l2.y
                )
              }
            }
            val pointsRelative = List((u11, p21), (u12, p22), (u21, p11), (u22, p12))
            val overlappingPoints = pointsRelative.filter(up => up._1 >= 0.0 && up._1 <= 1.0).map(_._2)

            overlappingPoints.size match {
              case 2 | 3 | 4 => {
                val sortedOverlappingPoints = overlappingPoints.sortWith((a, b) => {
                  val f = a.x - b.x
                  if (f != 0.0) f > 0.0
                  else {
                    val f2 = a.y - b.y
                    if (f2 != 0.0) f2 > 0.0
                    else false
                  }
                })
                val first = sortedOverlappingPoints.head
                val last = sortedOverlappingPoints.last
                Line.create(first, last)
              }
              case 1 => {
                Point(overlappingPoints.head)
              }
              case 0 => {
                Empty
              }
            }
          }
          case _ => {
            Empty
          }
        }
      }
      case _ => {
        val u1 = (- p21.x * l2.y + p11.x * l2.y + (p21.y - p11.y) * l2.x) / denominator
        val u2 = (- p21.x * l1.y + p11.x * l1.y + (p21.y - p11.y) * l1.x) / denominator

        if (u1 >= 0.0 && u1 <= 1.0 && u2 >= 0.0 && u2 <= 1.0) {
          Point(p11 + (l1 * u1))
        }
        else {
          Empty
        }
      }
    }
  }

  /** Finds the intersection between a point and a line segment.
    *
    * @param point a point
    * @param line a line
    * @return the intersection
    */
  def handlePointLine(point:Point, line:Line):EmptyPoint = {

    val p11 = line.p1
    val p12 = line.p2
    val p21 = point.p

    val v1 = p12 - p11
    val v2 = p21 - p11

    if ((v1 X v2) == 0.0) {
      val u = {
        if (v1.x != 0.0) {
          (p21.x - p11.x) / v1.x
        }
        else {
          (p21.y - p11.y) / v1.y
        }
      }

      if (u >= 0 && u <= 1.0) {
        point
      }
      else {
        Empty
      }
    }
    else {
      Empty
    }
  }
}
