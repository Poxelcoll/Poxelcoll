package poxelcoll.mask

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

import poxelcoll.geometry.convexccwpolygon.NonemptyConvexCCWPolygon
import poxelcoll.IP
import poxelcoll.BoundingBox
import poxelcoll.binaryimage.BinaryImage
import poxelcoll.binaryimage.SimpleBinaryImage
import poxelcoll.P
import poxelcoll.geometry.convexccwpolygon.ConvexHull
import poxelcoll.geometry.convexccwpolygon.Empty
import poxelcoll.geometry.convexccwpolygon.Line
import poxelcoll.geometry.convexccwpolygon.Point
import poxelcoll.geometry.convexccwpolygon.Polygon
import poxelcoll.binaryimage.BinaryImageFactory

/**
 * A mask consists of either a binary image and an approximating convex hull and axis-aligned bounding box,
 * or a full convex hull and an approximating axis-aligned bounding box.
 *
 * A mask may not be empty. An empty mask can never have collisions, and is therefore not allowed.
 */
final case class Mask(

  /**
   * The origin point of the mask.
   *
   * If a point in the mask has position P(1, 2),
   * and the origin point is P(5, 5), the effective position of the point
   * in the mask is P(-4, -3).
   *
   * @return the origin point of the mask.
   */
  val origin: P,

  /**
   * The axis-aligned bounding box of the mask.
   *
   * The bounding box may never under-approximate the binary image
   * and the convex hull.
   *
   * @return the axis-aligned, over-approximating bounding box
   */
  val boundingBox: BoundingBox,

  /**
   * Either a over-approximating convex hull of the binary image if the binary image is present,
   * or a shape representing the mask accurately if the binary image is not present.
   *
   * @return the convex hull of the binary image or the convex hull representing the mask
   */
  val convexHull: NonemptyConvexCCWPolygon,

  /**
   * The binary image if present, or none if not.
   *
   * @return Some binary image or None
   */
  val binaryImageO: Option[BinaryImage]) {

  /**
   * Whether the mask is full or not. Equivalent to whether it does not have a binary image or not.
   *
   * @return whether the mask is full or not
   */
  def isPolygonFull = binaryImageO == None
}

/** Contains factory methods for Mask construction. */
object Mask {

  /**
   * Creates a mask from the given image source, origin and binary image factory,
   * or none if input arguments are invalid, such as if the source is empty.
   *
   * @param imageSource the source of the image, a sequence of rows. The number of rows
   *                    must be non-zero, and the row length must be consistent
   * @param origin the origin point of the mask
   * @param binaryImageFactory the factory for creating the binary image
   * @return binary image if input valid, else none
   */
  def createMaskOFromImageSource(
    imageSource: IndexedSeq[IndexedSeq[Boolean] with Immutable] with Immutable,
    origin: P,
    binaryImageFactory: BinaryImageFactory = SimpleBinaryImage): Option[Mask] = {

    val binaryImageO: Option[BinaryImage] = binaryImageFactory.createO(imageSource)

    binaryImageO match {
      case Some(binaryImage) => {
        val width = binaryImage.width
        val height = binaryImage.height

        //Make positions out of points.
        val points =
          (for (x <- 0 until width; y <- 0 until height if binaryImage.hasPoint(x, y))
            yield Seq((x, y), (x + 1, y), (x, y + 1), (x + 1, y + 1))).flatten
        val coords = points.map(a => new P(a._1, a._2)).toArray

        if (coords.isEmpty) {
          //The mask is empty, so don't continue.
          None
        } else {
          val boundingBox = {
            val (xMin, xMax) = {
              val xs = points.map(_._1)
              (xs.min, xs.max)
            }
            val (yMin, yMax) = {
              val ys = points.map(_._2)
              (ys.min, ys.max)
            }
            BoundingBox(P(xMin, yMin), P(xMax, yMax))
          }

          val someConvexHull = ConvexHull.calculateConvexHull(coords)
          val nonemptyConvexHull = someConvexHull match {
            case a: NonemptyConvexCCWPolygon => a
            case _ => throw new IllegalStateException("The coordinates are not empty at this point, the hull should not be empty: " + points + ", " + someConvexHull)
          }

          val mask = Mask(origin, boundingBox, nonemptyConvexHull, Some(binaryImage))
          Some(mask)
        }
      }
      case None => {
        throw new IllegalArgumentException("Image source was not valid: " + imageSource)
      }
    }
  }

  /**
   * Given a non-empty convex hull, create a mask.
   *
   * @param polygon the non-empty convex hull
   * @param origin the origin
   * @return the created mask
   */
  def createMaskFromPolygon(polygon: NonemptyConvexCCWPolygon, origin: P): Mask = {

    polygon match {
      case Point(_) | Line(_, _) | Polygon(_, _, _, _) => {

        val boundingBox = polygon match {
          case Point(p1) => BoundingBox(p1, p1)
          case Line(p1, p2) => {
            val pMinX = math.min(p1.x, p2.x)
            val pMinY = math.min(p1.y, p2.y)
            val pMaxX = math.max(p1.x, p2.x)
            val pMaxY = math.max(p1.y, p2.y)
            BoundingBox(
              P(pMinX, pMinY),
              P(pMaxX, pMaxY))
          }
          case Polygon(p1, p2, p3, rest) => {
            val points = Seq(p1, p2, p3) ++ rest
            val pMinX = points.map(_.x).min
            val pMinY = points.map(_.y).min
            val pMaxX = points.map(_.x).max
            val pMaxY = points.map(_.y).max
            BoundingBox(
              P(pMinX, pMinY),
              P(pMaxX, pMaxY))
          }
        }

        Mask(origin, boundingBox, polygon, None)
      }
    }

  }

  def createSimpleRectangle(
    binaryImageFactory: BinaryImageFactory = SimpleBinaryImage): Mask = {

    val width = 30
    val height = 15
    val origin = P(15, 45)

    val imageSource: IndexedSeq[IndexedSeq[Boolean] with Immutable] with Immutable = for (y <- 0 until height) yield {
      for (x <- 0 until width) yield {
        if (x >= 3 && x <= 28 && y >= 2 && y <= 11) {
          true
        } else {
          false
        }
      }
    }

    createMaskOFromImageSource(imageSource, origin) match {
      case Some(mask) => mask
      case None => throw new IllegalStateException("Creation of simple rectangle should never fail, but it did: " + origin + ", " + origin)
    }
  }

  def createSimplePolygonWithHoles(
    binaryImageFactory: BinaryImageFactory = SimpleBinaryImage): Mask = {

    val width = 30
    val height = 15
    val origin = P(15, 45)

    val imageSource: IndexedSeq[IndexedSeq[Boolean] with Immutable] with Immutable = for (y <- 0 until height) yield {
      for (x <- 0 until width) yield {
        if (x >= 3 && x <= 28 && y >= 2 && y <= 11 && x > y) {
          x <= 15 || x >= 18 || y <= 5 || y >= 8
        } else {
          false
        }
      }
    }

    createMaskOFromImageSource(imageSource, origin, binaryImageFactory) match {
      case Some(mask) => mask
      case None => throw new IllegalStateException("Creation of simple rectangle should never fail, but it did: " + imageSource + ", " + origin)
    }
  }

  def createL(
    binaryImageFactory: BinaryImageFactory = SimpleBinaryImage): Mask = {

    val width = 30
    val height = 30
    val origin = P(0, 0)

    val imageSource: IndexedSeq[IndexedSeq[Boolean] with Immutable] with Immutable = for (y <- 0 until height) yield {
      for (x <- 0 until width) yield {
        //        if (x >= 3 && x <= 28 && y >= 2 && y <= 11 && x > y) {
        //          x <= 15 || x >= 18 || y <= 5 || y >= 8
        //        }
        //        else {
        //          false
        //        }
        //        (x >= y && x + y <= 30 && y >= 5) ||
        //        (x + y >= 15 && y <= 10 && x - 15 <= y)
        (x >= 5 && x <= 10 && y >= 0 && y <= 30) || (x >= 5 && x <= 30 && y >= 0 && y <= 5)
      }
    }

    createMaskOFromImageSource(imageSource, origin, binaryImageFactory) match {
      case Some(mask) => mask
      case None => throw new IllegalStateException("Creation of simple rectangle should never fail, but it did: " + imageSource + ", " + origin)
    }
  }

  def createPentagon = createMaskFromPolygon(Polygon.create(P(0, 0), P(10, 0), P(15, 10), scala.collection.immutable.Seq(P(5, 15), P(-5, 10))), P(0, 0))
}
