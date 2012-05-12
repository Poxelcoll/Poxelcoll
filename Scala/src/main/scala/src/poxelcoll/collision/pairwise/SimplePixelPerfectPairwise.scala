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

import poxelcoll.collision.pixelperfect.PixelPerfect
import poxelcoll.geometry.convexccwpolygon.ConvexCCWPolygon
import poxelcoll.geometry.convexccwpolygon.PolygonIntersection
import poxelcoll.mask.Mask
import poxelcoll.CollisionInfo
import poxelcoll.P
import poxelcoll.IP
import poxelcoll.geometry.matrix.P3
import poxelcoll.geometry.convexccwpolygon.Empty
import poxelcoll.geometry.convexccwpolygon.Line
import poxelcoll.geometry.convexccwpolygon.Point
import poxelcoll.geometry.convexccwpolygon.Polygon
import poxelcoll.BoundingBox
import poxelcoll.geometry.matrix.Transformation

/** The simple pairwise collision detection takes 2 pairs and determines if they collide.
  *
  * The method takes everything into account, including transformation (rotation, translation, scaling),
  * binary images, filled objects, etc.
  *
  * Strictly over-approximating bounding boxes are used to speed up collision detection
  * by excluding collision objects that do not overlap.
  *
  * '''Method'''
  *
  * The implementation first checks the approximate bounding box found by
  * transforming the bounding box according to the collision objects transformation data,
  * and then finding the axis-aligned bounding box of the transformed bounding box.
  * This is efficient, but not very precise.
  * If they still collide, the detection goes on, else it stops with false.
  * Then the convex hulls of the collision objects is transformed in linear time of the points on the hulls themselves.
  * The intersection of the convex hulls are then found, again in linear time of the points on the hulls themselves.
  *
  * If the intersection is found to be empty, the objects do not collide.
  * Else, all the points that overlaps the intersection is found:
  * Now, for each of these points, the point is transformed back to each of the
  * coordinate systems of the original collision objects, and the binary
  * images of each object is checked. If both are filled, a collision is
  * decided to have occurred, and the algorithm stops with true.
  * If this collision test fails for all points overlapping with the intersection,
  * it is decided that there is no collision.
  *
  * In general, the above method stops as soon as a colliding pixel has been found.
  * Furthermore, if both of the collision objects are filled (ie. they have no binary image),
  * the method stops the moment it has been decided whether or not there is an intersection.
  *
  * This method is generally very performant if the collision objects (including their
  * binary images) are well approximated by their convex hulls.
  * For instance, two triangles that are transformed and scale will generally
  * be checked very quickly, because either their convex hulls will find no
  * intersection quickly (meaning there is no intersection, and thus no need to go on),
  * or else the intersection will consist of on-pixels, and the pixel-perfect detection
  * will stop quickly.
  * Conversely, if collision objects are not well approximated by their convex hulls,
  * collision detection will generally not be so performant, since the convex hulls
  * may have a large intersection where there is few or no on-pixels.
  *
  * '''Pixel-perfect collision detection, precision and scaling'''
  *
  * For the pixel-perfect collision detection,
  * a scaling-invariant method is used. This means that collision detection will generally
  * only be precise and performant as long as the collision objects are not scaled.
  * If the objects are scaled up, they will not loose precision, but they may become less
  * performant. Conversely, if the objects are scaled down, they will loose precision,
  * and may become more performant. The performance and precision of mixed scaling
  * (such as one object scaling up and the other scaling down,
  * or scaling up along one axis and down along the other)
  * is generally difficult to predict, but can be assumed in general to be less
  * performant and precise than if no scaling is present.
  */
object SimplePixelPerfectPairwise extends Pairwise {

  /** Given a set of points that form a valid convex polygon, that is either clockwise or counter-clockwise,
    * return a counter-clockwise convex polygon.
    *
    * @param points CW or CCW convex points
    * @return CCW convex polygon
    */
  private[this] def assumingValidConvexPolygonPointsTransformToCCWEvenIfCW(points:IndexedSeq[P]):ConvexCCWPolygon = {
    val size = points.size
    size match {
      case 0 => Empty
      case 1 => Point(points.head)
      case 2 => Line.create(points.head, points.last)
      case _ => {
        //Get the first 3 points, and check their direction.

        val p1 = points.head
        val p2 = points.drop(1).head
        val p3 = points.drop(2).head

        val v1 = p2 - p1
        val v2 = p3 - p1

        v1 X v2 match {
          case 0.0 => throw new IllegalStateException("A valid convex polygon will never have 3 points on the same line in the convex hull: " + points)
          case a if a > 0.0 => {
            //The polygon is CCW, do nothing.
            Polygon.createUtterlyUnsafelyNotChecked(
              points match {
                case p:IndexedSeq[_] with Immutable => p
                case _ => scala.collection.immutable.IndexedSeq[P](points:_*)
              }
            )
          }
          case _ => { //a < 0.0
            //The polygon is CW, reverse in other to get CCW.
            val pointsReverse = points.reverse
            Polygon.createUtterlyUnsafelyNotChecked(
                pointsReverse match {
                case p:IndexedSeq[_] with Immutable => p
                case _ => scala.collection.immutable.IndexedSeq[P](pointsReverse:_*)
              }
            )
          }
        }
      }
    }
  }

  /** Check whether a point is contained in a binary image.
    *
    * @param image a binary image
    * @param v a point which has a superfluous third coordinate, and which coordinates may be outside the images dimension
    * @return whether the image contains the point
    */
  private[this] def checkImage(image:Mask, v:P3) = image.binaryImageO match {
    case Some(binaryImage) => {
      val point = IP(v.x.round.toInt, v.y.round.toInt)
      val x = point.x
      val y = point.y

      x >= 0 && x < binaryImage.width &&
      y >= 0 && y < binaryImage.height &&
      binaryImage.hasPoint(x, y)
    }
    case None => true
  }

  /** For 2 images and 2 transformation matrices, give a function that test whether a given point is contained in both images.
    *
    * The transformation matrices is used to map from some coordinate system to that of the images.
    *
    * @param image1 first binary image to test whether a point is contained in
    * @param image2 second binary image to test whether a point is contained in
    * @param inv1 transform given points to the coordinate system of the first image
    * @param inv2 transform given points to the coordinate system of the second image
    * @param point whether a point is contained in the given function
    * @return first currying yields a test function that yields if a point is contained in both images using the given transformation matrices
    */
  private[this] def generalTestFunction(image1:Mask, image2:Mask, inv1:Transformation.MT, inv2:Transformation.MT)(point:IP):Boolean = {
    val vector = P3(point.x, point.y, 1.0)

    val imageVector1 = inv1 * vector
    val imageVector2 = inv2 * vector

    checkImage(image1, imageVector1) && checkImage(image2, imageVector2)
  }

  def testForCollision(collInfo1:CollisionInfo, collInfo2:CollisionInfo):Boolean = {

    val mask1 = collInfo1.mask
    val mask2 = collInfo2.mask

    val transformationMatrix1 = Transformation.getTransformationMatrix(collInfo1)
    val transformationMatrix2 = Transformation.getTransformationMatrix(collInfo2)

    val (inv1O, inv2O) = (transformationMatrix1.inverseO, transformationMatrix2.inverseO)
    (inv1O, inv2O) match {
      case (None, _) | (_, None) => false //If the inverse is not well-defined, there is no collision (no inverse == line without width or similar).
      case (Some(inv1), Some(inv2)) => { //There is a well-defined inverse, continue.

        val transConHull1 = assumingValidConvexPolygonPointsTransformToCCWEvenIfCW(
          transformationMatrix1.transformPoints(mask1.convexHull.points)
        )
        val transConHull2 = assumingValidConvexPolygonPointsTransformToCCWEvenIfCW(
          transformationMatrix2.transformPoints(mask2.convexHull.points)
        )
        val approxBoundingBox1 = Transformation.approximateBoundingBox(transformationMatrix1, mask1.boundingBox)
        val approxBoundingBox2 = Transformation.approximateBoundingBox(transformationMatrix2, mask2.boundingBox)

        //If both full, check for intersection.
        //If not both full, find the intersection.
        val otherIntersection = PolygonIntersection.intersection(
          transConHull1, transConHull2,
          mask1.isPolygonFull, mask2.isPolygonFull,
          Some(approxBoundingBox1), Some(approxBoundingBox2)
        )

        otherIntersection match {
          case Right(collisionIntersection:ConvexCCWPolygon) => {

            val testFunction = generalTestFunction(mask1, mask2, inv1, inv2) _

            //Given the intersection, test the pixels by taking a pixel in the intersection polygon,
            //and using the inverse transformation matrices to get the corresponding point in the
            //binary image (or if full, just true).
            collisionIntersection match {
              case a@Point(_) => PixelPerfect.collisionTest(a, testFunction)
              case a@Line(_, _) => PixelPerfect.collisionTest(a, testFunction)
              case a@Polygon(_, _, _, _) => PixelPerfect.collisionTest(a, testFunction)
              case Empty => false
            }

          }
          case Left(hasIntersection) => hasIntersection
        }
      }
    }
  }
}
