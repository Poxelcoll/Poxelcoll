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

import poxelcoll.CollisionInfo
import poxelcoll.BoundingBox
import poxelcoll.P

/** Supports operations for the default matrix implementation. */
object Transformation {

  /** The type of the default matrix implementation. */
  type MT = SimpleMatrix
  /** The matrix factory for the default matrix implementation. */
  val factory:MatrixFactory[MT] = SimpleMatrix

  /** Given the info of a collision object, derive a transformation matrix
    * from it.
    *
    * The transformation is applied in the following order:
    * First the object is translated according to where its origin point is,
    * then it is scaled along the axis,
    * then it is rotated,
    * and finally it is translated according to its general position.
    *
    * @param collInfo the collision info of a collision object
    * @return a transformation matrix that handles origin, translation, scaling and rotation
    */
  def getTransformationMatrix(collInfo:CollisionInfo) = {

    //TODO: Optimization: Consider handling the special case where scaling = 1 and rotation = 0.0 more efficiently.

    //NOTE: Please keep the below out-commented code. It helps act as documentation.

//    val posTransl = {
//      val pos = collInfo.position
//      factory.createMatrixArray(Array(1.0, 0.0, pos.x.toDouble, 0.0, 1.0, pos.y.toDouble, 0.0, 0.0, 1.0))
//    }
//
//    val rot = {
//      val ang = collInfo.angle
//      val cosA = math.cos(ang)
//      val sinA = math.sin(ang)
//      val ang90 = ang + math.Pi/2
//      val cosA90 = math.cos(ang90)
//      val sinA90 = math.sin(ang90)
//      factory.createMatrixArray(Array(cosA, sinA, 0.0, cosA90, sinA90, 0.0, 0.0, 0.0, 1.0))
//    }
//
//    val scaling = {
//      factory.createMatrixArray(Array(collInfo.scaleX, 0.0, 0.0, 0.0, collInfo.scaleY, 0.0, 0.0, 0.0, 1.0))
//    }
//
//    val originTransl = {
//      val origin = collInfo.mask.origin
//      factory.createMatrixArray(Array(1.0, 0.0, -origin.x.toDouble, 0.0, 1.0, -origin.y.toDouble, 0.0, 0.0, 1.0.toDouble))
//    }
//
//    posTransl *** rot *** scaling *** originTransl

    //NOTE: If any bugs are found in the below code, please fix them in the above out-commented code too.


    if (collInfo.angle != 0.0 || collInfo.scaleX != 1.0 || collInfo.scaleY != 1.0) {

      val (posX, posY) = {
        val pos = collInfo.position
        (pos.x.toDouble, pos.y.toDouble)
      }
      val (cosA, sinA, cosA90, sinA90) = {
        val ang = collInfo.angle
        val ang90 = ang + math.Pi/2
        (math.cos(ang), math.sin(ang), math.cos(ang90), math.sin(ang90))
      }
      val (scaleX, scaleY) = (collInfo.scaleX, collInfo.scaleY)
      val (originX, originY) = {
        val origin = collInfo.mask.origin
        (origin.x.toDouble, origin.y.toDouble)
      }

      factory.createMatrixArray(Array(
        cosA*scaleX,   scaleY*sinA,   -cosA*originX*scaleX - originY*scaleY*sinA + posX,
        cosA90*scaleX, scaleY*sinA90, -cosA90*originX*scaleX - originY*scaleY*sinA90 + posY,
        0,             0,             1
      ))
    }
    else {
      val originX = collInfo.mask.origin.x.toDouble
      val originY = collInfo.mask.origin.y.toDouble
      val posX = collInfo.position.x.toDouble
      val posY = collInfo.position.y.toDouble
      factory.createMatrixArray(Array(
        1.0,   0.0,   -originX + posX,
        0.0,   1.0,   -originY + posY,
        0,             0,             1
      ))
    }

    /*
     * [ cosA*scaleX          scaleY*sinA       -cosA*originX*scaleX - originY*scaleY*sinA + posX     ]
     * [ cosA90*scaleX        scaleY*sinA90     -cosA90*originX*scaleX - originY*scaleY*sinA90 + posY ]
     * [ 0                    0                 1                                                     ]
     */
  }

  /** Given a transformation matrix and an axis-aligned bounding box,
    * find the axis-aligned bounding box of the transformed axis-aligned bounding box.
    *
    * @param transformationMatrix the transformation matrix
    * @param boundingBox the axis-aligned bounding box
    * @return the axis-aligned bounding box of the transformed given axis-aligned bounding box.
    */
  def approximateBoundingBox(transformationMatrix:MT, boundingBox:BoundingBox) = {
      val pMin = boundingBox.pMin
      val pMax = boundingBox.pMax
      val p1 = pMin
      val p2 = pMax
      val p3 = P(pMin.x, pMax.y)
      val p4 = P(pMax.x, pMin.y)


      val approximateBoundingBoxPoints = transformationMatrix.transformPoints(IndexedSeq(p1, p2, p3, p4))
      val xs = approximateBoundingBoxPoints.map(_.x)
      val ys = approximateBoundingBoxPoints.map(_.y)

      BoundingBox(
        P(xs.min, ys.min),
        P(xs.max, ys.max)
      )
  }
}
