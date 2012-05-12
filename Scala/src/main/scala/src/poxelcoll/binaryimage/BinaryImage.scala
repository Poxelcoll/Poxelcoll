package poxelcoll.binaryimage

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

/** A binary image is a 2D image with non-zero area which only
  * has 2 different values: off and on.
  *
  * It is useful for helping represent collision masks that are not
  * easily or efficiently expressed purely through the use of
  * a convex hull, as well as generating masks directly from existing
  * sprites and images.
  *
  * A binary image is not required to have any on-values,
  * but such an image can not cause any collisions.
  *
  * In regards to the pixels of the binary image, a pixel
  * is understood as a square taking up area exactly one square pixel-unit,
  * with side-length one pixel-unit.
  * The index (x, y) refers to the pixel that takes up space
  * defined by the intervals [x, x+1], [y, y+1].
  * Thus, the pixel at index (0, 0)
  * takes up the area located at [0, 1], [0, 1].
  * This definition is consistent with how pixels in general
  * are drawn and indexed.
  */
trait BinaryImage {

  /** @return strictly positive width of the image
    */
  def width:Int

  /** @return strictly positive height of the image
    */
  def height:Int

  /** @param x value in the range [0; width[
    * @param y value in the range [0; height[
    * @return the value of the position in the image. Behaviour is undefined if outside range
    */
  def hasPoint(x:Int, y:Int):Boolean

}
