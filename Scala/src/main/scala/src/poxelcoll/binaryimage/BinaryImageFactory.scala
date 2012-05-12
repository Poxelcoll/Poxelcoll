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

/** A binary image factory supports the creation of binary images.
  *
  * Different binary image factories produces different binary images,
  * which may have different efficiency properties.
  */
trait BinaryImageFactory {

  /** @param imageSourceRows a non-empty sequence of rows, each of the same non-zero length
    * @return a binary image, or failure if the arguments were not valid
    */
  def createO(imageSourceRows:IndexedSeq[IndexedSeq[Boolean] with Immutable] with Immutable):Option[BinaryImage]
}
