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

import scala.collection.immutable.BitSet

/** A binary image utilising a bitset to get specific performance characteristics.
  *
  * Notably, a bitset generally requires relatively very little memory,
  * while accessing points is a bit more expensive compared to other methods.
  * See for instance http://en.wikipedia.org/wiki/Bitset.
  */
class BitsetBinaryImage private(
      private val imageSourceRows:BitSet with Immutable,
      private val myWidth:Int,
      private val myHeight:Int
    )
    extends BinaryImage {

  def width = myWidth
  def height = myHeight

  def hasPoint(x:Int, y:Int) = {
    imageSourceRows(x + y * width)
  }
}

/** The factory for the bitset binary image.
  */
object BitsetBinaryImage extends BinaryImageFactory {

  def createO(imageSourceRows:IndexedSeq[IndexedSeq[Boolean] with Immutable] with Immutable):Option[BinaryImage] = {
    val height = imageSourceRows.size
    if (height < 1) {
      None
    }
    else {
      val firstRow = imageSourceRows(0) //Size is at least 1, so this is fine.
      val width = firstRow.size
      if (imageSourceRows.exists(_.size != width)) {
        None
      }
      else {

        val flattened = imageSourceRows.flatten
        //Combine (zip) the bits with their index, filter for on-bits, and get the index values.
        //The index values indicates the positions of on-bits.
        val bitSet = BitSet(flattened.zip(0 until flattened.length).filter(_._1).map(_._2):_*)

        Some(new BitsetBinaryImage(bitSet, width, height))
      }
    }
  }
}
