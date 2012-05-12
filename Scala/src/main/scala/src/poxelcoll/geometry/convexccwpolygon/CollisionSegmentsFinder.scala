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

/** The dir trait stands for direction, and has three values.
  *
  * The direction is used in the rotating callipers method in CollisionSegmentsFinder.
  * For a given pair of callipers, the direction is the relative position of the
  * second calliper to the first.
  * For instance, if the second calliper is to the right (as seen from the counter-clockwise (CCW) view),
  * then the direction is RightDir.
  */
sealed abstract trait Dir
/** The left direction. */
final case object LeftDir extends Dir
/** The right direction. */
final case object RightDir extends Dir
/** The same direction. */
final case object SameDir extends Dir

/** The collision segments finder finds all the collision segments between two convex polygons in CCW-order.
  *
  * This is done in linear time in the number of points of the polygons.
  * The general method used is rotating callipers.
  *
  * This collision segments finder is part of a robust variation of the algorithm found here:
  * http://www-cgrl.cs.mcgill.ca/~godfried/teaching/cg-projects/97/Plante/CompGeomProject-EPlante/algorithm.html
  * This handles finding all the intersections of the first and second polygon, including but not limited to
  * those intersections that lies in pockets.
  *
  * The collision segments are found in CCW-order, meaning that the segments collisions follow the intersection.
  *
  * Note the definition of a collision segment: for a given index in one of the counter-clockwise (CCW) convex polygons,
  * form a line segment from the current point to the next point, and include all points on the line segment except
  * the current point. Call this the directed line segment. For any given two indices in two polygons,
  * these two indices collide if and only if their directed line segments overlap.
  *
  * ==Status==
  *
  * The current implementation as of 0.1 is meant to be geometrically robust,
  * but gives no guarantees in regards to being numerically robust.
  * The consequences of the lack of numerical robustness is unknown,
  * but may range from imprecision to undefined behaviour.
  * The numerical robustness may be improved in the future.
  *
  * @param poly1Points points of the first convex CCW polygon
  * @param poly2Points points of the second convex CCW polygon
  * @param originIndex1 origin inex of the first polygon
  * @param originIndex2 origin inex of the second polygon
  */
final class CollisionSegmentsFinder(poly1Points:IndexedSeq[P], poly2Points:IndexedSeq[P], originIndex1:Int, originIndex2:Int) {

  import GeneralFunctions._

  /** Number of points in polygon 1. */
  private[this] val size1 = poly1Points.size
  /** Number of points in polygon 2. */
  private[this] val size2 = poly2Points.size

  /** Given an index in a polygon of a given size, find the circular next index.
    *
    * @param a valid index of a polygon
    * @param size the number of points in the convex polygon
    * @return the next circular index
    */
  private[this] def next(a:Int, size:Int) = (a + 1) % size

  /** Given an index in a polygon of a given size, find the circular previous index.
    *
    * @param a valid index of a polygon
    * @param size the number of points in the convex polygon
    * @return the previous circular index
    */
  private[this] def prev(a:Int, size:Int) =  if (a - 1 < 0) size - 1 else a - 1

  /** Find the next circular index for polygon 1.
    *
    * @param i valid index for polygon 1
    * @return the next circular index
    */
  private[this] def next1(i:Int) = next(i, size1)

  /** Find the next circular index for polygon 2.
    *
    * @param i valid index for polygon 2
    * @return the next circular index
    */
  private[this] def next2(i:Int) = next(i, size2)

  /** Assuming that the indices fits with rotating callipers, find the next rotating callipers indices.
    *
    * @param i1 index in polygon 1
    * @param i2 index in polygon 2
    * @param currentDir the current direction
    * @return the next indices when moving the callipers one step
    */
  //Find the coming index when finding the collision segments.
  private[this] def findComingIndex(i1:Int, i2:Int, currentDir:Dir):(Int, Int) = {
    val nextI1 = next1(i1)
    val nextI2 = next2(i2)

    val p11 = poly1Points(i1)
    val p12 = poly1Points(nextI1)
    val p21 = poly2Points(i2)
    val p22 = poly2Points(nextI2)

    val v1 = p12 - p11
    val v2 = p22 - p21

    v1 X v2 match {
      case 0.0 => {
        (nextI1, nextI2)
      }
      case a if a > 0.0 => {
        (nextI1, i2)
      }
      case _ => {//a < 0.0
        (i1, nextI2)
      }
    }
  }

  /** The cross left finder is used to find crosses, assuming that the second polygon was previously to the left.
    *
    * If the second polygon was not previously to the left, the behaviour is undefined.
    *
    * A cross is understood as the cross found when using rotating callipers, as described here:
    * http://www-cgrl.cs.mcgill.ca/~godfried/teaching/cg-projects/97/Plante/CompGeomProject-EPlante/algorithm.html
    *
    * @param startIndex1 the start index for polygon 1 after cross has been detected
    * @param startIndex2 the start index for polygon 2 after cross has been detected
    * @param s1 the size of polygon 1
    * @param s2 the size of polygon 2
    * @param p1Points points of polygon 1
    * @param p2Points points of polygon 2
    * @param getColli function that given 2 indices, returns the collision segments for the directed line segments at those indices
    */
  final private[this] class CrossLeftFinder(startIndex1:Int, startIndex2:Int, s1:Int, s2:Int, p1Points:IndexedSeq[P], p2Points:IndexedSeq[P],
      getColli:(Int, Int) => CollisionSegments
      ) {


    /** Find the cross, assuming that the given argument polygon 2 was previously to the left of polygon 1.
      *
      * @return the cross indicated by the start indices, or nothing if there is no cross
      */
    def getCrossLeft = {
      getCrossLeftInner(startIndex1, startIndex2)
    }

    /** Given indices, step towards the cross, until the cross is found or it is detected that there is no cross.
      *
      * @param i1 the index for the first polygon
      * @param i2 the index for the second polygon
      * @return Some collision segments if cross found, or None
      */
    private[this] def getCrossLeftInner(i1:Int, i2:Int):Option[CollisionSegments] = {
      //Go as far as possible, and then check.

      val p11 = p1Points(i1)
      val p12 = p1Points(next(i1, s1))

      val p21 = p2Points(i2)
      val p22 = p2Points(prev(i2, s2))//NOTE: Going backwards.

      val v1 = p12 - p11
      val v2 = p22 - p21

      if ((v1 X v2) >= 0.0) {

        //Try to move along polygon 1.
        val v21 = p12 - p21

        if ((v2 X v21) > 0.0) {
          getCrossLeftInner(next(i1, s1), i2)
        }
        else {

          val v12 = p22 - p11

          if ((v1 X v12) < 0.0) {
            getCrossLeftInner(i1, prev(i2, s2)) //NOTE: Going backwards.
          }
          else {

            //Test for collisions!

            //Go back once, and if something, return it.

            val i22 = prev(i2, s2) //NOTE: Going backwards.
            val collisionSegments = getColli(i1, i22)

            if (collisionSegments.isEmpty) {
              val i23 = prev(i22, s2)
              val collisionSegments2 = getColli(i1, i23)
              if (collisionSegments2.isEmpty) {
                None
              }
              else {
                Some(collisionSegments2)
              }
            }
            else {
              Some(collisionSegments)
            }
          }
        }
      }
      else {
        None //There is no cross, no overlap between the polygons at all.
      }
    }
  }

  /** Given a position with a potential cross (ie. when the direction between the polygons change), find the cross.
    *
    * An example of changing directions is when the second polygon goes from being on the left to being on the right.
    * Note that if there is no collision segments in a cross, there are no collision segments whatsoever,
    * and there is thus no intersecting polygon at all.
    *
    * @param i1 index for polygon 1
    * @param i2 index for polygon 2
    * @param prevDir the previous direction the second polygon was relative to the first one
    * @param currentDir the current direction the second polygon is relative to the first one
    * @return the collision segments of the cross, or None if no intersection at all
    */
  private[this] def getCross(i1:Int, i2:Int, prevDir:Dir, currentDir:Dir):Option[CollisionSegments] = {

    //There are 2 cases: When the shift is from left, and when the shift is from right.
    //These 2 cases are symmetric, thus that handling polygon1 and polygon2 left to right
    //is the same as handling polygon2 and polygon1 right to left.
    //In order to avoid duplication of code, these 2 cases are handled by swapping the polygons
    //according to direction.
    (prevDir, currentDir) match {
      //Case 1.
      case (LeftDir, RightDir) | (LeftDir, SameDir)=> {
        val finder = new CrossLeftFinder(i1, i2, size1, size2, poly1Points, poly2Points, getCollisionDirectedLineSegment(_, _, poly1Points, poly2Points))
        finder.getCrossLeft
      }
      //Case 2.
      case (RightDir, LeftDir) | (RightDir, SameDir)=> {
        //Reversal of LR,LS.
        val finder = new CrossLeftFinder(i2, i1, size2, size1, poly2Points, poly1Points, (a, b) => getCollisionDirectedLineSegment(b, a, poly1Points, poly2Points))
        finder.getCrossLeft
      }
      case (_, _) => {
        //All the rest of the cases are not accepted.
        //Just return None.
        None
      }
    }
  }

  /** Find the direction of the second polygon relative to the first polygon.
    *
    * "direction of the second polygon relative to the first polygon"
    * is defined from the callipers of the convex counter-clockwise (CCW) polygons.
    * It is assumed that the indices are valid in regards to the callipers.
    *
    * The two callipers are defined as one line for each polygon.
    * They go in the counter-clockwise direction, to fit with the
    * definition of the polygons.
    * See http://en.wikipedia.org/wiki/Rotating_calipers or google for
    * "rotating callipers intersection" for more information on callipers and intersection.
    *
    * @param i1 index of polygon 1
    * @param i2 index of polygon 2
    * @return the direction of the second calliper relative to the first
    */
  private[this] def findDir(i1:Int, i2:Int):Dir = {

    //First, find the competing vectors.

    val nextI1 = next1(i1)
    val nextI2 = next2(i2)

    val p11 = poly1Points(i1)
    val p12 = poly1Points(nextI1)
    val p21 = poly2Points(i2)
    val p22 = poly2Points(nextI2)

    val v1 = p12 - p11
    val v2 = p22 - p21

    v1 X v2 match {
      case 0.0 => {
        //Vectors have the same direction!
        //Choose the first, the best.
        val vChosen = v1
        val vPoint = p21 - p11
        vChosen X vPoint match {
          case 0.0 => SameDir
          case a if a > 0.0 => LeftDir
          case _ => RightDir
        }
      }
      case a if a > 0.0 => {
        val vChosen = v1
        val vPoint = p21 - p11
        vChosen X vPoint match {
          case 0.0 => SameDir
          case a if a > 0.0 => LeftDir
          case _ => RightDir
        }
      }
      case _ => {//a < 0.0
        val vChosen = v2
        val vPoint = p11 - p21
        vChosen X vPoint match {
          case 0.0 => SameDir
          case a if a > 0.0 => RightDir
          case _ => LeftDir
        }
      }
    }
  }

  /** Go through the polygons, and find all collision segments. None may be returned if there is no collision segments at all.
    *
    * @param i1 the index of the calliper of the first polygon at this point
    * @param i2 the index of the calliper of the second polygon at this point
    * @param previousDirO the previous direction of the callipers
    * @param prevRes the collision segments found so far
    * @return all the collision segments between the convex polygons, if any
    */
  private[this] def findAllCollisionSegments(i1:Int, i2:Int, previousDirO:Option[Dir], prevRes:CollisionSegments):Option[CollisionSegments] = {

    //Find the current dir.

    val currentDir = findDir(i1, i2)

    //Get overlapping for line segments, and add if one.

    previousDirO match {
      case None => {
        val (comingI1, comingI2) = findComingIndex(i1, i2, currentDir)
        findAllCollisionSegments(comingI1, comingI2, Some(currentDir), prevRes)
      }
      case Some(previousDir) => {

        currentDir match {
          case SameDir => {

            //When the current direction is the same, finding the collision segments get complicated.
            //The same direction is handled in order to achieve geometric robustness.

            //If the polygons are overlapping along the callipers at this point, it requires special handling.
            val isOverlapping = {

              val p11 = poly1Points(i1)
              val p12 = poly1Points(next1(i1))
              val p21 = poly2Points(i2)
              val p22 = poly2Points(next2(i2))

              val isOverlapping = handleLineLine(p11, p12, p21, p22) match {
                case Empty => false
                case _ => true
              }

              isOverlapping
            }

            if (!isOverlapping) { //If non-overlapping, simply find the cross.
              val crossO = getCross(i1, i2, previousDir, currentDir)
              crossO match {
                case None => None
                case Some(cross) => {
                  val res = prevRes ++ cross
                  if (i1 == originIndex1 && i2 == originIndex2) {
                    Some(res)
                  }
                  else {
                    val (comingI1, comingI2) = findComingIndex(i1, i2, currentDir)
                    findAllCollisionSegments(comingI1, comingI2, Some(currentDir), res)
                  }
                }
              }
            }
            else {

              //The polygons are overlapping along the callipers, complicating things.

              //Ensure that the correct directed overlapping head-points are included.
              val newRes = {

                val p11 = poly1Points(i1)
                val p12 = poly1Points(next1(i1))
                val p21 = poly2Points(i2)
                val p22 = poly2Points(next2(i2))

                val backs = {
                  if (p11 == p21) {
                    List.empty
                  }
                  else {
                    val back1 = {
                      val overlap = handlePointLine(Point(p11), Line.createUtterlyUnsafelyNotChecked(p21, p22)) //Safe, because p21 and p22 are always different.
                      overlap match {
                        case Empty => List.empty
                        case Point(p) => List(CollisionSegment(prev(i1, size1), i2, p))
                      }
                    }
                    val back2 = {
                      val overlap = handlePointLine(Point(p21), Line.createUtterlyUnsafelyNotChecked(p11, p12)) //Safe, because p11 and p12 are always different.
                      overlap match {
                        case Empty => List.empty
                        case Point(p) => List(CollisionSegment(i1, prev(i2, size2), p))
                      }
                    }

                    back1 ++ back2
                  }
                }

                backs ++ getCollisionDirectedLineSegment(i1, i2, poly1Points, poly2Points)
              }

              val res = prevRes ++ newRes
              if (i1 == originIndex1 && i2 == originIndex2) {
                Some(res)
              }
              else {
                val (comingI1, comingI2) = findComingIndex(i1, i2, currentDir)
                findAllCollisionSegments(comingI1, comingI2, Some(currentDir), res)
              }
            }
          }
          case LeftDir | RightDir => {
            (previousDir, currentDir) match {
              case (LeftDir, RightDir) | (RightDir, LeftDir) => {

                //When the callipers change relative direction cleanly (instead of having the "same direction"),
                //simply find the cross.

                val crossO = getCross(i1, i2, previousDir, currentDir)

                crossO match {
                  case None => None
                  case Some(cross) => {

                    val res = prevRes ++ cross

                    if (i1 == originIndex1 && i2 == originIndex2) {
                      Some(res)
                    }
                    else {
                      val (comingI1, comingI2) = findComingIndex(i1, i2, currentDir)
                      findAllCollisionSegments(comingI1, comingI2, Some(currentDir), res)
                    }
                  }
                }

              }
              case _ => {

                //If the current direction is either left or right, and the previous was the same,
                //collision segments at this point has already been handled or will be handled.

                val res = prevRes

                if (i1 == originIndex1 && i2 == originIndex2) {
                  Some(res)
                }
                else {
                  val (comingI1, comingI2) = findComingIndex(i1, i2, currentDir)
                  findAllCollisionSegments(comingI1, comingI2, Some(currentDir), res)
                }
              }
            }
          }
        }
      }
    }
  }

  /** Find all the collision segments between the two convex polygons in CCW-order.
    *
    * @return the collision segments between the two convex polygons, if any.
   *         If there are no collision segments (ie. empty collection),
   *         it means the intersection is either empty, or one is strictly inside the other.
   *         If none is returned, there are no intersection at all
    */
  def getCollisionSegments = {
    findAllCollisionSegments(originIndex1, originIndex2, None, CollisionSegments.empty)
  }
}
