/* PixelPerfect.hp */

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

#ifndef POXELCOLL_COLLISION_PIXELPERFECT_PIXELPERFECT_HPP_
#define POXELCOLL_COLLISION_PIXELPERFECT_PIXELPERFECT_HPP_

#include <memory>
#include <set>
#include <vector>
#include <map>

#include "../../DataTypes.hpp"
#include "../../functional/Either.hpp"
#include "../../geometry/convexccwpolygon/GeneralFunctions.hpp"

namespace poxelcoll {

/** \ingroup poxelcollcollision
  *
  * The pixel-perfect collision detection supports collision through several functions.
  */
class PixelPerfect {

private:

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

	static const std::shared_ptr<const std::vector<IP>> bresenhamsLine(const IP start, const IP end) {

		const auto steep = abs(end.gY() - start.gY()) > abs(end.gX() - start.gX());

		int x0temp, y0temp, x1temp, y1temp;
		{
			const auto x_0 = steep ? start.gY() : start.gX();
			const auto y_0 = steep ? start.gX() : start.gY();
			const auto x_1 = steep ? end.gY() : end.gX();
			const auto y_1 = steep ? end.gX() : end.gY();

			const auto x0x1 = x_0 > x_1;

			const auto x0 = x0x1 ? x_1 : x_0;
			const auto x1 = x0x1 ? x_0 : x_1;
			const auto y0 = x0x1 ? y_1 : y_0;
			const auto y1 = x0x1 ? y_0 : y_1;

			x0temp = x0;
			y0temp = y0;
			x1temp = x1;
			y1temp = y1;
		}
		const auto x0 = x0temp;
		const auto y0 = y0temp;
		const auto x1 = x1temp;
		const auto y1 = y1temp;

		const auto deltax = x1 - x0;
		const auto deltay = abs(y1-y0);
		const auto ystep = y0 < y1 ? 1 : -1;

		auto error = deltax / 2;
		auto y = y0;

		if (steep) {

			auto finalVector = new std::vector<IP>();

			for (int x = x0; x <= x1; x++) {
				const auto returnValue = IP(y, x);
				error -= deltay;
				if (error < 0) {
					y += ystep;
					error += deltax;
				}
				finalVector->push_back(returnValue);
			}

			return std::shared_ptr<const std::vector<IP>>(finalVector);
		}
		else {

			auto finalVector = new std::vector<IP>();

			for (int x = x0; x <= x1; x++) {
				const auto returnValue = IP(x, y);
				error -= deltay;
				if (error < 0) {
					y += ystep;
					error -= deltax;
				}
				finalVector->push_back(returnValue);
			}

			return std::shared_ptr<const std::vector<IP>>(finalVector);
		}
	}

	/** Round and convert to integer. */
	static const int r(const double a) {
		return (int) round(a);
	}

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
	static const std::shared_ptr<const std::set<IP, IP::Comparer>> lineToPoints(const P c1, const P c2, const P middle) {


		//Overview: Over-approximate the convex hull by placing the line 1 or sqrt(2) pixel moved,
		//in the direction away from the middle.

		const auto lineAboveMiddleCalc = [c1, c2, middle](){

		  //Double-precision-version, such that the middle isn't rounded off to lie on the line segment.
		  //or be on the wrong side of the line segment.
		  //NOTE: This part is highly vulnerable to numerical stability issues.

			const auto x1 = c1.gX();
			const auto y1 = c1.gY();
			const auto x2 = c2.gX();
			const auto y2 = c2.gY();
			const auto xm = middle.gX();
			const auto ym = middle.gY();

			const auto xD = x2 - x1;
			const auto yD = y2 - y1;

			if (xD != 0) {

				const auto ym2 = (xm - x1) * yD / xD + y1;

				if (ym2 > ym) {
					return 1;
				}
				else if (ym2 < ym) {
					return -1;
				}
				else {
					return 0;
				}
			}
			else { //A vertical line is above the middle iff it is to the right of the middle.

				if (x1 > xm) {
					return 1;
				}
				else if (x1 < xm) {
					return -1;
				}
				else {
					return 0;
				}
			}
		};
		const auto lineAboveMiddle = lineAboveMiddleCalc();

		const auto x1 = r(c1.gX());
		const auto y1 = r(c1.gY());
		const auto x2 = r(c2.gX());
		const auto y2 = r(c2.gY());

		const auto points = std::set<IP, IP::Comparer>({{IP(x1, y1), IP(x2, y2)}});

		const auto xD = x2 - x1;
		const auto yD = y2 - y1;

		const auto lineCalc = [xD, yD, x1, y1, x2, y2, lineAboveMiddle](){

			if (xD == 0 && yD == 0) {
				return std::shared_ptr<const std::vector<IP>>(new std::vector<IP>({{IP(x1, y1)}}));
			}
			else if (xD == 0) {
				const auto n1 = IP(x1 + lineAboveMiddle, y1);
				const auto n2 = IP(x2 + lineAboveMiddle, y2);
				return bresenhamsLine(n1, n2); //NOTE: Call only with different coordinates.
			}
			else if (yD == 0) {
				const auto n1 = IP(x1, y1 + lineAboveMiddle);
				const auto n2 = IP(x2, y2 + lineAboveMiddle);
				return bresenhamsLine(n1, n2); //NOTE: Call only with different coordinates.
			}
			else if ( (xD > 0 && yD > 0) || (xD < 0 && yD < 0)) {
				const auto n1 = IP(x1 - lineAboveMiddle, y1 + lineAboveMiddle);
				const auto n2 = IP(x2 - lineAboveMiddle, y2 + lineAboveMiddle);
				return bresenhamsLine(n1, n2); //NOTE: Call only with different coordinates.
			}
			else if ( (xD > 0 && yD < 0) || (xD < 0 && yD > 0)) {
				const auto n1 = IP(x1 + lineAboveMiddle, y1 + lineAboveMiddle);
				const auto n2 = IP(x2 + lineAboveMiddle, y2 + lineAboveMiddle);
				return bresenhamsLine(n1, n2); //NOTE: Call only with different coordinates.
			}
			else {
				std::cerr << "This part should not be reachable." << std::endl;
				throw 1;
			}
		};
		const auto line = lineCalc();

		const auto iden = [](const IP ip){
			return ip;
		};
		const auto lineSet = map<std::vector<IP>, std::set<IP, IP::Comparer>, decltype(iden)>(*line, iden);

		auto result = addAll(points, *lineSet);

		return std::move(result);
	}

	  /** Find the point outline of a convex polygon.
	    *
	    * @param convexHullPolygon the convex polygon
	    * @return the outline
	    */
	static std::shared_ptr<const std::set<IP, IP::Comparer>> findOutline(const std::shared_ptr<const NonemptyConvexCCWPolygon> convexHullPolygon) {


		const auto middlePoint = (*convexHullPolygon).middlePoint();

		//    def findOutline2(coordinates:List[P]):Set[IP] = coordinates.toList match {
		//    }

		typedef const std::shared_ptr<const IMList<const P>> arg1;
		typedef const std::shared_ptr<const std::set<IP, IP::Comparer>> return1;

		std::function<return1(arg1)> findOutline2;
		findOutline2 = [middlePoint, findOutline2](arg1 coordinates) -> return1 {

			const auto size = (*coordinates).size();

			if (size >= 2) {

				const auto a = *(*coordinates).headNull();
				const auto b = *(*(*coordinates).tailNull()).headNull();

				return std::move(addAll(*lineToPoints(a,b,middlePoint), *findOutline2((*coordinates).tailNull())));
			}
			else if (size == 1) {
				const auto a = *(*coordinates).headNull();
				return lineToPoints(a, a, middlePoint);
			}
			else {
				return std::shared_ptr<const std::set<IP, IP::Comparer>>(new std::set<IP, IP::Comparer>());
			}
		};

		//A point is added if the convex polygon is an actual 3-points-or-more polygon,
		//such that the line from the end to the start is also included.
		const auto polygonWithExtraEndCalc = [convexHullPolygon](){

			const auto convexHullPolygonType = (*convexHullPolygon).getType();

			if (convexHullPolygonType == ConvexCCWType::PolygonT) {

				const auto polygon = (*convexHullPolygon).getAPolygon();

				auto newPolygon = new std::vector<P>(*(*polygon).points());
				newPolygon->push_back(newPolygon->front());

				return std::shared_ptr<const NonemptyConvexCCWPolygon>(
						Polygon::createUtterlyUnsafelyNotChecked(
								std::shared_ptr<const std::vector<P>>(newPolygon)
						)
				);
			}
			else {
				return convexHullPolygon;
			}
		};
		const auto polygonWithExtraEnd = polygonWithExtraEndCalc();

		return findOutline2(IMList<const P>::constructFrom(
				*(*polygonWithExtraEnd).points()
		));
	}

	  /** Find the point outline of a convex polygon, or stop if the test function yields true for a point on the outline.
	    *
	    * @param convexHullPolygon the convex polygon
	    * @param testFunction a test function to test a point, for instance to test if a binary image is on or off at the given point
	    * @return the outline
	    */
	static const Either<const std::set<IP, IP::Comparer>, const bool> findOutlineStoppage(
				const std::shared_ptr<const NonemptyConvexCCWPolygon> convexHullPolygon,
				const std::function<const bool(const IP)> testFunction
			) {

		const auto middlePoint = (*convexHullPolygon).middlePoint();

		typedef const std::shared_ptr<const IMList<const P>> arg1;
		typedef const IMList<const std::set<IP, IP::Comparer>> part1;
		typedef const Either<part1, const bool> return1;

		std::function<return1(arg1)> findOutlineStoppage2;
		findOutlineStoppage2 = [middlePoint, testFunction, findOutlineStoppage2](arg1 coordinates) {

			const auto size = (*coordinates).size();

			if (size >= 2) {

				const auto a = *(*coordinates).headNull();
				const auto b = *(*(*coordinates).tailNull()).headNull();

				const auto linePoints = lineToPoints(a, b, middlePoint);

				if (exists(*linePoints, testFunction)) {
					return return1::createRight(std::shared_ptr<const bool>(new bool(true)));
				}
				else {
					const auto result = findOutlineStoppage2((*coordinates).tailNull());

					if (result.getIsLeft()) {
						const auto e = result.getLeft();
						return return1::createLeft(part1::prepend2(linePoints, e));
					}
					else { //NOTE: Can only be "Right" here.
						return result;
					}
				}

			}
			else if (size == 1) {

				const auto a = *(*coordinates).headNull();

				const auto linePoints = lineToPoints(a, a, middlePoint);

				if (exists(*linePoints, testFunction)) {
					return return1::createRight(std::shared_ptr<const bool>(new bool(true)));
				}
				else {
					return return1::createLeft(part1::create2(linePoints));
				}
			}
			else {
				return return1::createLeft(part1::nil());
			}
		};

		//A point is added if the convex polygon is an actual 3-points-or-more polygon,
		//such that the line from the end to the start is also included.

		const auto polygonWithExtraEndCalc = [convexHullPolygon](){

			const auto convexHullPolygonType = (*convexHullPolygon).getType();

			if (convexHullPolygonType == ConvexCCWType::PolygonT) {

				const auto polygon = (*convexHullPolygon).getAPolygon();

				auto newPolygon = new std::vector<P>(*(*polygon).points());
				newPolygon->push_back(newPolygon->front());

				return std::shared_ptr<const NonemptyConvexCCWPolygon>(
						Polygon::createUtterlyUnsafelyNotChecked(
								std::shared_ptr<const std::vector<P>>(newPolygon)
						)
				);
			}
			else {
				return convexHullPolygon;
			}
		};
		const auto polygonWithExtraEnd = polygonWithExtraEndCalc();

		const auto result = findOutlineStoppage2(IMList<const P>::constructFrom(*(*polygonWithExtraEnd).points()));

		if (result.getIsLeft()) {

			const std::shared_ptr<part1> left = result.getLeft();
			const auto result2 = part1::constructTo<std::list<std::set<IP, IP::Comparer>>>(left);

			auto result3 = flatten<std::list<std::set<IP, IP::Comparer>>, std::set<IP, IP::Comparer>>(*result2);

			const auto result4 = std::shared_ptr<const std::set<IP, IP::Comparer>>(std::move(result3));

			return Either<const std::set<IP, IP::Comparer>, const bool>::createLeft(result4);
		}
		else { //NOTE: Is right.
			const auto bo = result.getRight();

			return Either<const std::set<IP, IP::Comparer>, const bool>::createRight(bo);
		}
	}

	  /** Given some set of points forming an outline, fill that outline.
	    *
	    * If the outline is not connected, the filling is not well-defined.
	    *
	    * @param outline set of points forming an outline.
	    * @return if outline is connected, a filled outline, else not well-defined
	    */
	static const std::shared_ptr<const std::map<const int, const std::shared_ptr<const std::vector<int>>>> fillOutline(
			const std::shared_ptr<const std::set<IP, IP::Comparer>> outline) {

		typedef std::set<IP, IP::Comparer> setIP;

		const auto groupByFunction = [](const IP & ip){
			return ip.gY();
		};
		const auto groupedOutline = groupBy<setIP, int, decltype(groupByFunction)>(*outline, groupByFunction);

		const auto transformSortFunction = [](const std::pair<int, std::shared_ptr<setIP> > & ab){

			const auto a = ab.first;

			const auto getX = [](const IP ip) {return ip.gX();};

			const auto xs = map<setIP, std::vector<int>, decltype(getX) >(*ab.second, getX);

			const auto iden = [](const int a, const int b) {return a < b;};

			auto sortedXS = sortByToVector(*xs, iden);
			const auto sortedXS2 = std::shared_ptr<const std::vector<int>>(std::move(sortedXS));

			return std::pair<const int, const std::shared_ptr<const std::vector<int>>>(a, sortedXS2);

			throw 1;
		};
		const auto horizontalLines = map<
				std::map<int, std::shared_ptr<setIP>>,
				std::map<const int, const std::shared_ptr<const std::vector<int>>>,
				decltype(transformSortFunction)
					>(
				*groupedOutline, transformSortFunction
		);

		const auto finalTransform = [](const std::pair<const int, const std::shared_ptr<const std::vector<int>>> & ab){
			const auto size = (*ab.second).size();

			if (size < 1) {
				std::cerr << "The array may not be empty at this point." << std::endl;
				throw 1;
			}
			else if (size == 1) {
				return std::pair<const int, const std::shared_ptr<const std::vector<int>>>(ab);
			}
			else { //size >= 2.

				const auto head = (*ab.second).front();
				const auto last = (*ab.second).back();

				auto range = to(head, last, 1);

				return std::pair<const int, const std::shared_ptr<const std::vector<int>>>(ab.first, std::move(range));
			}
		};

		auto finalResult = map(*horizontalLines, finalTransform);
		const std::shared_ptr<const std::map<const int, const std::shared_ptr<const std::vector<int>>>> finalResult2 = std::move(finalResult);

		return finalResult2;
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

	static const bool fillOutlineStoppage(const std::shared_ptr<const std::set<IP, IP::Comparer>> outline, std::function<bool(IP)> testFunction) {


		typedef std::set<IP, IP::Comparer> setIP;

		const auto groupByFunction = [](const IP & ip){
			return ip.gY();
		};
		const auto groupedOutline = groupBy<setIP, int, decltype(groupByFunction)>(*outline, groupByFunction);

		const auto transformSortFunction = [](const std::pair<int, std::shared_ptr<setIP> > & ab){

			const auto a = ab.first;

			const auto getX = [](const IP ip) {return ip.gX();};

			const auto xs = map<setIP, std::vector<int>, decltype(getX) >(*ab.second, getX);

			const auto iden = [](const int a, const int b) {return a < b;};

			auto sortedXS = sortByToVector(*xs, iden);
			const auto sortedXS2 = std::shared_ptr<const std::vector<int>>(std::move(sortedXS));

			return std::pair<const int, const std::shared_ptr<const std::vector<int>>>(a, sortedXS2);

			throw 1;
		};
		const auto horizontalLines = map<
				std::map<int, std::shared_ptr<setIP>>,
				std::map<const int, const std::shared_ptr<const std::vector<int>>>,
				decltype(transformSortFunction)
					>(
				*groupedOutline, transformSortFunction
		);

		const auto testLine = [testFunction](const std::pair<const int, const std::shared_ptr<const std::vector<int>>> yXs) {

			const auto y = yXs.first;

			const auto head = (*yXs.second).front();
			const auto last = (*yXs.second).back();

			const auto range = to(head, last, 1);

			const auto innerTestFunction = [testFunction, y](const int x) {
				return testFunction(IP(x, y));
			};

			return exists(*range, innerTestFunction);
		};

		return exists(*horizontalLines, testLine);
	}

public:

	  /** Given an area defined by a non-empty convex polygon, test if any of the points in it yields true.
	    *
	    * The method guarantees correct handling of pixels in regards to that pixels are defined
	    * as areas, and that the index (x, y) in a binary image refers to the area [x, x+1], [y, y+1].
	    *
	    * @param nonemptyConvexPolygon the area to test for
	    * @param testFunction the test function
	    * @return whether any point in the area yields true for the test function
	    */
	static const bool collisionTest(const std::shared_ptr<const NonemptyConvexCCWPolygon> nonemptyConvexPolygon, std::function<bool(IP)> testFunction) {

		//NOTE: Translate the polygon a little backwards, such that the pixel test will be correct.
		const auto correctedPolygon = (*nonemptyConvexPolygon).translate(P(-0.5, -0.5));

		const auto size = (*(*correctedPolygon).points()).size();

		if (size < 1) {
			return false;
		}
		else {
			const auto outlineResult = findOutlineStoppage(nonemptyConvexPolygon, testFunction);

			if (outlineResult.getIsRight()) {
				return *outlineResult.getRight();
			}
			else { //NOTE: Is left here.
				const auto outline = outlineResult.getLeft();
				return fillOutlineStoppage(outline, testFunction);
			}
		}
	}
};

}

#endif /* POXELCOLL_COLLISION_PIXELPERFECT_PIXELPERFECT_HPP_ */
