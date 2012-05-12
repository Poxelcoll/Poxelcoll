/* ConvexHull.hpp */

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

#ifndef POXELCOLL_GEOMETRY_CONVEXCCWPOLYGON_CONVEXHULL_HPP_
#define POXELCOLL_GEOMETRY_CONVEXCCWPOLYGON_CONVEXHULL_HPP_

using namespace poxelcoll::functional;

namespace poxelcoll {


/** \ingroup poxelcollgeometryconvexccwpolygon
 *
 * General functions for convex hulls. */
class ConvexHull {

public:

	  //This calculates the convex hull.
	  //It is meant to be robust,
	  //meaning that duplicate points, empty argument input,
	  //colinearity, etc.,
	  //does not break the calculation of the convex hull.
	  //Any input is accepted.
	  /** Given a set of points, find the convex hull of those points, and return a simple, convex, CCW polygon representing the hull.
	    *
	    * The implementation is meant to be geometrically robust, meaning it accepts any input and gives a valid polygon.
	    *
	    * @param points any kind of set of points, whether it be empty, has internal duplicates, colinearity, etc.
	    * @return a valid simple, convex CCW polygon representing the convex hull
	    */
	const static std::shared_ptr<const poxelcoll::ConvexCCWPolygon> calculateConvexHull(
			std::vector<P> & points) {

		//    //TODO: Extension: Consider supporting a mask or similar.
		//    //TODO: Optimization: Simple optimization for mask: extract the upper and lower point for each column, if any.
		//    //Then sort that.
		//    //Implement simple monotone chain. Fair time complexity (O(n log n) worst case),
		//    //but is not optimal, especially considering the domain (binary images),
		//    //which tend to be dense.
		//    //Possible optimizations that could be considered for implementation in the future
		//    //include using heuristics to remove the bulk of inner points,
		//    //as well as other algorithms such as Chan's algorithm and others.
		//    //See wikipedia on convex hull algorithms.

		const auto pointsLength = points.size();

		if (pointsLength == 0) {
			return poxelcoll::Empty::getEmpty();
		} else if (pointsLength == 1) {
			return std::shared_ptr<const poxelcoll::ConvexCCWPolygon>(
					new poxelcoll::Point(points[0]));
		} else if (pointsLength == 2) {
			return poxelcoll::Line::create(points[0], points[1]);
		} else {

			auto byX = [](const P p) {return p.gX();};
			auto groupedPoints1 =
					groupBy<std::vector<P>, double, decltype(byX)>(points, byX);

			auto mapToVector =
					[](std::pair<double, std::shared_ptr<std::vector<P>>> id) {return id;};
			typedef typename std::pair<double, std::shared_ptr<std::vector<P>>>pairDV;
			typedef typename std::map<double, std::shared_ptr<std::vector<P>>>typeM;
			typedef typename std::vector<pairDV> typeV;
			auto groupedPoints2 = map<typeM, typeV, decltype(mapToVector)>(
					*groupedPoints1, mapToVector);

			auto sortByX = [](
					const pairDV & pair1,
					const pairDV & pair2) {
				return pair1.first < pair1.first;
			};
			std::unique_ptr<std::vector<pairDV>> groupedPoints3 =
					sortByToVector(*groupedPoints2, sortByX);

			auto sortPointsByY = [](
					const P & p1,
					const P & p2
			) {
				return p1.gY() < p2.gY();
			};
			auto mapPointsAndSort =
					[&sortPointsByY](
							const pairDV & xYPoints
					) {
						return std::shared_ptr<std::vector<P>>(sortByToVector(*xYPoints.second, sortPointsByY));
					};
			auto groupedPoints4 = map<std::vector<pairDV>,
					std::vector<std::shared_ptr<std::vector<P>>>, decltype(mapPointsAndSort)>(*groupedPoints3, mapPointsAndSort);

			auto headEndMapping =
					[](const std::shared_ptr<std::vector<P>> & ele) {
						return std::pair<const P, const P>(*(*ele).begin(), *(--(*ele).end()));
					};
			auto trimmedPoints = map<
					std::vector<std::shared_ptr<std::vector<P>>>, std::vector<std::pair<P, P>>>(*groupedPoints4, headEndMapping);

			auto getLower = [](const std::pair<P, P> & lowUp) {
				return lowUp.first;
			};
			auto getUpper = [](const std::pair<P, P> & lowUp) {
				return lowUp.second;
			};
			auto lower = map<std::vector<std::pair<P, P>>, std::list<P>>(
					*trimmedPoints, getLower);
			auto upper = map<std::vector<std::pair<P, P>>, std::list<P>>(
					*trimmedPoints, getUpper);

			auto lowerOutline = std::list<P>();
			{
				auto lowerCopyReverse = *lower;
				lowerCopyReverse.reverse();

				while (!lowerCopyReverse.empty()) {

					//Add a point to the outline and remove that point from the reverse copy.
					lowerOutline.push_back(*(--lowerCopyReverse.end()));
					lowerCopyReverse.pop_back();

					//Keep removing until the outline is valid again.
					while (lowerOutline.size() >= 3 && ([lowerOutline]() {
						const auto p1 = *(------lowerOutline.end());
						const auto p2 = *(----lowerOutline.end());
						const auto p3 = *(--lowerOutline.end());
						const auto v12 = p2.minus(p1);
						const auto v13 = p3.minus(p1);
						return v12.cross(v13) <= 0.0;

					})()) {
						const auto p3 = *(--lowerOutline.end());
						lowerOutline.pop_back();
						lowerOutline.pop_back();
						lowerOutline.push_back(p3);

					}
				}
			}

			auto upperOutline = std::list<P>();
			{
				auto upperCopyReverse = *upper;
				upperCopyReverse.reverse();

				while (!upperCopyReverse.empty()) {

					//Add a point to the outline and remove that point from the reverse copy.
					upperOutline.push_back(*(--upperCopyReverse.end()));
					upperCopyReverse.pop_back();

					//Keep removing until the outline is valid again.
					while (upperOutline.size() >= 3 && ([upperOutline]() {
						const auto p1 = *(------upperOutline.end());
						const auto p2 = *(----upperOutline.end());
						const auto p3 = *(--upperOutline.end());
						const auto v12 = p2.minus(p1);
						const auto v13 = p3.minus(p1);
						return v12.cross(v13) >= 0.0; //NOTE: The big difference.

						})()) {
						const auto p3 = *(--upperOutline.end());
						upperOutline.pop_back();
						upperOutline.pop_back();
						upperOutline.push_back(p3);

					}
				}
			}
			std::list<P> upperOutlineReverse = upperOutline;
			upperOutlineReverse.reverse();

			if ((*lowerOutline.begin()).equal(*(--upperOutlineReverse.end()))) {
				upperOutlineReverse.pop_back();
			}
			if (lowerOutline.size() > 0 && upperOutlineReverse.size() > 0 && (*(--lowerOutline.end())).equal(*upperOutlineReverse.begin())) {
				lowerOutline.pop_back();
			}

			auto finalRes = lowerOutline;
			auto outputIterator = back_inserter(finalRes);
			auto i = upperOutlineReverse.begin();
			while (i != upperOutlineReverse.end()) {
				*outputIterator++ = *i++;
			}

			const auto finalResLength = finalRes.size();

			if (finalResLength == 0) {
				std::cerr << "The output was of zero length, which is an error."
						<< std::endl;
				throw "Illegal state";
			} else if (finalResLength == 1) {
				return std::shared_ptr<ConvexCCWPolygon>(
						new Point(*finalRes.begin()));
			} else if (finalResLength == 2) {
				return Line::create(*finalRes.begin(), *(++finalRes.begin()));
			} else { //length >= 3.

				const auto p1 = *(finalRes.begin());
				const auto p2 = *(++finalRes.begin());
				const auto p3 = *(++++finalRes.begin());
				finalRes.pop_front();
				finalRes.pop_front();
				finalRes.pop_front();

				const auto restVector = map<std::list<P>, std::vector<P>>(
						finalRes, [](const P ide) {return ide;});

				return Polygon::createUtterlyUnsafelyNotChecked(p1, p2, p3,
						std::shared_ptr<std::vector<P>>(
								new std::vector<P>(*restVector)));
			}
		}
	}
	;

}
;
}

#endif /* POXELCOLL_GEOMETRY_CONVEXCCWPOLYGON_CONVEXHULL_HPP_ */
