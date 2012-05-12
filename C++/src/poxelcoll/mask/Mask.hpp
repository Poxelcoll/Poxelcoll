/* Mask.hpp */

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

#ifndef POXELCOLL_MASK_MASK_HPP_
#define POXELCOLL_MASK_MASK_HPP_

#include <algorithm>

#include "../DataTypes.hpp"
#include "../binaryimage/BinaryImage.hpp"
#include "../geometry/convexccwpolygon/DataTypes.hpp"
#include "../geometry/convexccwpolygon/ConvexHull.hpp"
#include "../binaryimage/SimpleBinaryImage.hpp"
#include "../binaryimage/BinaryImage.hpp"
#include "../functional/Functional.hpp"

using namespace poxelcoll::functional;

namespace poxelcoll {

/** \ingroup poxelcoll
 *
 * A mask consists of either a binary image and an approximating convex hull and axis-aligned bounding box,
 * or a full convex hull and an approximating axis-aligned bounding box.
 *
 * A mask may not be empty. An empty mask can never have collisions, and is therefore not allowed.
 */
class Mask {
private:
	const P myOrigin;
	const BoundingBox myBoundingBox;
	const std::shared_ptr<const NonemptyConvexCCWPolygon> myConvexHull;
	const std::shared_ptr<const BinaryImage> myBinaryImageNull; //NOTE: Handle potential null.

public:

	Mask(const P origin, const BoundingBox boundingBox,
			const std::shared_ptr<const NonemptyConvexCCWPolygon> convexHull,
			const std::shared_ptr<const BinaryImage> binaryImageNull);

	/** The origin point of the mask.
	 *
	 * If a point in the mask has position P(1, 2),
	 * and the origin point is P(5, 5), the effective position of the point
	 * in the mask is P(-4, -3).
	 *
	 * @return the origin point of the mask.
	 */
	const P origin() const;

	/** The axis-aligned bounding box of the mask.
	 *
	 * The bounding box may never under-approximate the binary image
	 * and the convex hull.
	 *
	 * @return the axis-aligned, over-approximating bounding box
	 */
	const BoundingBox boundingBox() const;

	/** Either a over-approximating convex hull of the binary image if the binary image is present,
	 * or a shape representing the mask accurately if the binary image is not present.
	 *
	 * @return the convex hull of the binary image or the convex hull representing the mask
	 */
	const std::shared_ptr<const NonemptyConvexCCWPolygon> convexHull() const;

	/** The binary image if present, or none if not.
	 *
	 * @return Some binary image or None
	 */
	const std::shared_ptr<const BinaryImage> binaryImageNull() const;

	/** Whether the mask is full or not. Equivalent to whether it does not have a binary image or not.
	 *
	 * @return whether the mask is full or not
	 */
	const bool isPolygonFull() const;

	/** Creates a mask from the given image source, origin and binary image factory,
	 * or none if input arguments are invalid, such as if the source is empty.
	 *
	 * @param imageSource the source of the image, a sequence of rows. The number of rows
	 *                    must be non-zero, and the row length must be consistent
	 * @param origin the origin point of the mask
	 * @param binaryImageFactory the factory for creating the binary image
	 * @return binary image if input valid, else none
	 */
	static const Mask* createMaskNullFromImageSource(
			const std::deque<std::deque<bool>>& imageSourceRows, const P origin,
			const BinaryImageFactory& binaryImageFactory =
					SimpleBinaryImageFactory()) {

			const
		auto binaryImageNull = binaryImageFactory.createNull(imageSourceRows); //NOTE: Check for null.

		if (binaryImageNull == 0) { //NOTE: Handling null.
			std::cerr << "Illegal input, given image was null." << std::endl;
			return 0;
		} else { //NOTE: Not null, put into a handled pointer.
			const auto binaryImage = std::shared_ptr<const BinaryImage>(
					binaryImageNull);

			const auto width = (*binaryImage).width();
			const auto height = (*binaryImage).height();

			std::vector<P> points;
			for (unsigned int x = 0; x < width; x++) {
				for (unsigned int y = 0; y < height; y++) {
					if ((*binaryImage).hasPoint(x, y)) {
						points.push_back(P(x, y));
						points.push_back(P(x + 1, y));
						points.push_back(P(x, y + 1));
						points.push_back(P(x + 1, y + 1));
					}
				}
			}

			if (points.empty()) {
				std::cerr << "The given image source was empty." << std::endl;
				return 0;
			} else {

				const auto getX = [](const P p) {return p.gX();};
				const auto xs = map<std::vector<P>, std::vector<double>,
						decltype(getX)>(points, getX);
				const auto xMin = minDefault(*xs, 0.0);
				const auto xMax = maxDefault(*xs, 0.0);

				const auto getY = [](const P p) {return p.gY();};
				const auto ys = map<std::vector<P>, std::vector<double>,
						decltype(getY)>(points, getY);
				const auto yMin = minDefault(*ys, 0.0);
				const auto yMax = maxDefault(*ys, 0.0);

				const BoundingBox boundingBox(P(xMin, yMin), P(xMax, yMax));

				const auto someConvexHull = ConvexHull::calculateConvexHull(
						points);

				const auto type = (*someConvexHull).getType();

				switch (type) {
				case ConvexCCWType::EmptyT: {
					std::cerr << "Illegal state, the calculated hull was empty."
							<< std::endl;
					throw 1;
				}
				case ConvexCCWType::PointT: {
					const auto point = (*someConvexHull).getAPoint();
					return new Mask(origin, boundingBox, point, binaryImage);
				}
				case ConvexCCWType::LineT: {
					const auto line = (*someConvexHull).getALine();
					return new Mask(origin, boundingBox, line, binaryImage);
				}
				case ConvexCCWType::PolygonT: {
					const auto polygon = (*someConvexHull).getAPolygon();
					return new Mask(origin, boundingBox, polygon, binaryImage);
				}
				default: {
					std::cerr << "Didn't match anything in enum." << std::endl;
					throw 1;
				}
				}
			}
		}
	}

	/** Given a non-empty convex hull, create a mask.
	 *
	 * @param polygon the non-empty convex hull
	 * @param origin the origin
	 * @return the created mask
	 */
	static const std::shared_ptr<const Mask> createMaskFromPolygon(
			std::shared_ptr<const NonemptyConvexCCWPolygon> polygon,
			const P origin) {

		const auto type = (*polygon).getType();
		switch (type) {
		case ConvexCCWType::EmptyT: {
			std::cerr << "Empty should not be possible here." << std::endl;
			throw 1;
		}
		case ConvexCCWType::PointT: {
			const auto point = (*polygon).getAPoint();
			const BoundingBox boundingBox((*point).myPoint, (*point).myPoint);
			return std::shared_ptr<const Mask>(
					new Mask(origin, boundingBox, point,
							std::shared_ptr<const BinaryImage>(0)));
		}
		case ConvexCCWType::LineT: {
			const auto line = (*polygon).getALine();
			const auto p1 = (*line).myP1;
			const auto p2 = (*line).myP2;
			const auto pMinX = std::min(p1.gX(), p2.gX());
			const auto pMinY = std::min(p1.gY(), p2.gY());
			const auto pMaxX = std::max(p1.gX(), p2.gX());
			const auto pMaxY = std::max(p1.gY(), p2.gY());
			const BoundingBox boundingBox(P(pMinX, pMinY), P(pMaxX, pMaxY));
			return std::shared_ptr<const Mask>(
					new Mask(origin, boundingBox, line,
							std::shared_ptr<const BinaryImage>(0)));
		}
		case ConvexCCWType::PolygonT: {

			const auto poly = (*polygon).getAPolygon();
			const auto points = (*poly).points();

			const auto getX = [](const P p) {return p.gX();};
			const auto getY = [](const P p) {return p.gY();};
			const auto xs = map<const std::vector<P>, std::vector<double>,
					decltype(getX)>(*points, getX);
			const auto ys = map<const std::vector<P>, std::vector<double>,
					decltype(getY)>(*points, getY);
			const auto pMinX = minDefault(*xs, 0.0);
			const auto pMinY = minDefault(*ys, 0.0);
			const auto pMaxX = maxDefault(*xs, 0.0);
			const auto pMaxY = maxDefault(*ys, 0.0);

			const BoundingBox boundingBox(P(pMinX, pMinY), P(pMaxX, pMaxY));
			return std::shared_ptr<const Mask>(
					new Mask(origin, boundingBox, poly,
							std::shared_ptr<const BinaryImage>(0)));
		}
		default: {
			std::cerr << "Didn't match anything in enum." << std::endl;
			throw 1;
		}
		}
	}

	static const std::shared_ptr<const Mask> createL(const BinaryImageFactory & binaryImageFactory = SimpleBinaryImageFactory()) {

		const auto width = 30;
		const auto height = 30;
		const auto origin = P(0, 0);

		std::deque<std::deque<bool>> rows;

		for (auto y = 0; y < height; y++) {
			std::deque<bool> row;
			for (auto x = 0; x < width; x++) {
				const auto value = (x >= 5 && x <= 10 && y >= 0 && y <= 30) || (x >= 5 && x <= 30 && y >= 0 && y <= 5);
				row.push_back(value);
			}
			rows.push_back(row);
		}

		const auto maskNull = createMaskNullFromImageSource(rows, origin); //NOTE: Handle null.

		if (maskNull == 0) { //NOTE: Null, so error.
			std::cerr << "Illegal state, mask should not be null at this point." << std::endl;
			throw 1;
		}
		else { //NOTE: Not null, so make shared_ptr.
			const auto mask = std::shared_ptr<const Mask>(maskNull);
			return mask;
		}
	}

	static const std::shared_ptr<const Mask> createPentagon() {

		const auto restVector = new std::vector<P>({{P(5, 15), P(-5, 10)}});
		const std::shared_ptr<const std::vector<P>> rest(restVector);

		const auto pentagon = createMaskFromPolygon(Polygon::createUtterlyUnsafelyNotChecked(P(0, 0), P(10, 0), P(15, 10), rest), P(0, 0));

		return pentagon;
	}
};

}

#endif /* POXELCOLL_MASK_MASK_HPP_ */
