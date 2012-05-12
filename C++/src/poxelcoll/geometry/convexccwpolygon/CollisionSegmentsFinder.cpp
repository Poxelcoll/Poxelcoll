/* CollisionSegmentsFinder.cpp */

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

#include "CollisionSegmentsFinder.hpp"

namespace poxelcoll {

CollisionSegmentsFinder::CollisionSegmentsFinder(
		const std::shared_ptr<const std::vector<P>> aPoly1Points,
		const std::shared_ptr<const std::vector<P>> aPoly2Points,
		const int aOriginIndex1, const int aOriginIndex2) :
		poly1Points(aPoly1Points), poly2Points(aPoly2Points), originIndex1(
				aOriginIndex1), originIndex2(aOriginIndex2), size1(
				(*poly1Points).size()), size2((*poly2Points).size()) {
}

const int CollisionSegmentsFinder::next(const int a, const int size) const {
	return (a + 1) % size;
}

const int CollisionSegmentsFinder::prev(const int a, const int size) const {
	return (a - 1 < 0) ? size - 1 : a - 1;
}

const int CollisionSegmentsFinder::next1(const int i) const {
	return next(i, size1);
}

const int CollisionSegmentsFinder::next2(const int i) const {
	return next(i, size2);
}

const std::pair<int, int> CollisionSegmentsFinder::findComingIndex(const int i1, const int i2,
		const Dir currentDir) const {

	const auto nextI1 = next1(i1);
	const auto nextI2 = next2(i2);

	const auto p11 = (*poly1Points)[i1];
	const auto p12 = (*poly1Points)[nextI1];
	const auto p21 = (*poly2Points)[i2];
	const auto p22 = (*poly2Points)[nextI2];

	const auto v1 = p12.minus(p11);
	const auto v2 = p22.minus(p21);

	const auto v1v2Cross = v1.cross(v2);
	if (v1v2Cross == 0.0) {
		return std::pair<int, int>(nextI1, nextI2);
	} else if (v1v2Cross > 0.0) {
		return std::pair<int, int>(nextI1, i2);
	} else {
		return std::pair<int, int>(i1, nextI2);
	}
}


const std::shared_ptr<const std::vector<CollisionSegment>> CollisionSegmentsFinder::getCrossNull(
		const int i1, const int i2, const Dir prevDir,
		const Dir currentDir) const {

	//There are 2 cases: When the shift is from left, and when the shift is from right.
	//These 2 cases are symmetric, thus that handling polygon1 and polygon2 left to right
	//is the same as handling polygon2 and polygon1 right to left.
	//In order to avoid duplication of code, these 2 cases are handled by swapping the polygons
	//according to direction.

	if ( //Case 1.
	prevDir == Dir::LeftDir
			&& (currentDir == Dir::RightDir || currentDir == Dir::SameDir)) {

		const auto getColliNormal =
				[poly1Points, poly2Points](const int i1, const int i2) {
					return GeneralFunctions::getCollisionDirectedLineSegment(i1, i2, poly1Points, poly2Points);
				};
		const auto finder = CrossLeftFinder<decltype(getColliNormal)>(i1,
				i2, size1, size2, poly1Points, poly2Points, getColliNormal);
		return finder.getCrossLeftNull();
	} else if ( //Case 2.
	prevDir == Dir::RightDir
			&& (currentDir == Dir::LeftDir || currentDir == Dir::SameDir)) {

		//Reversal of LR,LS.
		const auto getColliSwapped =
				[poly1Points, poly2Points](const int i1, const int i2) {

					return GeneralFunctions::getCollisionDirectedLineSegment(i2, i1, poly1Points, poly2Points);
				};

		const auto finder = CrossLeftFinder<decltype(getColliSwapped)>(i2,
				i1, size2, size1, poly2Points, poly1Points,
				getColliSwapped);

		return finder.getCrossLeftNull();
	} else {

		//All the rest of the cases are not accepted.
		//Just return None.
		return std::shared_ptr<const std::vector<CollisionSegment>>(0);
	}
}

const Dir CollisionSegmentsFinder::findDir(const int i1, const int i2) const {

	//First, find the competing vectors.

	const auto nextI1 = next1(i1);
	const auto nextI2 = next2(i2);

	const auto p11 = (*poly1Points)[i1];
	const auto p12 = (*poly1Points)[nextI1];
	const auto p21 = (*poly2Points)[i2];
	const auto p22 = (*poly2Points)[nextI2];

	const auto v1 = p12.minus(p11);
	const auto v2 = p22.minus(p21);

	const auto v1CrossV2 = v1.cross(v2);

	if (v1CrossV2 == 0.0) {

		//Vectors have the same direction!
		//Choose the first, the best.

		const auto vChosen = v1;
		const auto vPoint = p21.minus(p11);
		const auto vChosenCrossVPoint = vChosen.cross(vPoint);
		if (vChosenCrossVPoint == 0.0) {
			return Dir::SameDir;
		} else if (vChosenCrossVPoint > 0.0) {
			return Dir::LeftDir;
		} else {
			return Dir::RightDir;
		}
	} else if (v1CrossV2 > 0.0) {

		const auto vChosen = v1;
		const auto vPoint = p21.minus(p11);
		const auto vChosenCrossVPoint = vChosen.cross(vPoint);
		if (vChosenCrossVPoint == 0.0) {
			return Dir::SameDir;
		} else if (vChosenCrossVPoint > 0.0) {
			return Dir::LeftDir;
		} else {
			return Dir::RightDir;
		}
	} else { //a < 0.0

		const auto vChosen = v2;
		const auto vPoint = p11.minus(p21);
		const auto vChosenCrossVPoint = vChosen.cross(vPoint);
		if (vChosenCrossVPoint == 0.0) {
			return Dir::SameDir;
		} else if (vChosenCrossVPoint > 0.0) { //Note the change.
			return Dir::RightDir;
		} else {
			return Dir::LeftDir; //Note the change.
		}
	}
}


const std::shared_ptr<const std::vector<CollisionSegment>> CollisionSegmentsFinder::findAllCollisionSegmentsNull(
		const int i1, const int i2,
		std::shared_ptr<const Dir> previousDirNull,
		std::shared_ptr<const std::vector<CollisionSegment>> prevRes) const {

	//Find the current dir.

	const auto currentDir = findDir(i1, i2);

	//Get overlapping for line segments, and add if one.

	if (previousDirNull.get() == 0) { //Handle null.
		const auto comingI1ComingI2 = findComingIndex(i1, i2, currentDir);
		const auto comingI1 = comingI1ComingI2.first;
		const auto comingI2 = comingI1ComingI2.second;
		return findAllCollisionSegmentsNull(comingI1, comingI2,
				std::shared_ptr<const Dir>(new Dir(currentDir)), prevRes);
	} else {
		const auto previousDir = *previousDirNull; //Handle not null.

		if (currentDir == Dir::SameDir) {
			//When the current direction is the same, finding the collision segments get complicated.
			//The same direction is handled in order to achieve geometric robustness.

			//If the polygons are overlapping along the callipers at this point, it requires special handling.

			const auto isOverlappingFuncNext1 = next1(i1);
			const auto isOverlappingFuncNext2 = next2(i2);
			const auto isOverlappingFunc =
					[i1, i2, poly1Points, poly2Points, isOverlappingFuncNext1, isOverlappingFuncNext2]() {
						const auto p11 = (*poly1Points)[i1];
						const auto p12 = (*poly1Points)[isOverlappingFuncNext1];
						const auto p21 = (*poly2Points)[i2];
						const auto p22 = (*poly2Points)[isOverlappingFuncNext2];

						const auto lineLine = GeneralFunctions::handleLineLine(p11, p12, p21, p22);

						if ((*lineLine).getType() == ConvexCCWType::EmptyT) {
							return false;
						}
						else {
							return true;
						}
					};
			const auto isOverlapping = isOverlappingFunc();

			if (!isOverlapping) { //If non-overlapping, simply find the cross.

				const auto crossNull = getCrossNull(i1, i2, previousDir,
						currentDir); //Handle potential null.

				if (crossNull.get() == 0) { //Handle null.
					return std::shared_ptr<std::vector<CollisionSegment>>(0);
				} else {
					//Handle non-null.

					auto res = addAll(*prevRes, *crossNull);

					if (i1 == originIndex1 && i2 == originIndex2) {
						return std::shared_ptr<
								const std::vector<CollisionSegment>>(
								std::move(res));
					} else {

						const auto comingI1comingI2 = findComingIndex(i1,
								i2, currentDir);
						const auto comingI1 = comingI1comingI2.first;
						const auto comingI2 = comingI1comingI2.second;
						return findAllCollisionSegmentsNull(comingI1,
								comingI2,
								std::shared_ptr<const Dir>(
										new Dir(currentDir)),
								std::shared_ptr<
										const std::vector<CollisionSegment>>(
										std::move(res)));
					}
				}
			} else {
				//The polygons are overlapping along the callipers, complicating things.

				//Ensure that the correct directed overlapping head-points are included.

				const auto next1I1 = next1(i1);
				const auto next2I2 = next2(i2);
				const auto prevI1 = prev(i1, size1);
				const auto prevI2 = prev(i2, size2);
				const auto newResCalc =
						[i1, i2, poly1Points, poly2Points, next1I1, next2I2, prevI1, prevI2]() {

							const auto p11 = (*poly1Points)[i1];
							const auto p12 = (*poly1Points)[next1I1];
							const auto p21 = (*poly2Points)[i2];
							const auto p22 = (*poly2Points)[next2I2];

							const auto backsCalc = [p11, p12, p21, p22, i1, i2, prevI1, prevI2]() {

								if (p11.equal(p21)) {
									return std::shared_ptr<const std::vector<CollisionSegment>>(new std::vector<CollisionSegment>());
								}
								else {
									const auto back1Calc = [p11, p22, p21, i1, i2, prevI1]() {
										const auto overlap = GeneralFunctions::handlePointLine(
												Point(p11),
												*Line::createUtterlyUnsafelyNotChecked(p21, p22) //Safe, because p21 and p22 are always different.
										);
										if ((*overlap).getType() == ConvexCCWType::EmptyT) {
											return std::shared_ptr<const std::vector<CollisionSegment>>(new std::vector<CollisionSegment>());
										}
										else { //Can only be point here.
											const auto p = (*(*overlap).getAPoint()).myPoint;
											return std::shared_ptr<const std::vector<CollisionSegment>>(new std::vector<CollisionSegment>(
															{	{	CollisionSegment(prevI1, i2, p)}}
													));
										}
									};
									const auto back1 = back1Calc();

									const auto back2Calc = [p11, p12, p21, i1, i2, prevI2]() {

										const auto overlap = GeneralFunctions::handlePointLine(
												Point(p21),
												*Line::createUtterlyUnsafelyNotChecked(p11, p12) //Safe, because p11 and p12 are always different.
										);
										if ((*overlap).getType() == ConvexCCWType::EmptyT) {
											return std::shared_ptr<const std::vector<CollisionSegment>>(new std::vector<CollisionSegment>());
										}
										else { //Can only be point here.
											const auto p = (*(*overlap).getAPoint()).myPoint;
											return std::shared_ptr<const std::vector<CollisionSegment>>(new std::vector<CollisionSegment>(
															{	{	CollisionSegment(i1, prevI2, p)}}
													));
										}
									};
									const auto back2 = back2Calc();

									return std::shared_ptr<const std::vector<CollisionSegment>>(std::move(addAll(*back1, *back2)));
								}
							};
							const auto backs = backsCalc();

							return std::shared_ptr<const std::vector<CollisionSegment>>(std::move(
											addAll(*backs, *GeneralFunctions::getCollisionDirectedLineSegment(i1, i2, poly1Points, poly2Points))
									));
						};
				const auto newRes = newResCalc();

				auto res = std::shared_ptr<
						const std::vector<CollisionSegment>>(
						std::move(addAll(*prevRes, *newRes)));

				if (i1 == originIndex1 && i2 == originIndex2) {
					return res;
				} else {
					const auto comingI1comingI2 = findComingIndex(i1, i2,
							currentDir);
					const auto comingI1 = comingI1comingI2.first;
					const auto comingI2 = comingI1comingI2.second;
					return findAllCollisionSegmentsNull(comingI1, comingI2,
							std::shared_ptr<Dir>(new Dir(currentDir)), res);
				}
			}
		} else if ((previousDir == Dir::LeftDir
				&& currentDir == Dir::RightDir)
				|| (previousDir == Dir::RightDir
						&& currentDir == Dir::LeftDir)) {

			//When the callipers change relative direction cleanly (instead of having the "same direction"),
			//simply find the cross.

			const auto crossNull = getCrossNull(i1, i2, previousDir,
					currentDir); //Handle potential null.

			if (crossNull.get() == 0) { //Handle null.
				return std::shared_ptr<const std::vector<CollisionSegment>>(
						0);
			} else { //Handle not-null.

				const auto res = std::shared_ptr<
						const std::vector<CollisionSegment>>(
						std::move(addAll(*prevRes, *crossNull)));

				if (i1 == originIndex1 && i2 == originIndex2) {
					return res;
				} else {
					const auto comingI1comingI2 = findComingIndex(i1, i2,
							currentDir);
					const auto comingI1 = comingI1comingI2.first;
					const auto comingI2 = comingI1comingI2.second;
					return findAllCollisionSegmentsNull(comingI1, comingI2,
							std::shared_ptr<const Dir>(new Dir(currentDir)),
							res);
				}
			}
		} else {
			//If the current direction is either left or right, and the previous was the same,
			//collision segments at this point has already been handled or will be handled.

			const auto res = prevRes;

			if (i1 == originIndex1 && i2 == originIndex2) {
				return res;
			} else {
				const auto comingI1comingI2 = findComingIndex(i1, i2,
						currentDir);
				const auto comingI1 = comingI1comingI2.first;
				const auto comingI2 = comingI1comingI2.second;
				return findAllCollisionSegmentsNull(comingI1, comingI2,
						std::shared_ptr<const Dir>(new Dir(currentDir)),
						res);
			}
		}
	}
}

std::shared_ptr<const std::vector<CollisionSegment>> CollisionSegmentsFinder::getCollisionSegmentsNull() const {
	return findAllCollisionSegmentsNull(originIndex1, originIndex2,
			std::shared_ptr<const Dir>(0),
			std::shared_ptr<const std::vector<CollisionSegment>>(
					new std::vector<CollisionSegment>()));
}

}
