/* IntersectionFromCollisionSegments.cpp */

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

#include "IntersectionFromCollisionSegments.hpp"
#include "GeneralFunctions.hpp"

namespace poxelcoll {

IntersectionFromCollisionSegments::IntersectionFromCollisionSegments(
		const std::shared_ptr<const std::vector<CollisionSegment>> aCollisionSegments,
		const std::shared_ptr<const std::vector<P>> aPoly1Points,
		const std::shared_ptr<const std::vector<P>> aPoly2Points) :
		collisionSegments(aCollisionSegments), poly1Points(aPoly1Points), poly2Points(
				aPoly2Points), size1((*aPoly1Points).size()), size2(
				(*aPoly2Points).size()) {
}

const int IntersectionFromCollisionSegments::next(const int a, const int size) const {
	return (a + 1) % size;
}

const int IntersectionFromCollisionSegments::prev(const int a, const int size) const {
	return (a - 1 < 0) ? size - 1 : a - 1;
}

const int IntersectionFromCollisionSegments::next1(const int i) const {
	return next(i, size1);
}

const int IntersectionFromCollisionSegments::next2(const int i) const {
	return next(i, size2);
}

//If the same direction, or if one/both is zero-vector. Note that zero-vectors should never occur.
const bool IntersectionFromCollisionSegments::sameDir(const P v1, const P v2) const {
	return ((v1.cross(v2)) == 0.0) && (v1.dot(v2)) >= 0.0;
}

//If opposite direction. Zero-vectors are defined as not opposite. Note that zero-vectors should never occur.
const bool IntersectionFromCollisionSegments::oppositeDir(const P v1, const P v2) const {
	return ((v1.cross(v2)) == 0.0) && (v1.dot(v2)) < 0.0;
}

const bool IntersectionFromCollisionSegments::cwOrder(std::shared_ptr<const std::list<P>> vs) const {

	if ((*vs).empty()) {
		return true;
	} else {
		const auto x = (*vs).front();
		auto xs = (*vs); //Copy and pop the front to get the tail. O(n) due to lack of immutable lists.
		xs.pop_front();
		const auto all = (*vs);

		const auto normIsZero = [](const P a) {
			return a.norm() == 0.0;
		};
		if (exists(all, normIsZero)) { //NOTE: Checking for no zero-vectors.
			return false;
		} else {

			const auto xNorma = x.normaUnsafe(); //NOTE: Assuming no zero-vectors.

			const auto transformFunction = [xNorma](const P a) {
				const auto aNormaUnsafe = a.normaUnsafe(); //NOTE: Assuming no zero-vectors.
					return std::pair<double, double>(xNorma.cross(aNormaUnsafe), xNorma.dot(aNormaUnsafe));
				};
			const auto transformed = map<std::list<P>,
					std::list<std::pair<double, double>>,
					decltype(transformFunction)>(xs, transformFunction);

			//Iterative style instead of functional, due to lack of immutable lists and efficient operations.

			bool checkResult = true;
			auto transformedCopy = *transformed;
			while (transformedCopy.size() >= 2) {
				const auto y1 = transformedCopy.front();
				const auto y2 = *(++transformedCopy.begin());

				const auto y1_1 = y1.first;
				const auto y2_1 = y2.first;
				if (y1_1 == 0.0) {
					if (y2_1 == 0.0) {
						checkResult = false;
						break;
					} else if (y2_1 > 0.0) {
						//Go on.
						transformedCopy.pop_front();
						continue;
					} else { // y2._1 < 0.0
						checkResult = false;
						break;
					}
				} else if (y1_1 > 0.0) {
					if (y2_1 == 0.0) {
						checkResult = false;
						break;
					} else if (y2_1 > 0.0) {
						if (y1.second < y2.second) {
							transformedCopy.pop_front();
							continue;
						} else {
							checkResult = false;
							break;
						}
					} else { // y2._1 < 0.0
						checkResult = false;
						break;
					}
				} else { // y1._1 < 0.0
					if (y2_1 == 0.0) {
						transformedCopy.pop_front();
						continue;
					} else if (y2_1 > 0.0) {
						transformedCopy.pop_front();
						continue;
					} else {
						if (y1.second > y2.second) {
							transformedCopy.pop_front();
							continue;
						} else {
							checkResult = false;
							break;
						}
					}
				}
			}

			const auto noDirectionOfXCheck =
					[](const std::pair<double, double> a) {
						return !(a.first == 0.0 && a.second >= 0.0);
					};
			const auto noDirectionOfX = forall<
					std::list<std::pair<double, double>>,
					decltype(noDirectionOfXCheck)>(*transformed,
					noDirectionOfXCheck); //Check that no vector is in the direction of x.

			return checkResult && noDirectionOfX;
		}
	}
}

//Should only be used on overlapping, in-same-direction line pieces, where the heads does not overlap.
const bool IntersectionFromCollisionSegments::ahead(const int i1, const int i2) const {

	const auto p11 = (*poly1Points)[i1];
	const auto p12 = (*poly1Points)[next(i1, size1)];

	const auto p22 = (*poly2Points)[next(i2, size2)];

	const auto v1 = p12.minus(p11);
	const auto v2 = p22.minus(p12);

	return (v1.dot(v2) < 0.0);
}

template<class F>
const bool IntersectionFromCollisionSegments::pointInside(const std::shared_ptr<const std::vector<P>> points,
		const F & nextFun, const P point) const {

	const auto pointsSize = (*points).size();
	for (unsigned int i = 0; i < pointsSize; i++) {
		const auto nextI = nextFun(i);
		const auto p1 = (*points)[i];
		const auto p2 = (*points)[nextI];
		const auto v1 = p2.minus(p1);
		const auto v2 = point.minus(p1);
		if (!(v1.cross(v2) >= 0.0)) {
			return false;
		}
	}
	return true;
}

const std::shared_ptr<const IMReverseList<const P>> IntersectionFromCollisionSegments::F1(
		const std::shared_ptr<const IMList<const CollisionSegment>> collisionSegments,
		const std::shared_ptr<const IMReverseList<const P>> res,
		const int i1, const int i2,
		const std::shared_ptr<const CollisionSegment> lastSegmentNull) const {

	if ((*collisionSegments).size() >= 1) {
		const auto x = *(*collisionSegments).headNull();

		if (x.gIndex1() == i1) {
			return constructIntersection(collisionSegments, res,
					lastSegmentNull);
		} else {

			const auto nextI1 = next(i1, size1);
			const auto p12 = (*poly1Points)[nextI1];

			const auto newResCalc =
					[res, p12]() {
						if ((*res).size() >= 1) {
							if ((*(*res).lastNull()).equal(p12)) {
								return res;
							}
							else {
								return IMReverseList<const P>::append(res, p12);
							}
						}
						else {
							return IMReverseList<const P>::append(IMReverseList<const P>::nil(), p12);
						}
					};
			const auto newRes = newResCalc();

			return F1(collisionSegments, newRes, nextI1, i2,
					lastSegmentNull);
		}
	} else { //collisionSegments.size() == 0

		if (lastSegmentNull.get() == 0) {
			return res;
		} else {
			const auto last = *lastSegmentNull;
			if (last.gIndex1() == i1) {
				return res;
			} else {

				const auto nextI1 = next(i1, size1);
				const auto p12 = (*poly1Points)[nextI1];

				const auto newResCalc =
						[res, p12]() {
							if ((*res).size() >= 1) {
								if ((*(*res).lastNull()).equal(p12)) {
									return res;
								}
								else {
									return IMReverseList<const P>::append(res, p12);
								}
							}
							else {
								return IMReverseList<const P>::append(IMReverseList<const P>::nil(), p12);
							}
						};
				const auto newRes = newResCalc();

				return F1(collisionSegments, newRes, nextI1, i2,
						lastSegmentNull);
			}
		}
	}
}

const std::shared_ptr<const IMReverseList<const P>> IntersectionFromCollisionSegments::F2(
		const std::shared_ptr<const IMList<const CollisionSegment>> collisionSegments,
		const std::shared_ptr<const IMReverseList<const P>> res,
		const int i1, const int i2,
		const std::shared_ptr<const CollisionSegment> lastSegmentNull) const {

	if ((*collisionSegments).size() >= 1) {
		const auto x = *(*collisionSegments).headNull();

		if (x.gIndex2() == i2) {
			return constructIntersection(collisionSegments, res,
					lastSegmentNull);
		} else {

			const auto nextI2 = next(i2, size2);
			const auto p22 = (*poly2Points)[nextI2];

			const auto newResCalc =
					[res, p22]() {
						if ((*res).size() >= 1) {
							if ((*(*res).lastNull()).equal(p22)) {
								return res;
							}
							else {
								return IMReverseList<const P>::append(res, p22);
							}
						}
						else {
							return IMReverseList<const P>::append(IMReverseList<const P>::nil(), p22);
						}
					};
			const auto newRes = newResCalc();

			return F2(collisionSegments, newRes, i1, nextI2,
					lastSegmentNull);
		}
	} else { //collisionSegments.size() == 0

		if (lastSegmentNull.get() == 0) {
			return res;
		} else {
			const auto last = *lastSegmentNull;
			if (last.gIndex2() == i2) {
				return res;
			} else {

				const auto nextI2 = next(i2, size2);
				const auto p22 = (*poly2Points)[nextI2];

				const auto newResCalc =
						[res, p22]() {
							if ((*res).size() >= 1) {
								if ((*(*res).lastNull()).equal(p22)) {
									return res;
								}
								else {
									return IMReverseList<const P>::append(res, p22);
								}
							}
							else {
								return IMReverseList<const P>::append(IMReverseList<const P>::nil(), p22);
							}
						};
				const auto newRes = newResCalc();

				return F2(collisionSegments, newRes, i1, nextI2,
						lastSegmentNull);
			}
		}
	}
}

const std::shared_ptr<const IMReverseList<const P>> IntersectionFromCollisionSegments::constructIntersection(
		const std::shared_ptr<const IMList<const CollisionSegment>> collisionSegments,
		const std::shared_ptr<const IMReverseList<const P>> res,
		const std::shared_ptr<const CollisionSegment> lastSegmentNull) const {

	if ((*collisionSegments).size() == 0) {
		return res;
	} else { //collisionSegments.size() >= 1

		const auto x = *(*collisionSegments).headNull();
		const auto xs = (*collisionSegments).tailNull();

		const auto i1 = x.gIndex1();
		const auto nextI1 = next(i1, size1);
		const auto nextNextI1(next(nextI1, size1));
		const auto i2 = x.gIndex2();
		const auto nextI2 = next(i2, size2);
		const auto nextNextI2(next(nextI2, size2));

		const auto p11 = (*poly1Points)[i1];
		const auto p12 = (*poly1Points)[nextI1];
		const auto p13 = (*poly1Points)[nextNextI1];

		const auto p21 = (*poly2Points)[i2];
		const auto p22 = (*poly2Points)[nextI2];
		const auto p23 = (*poly2Points)[nextNextI2];

		const auto v11 = p12.minus(p11);
		const auto v12 = p13.minus(p12);

		const auto v21 = p22.minus(p21);
		const auto v22 = p23.minus(p22);

		if (p12.equal(p22)) {

			if (!sameDir(v11, v21)) {

				if (cwOrder(
						std::shared_ptr<const std::list<P>>(
								new std::list<P>( { { v11.unaryMinus(), v12,
										v21.unaryMinus() } })))
						&& cwOrder(
								std::shared_ptr<const std::list<P>>(
										new std::list<P>(
												{ { v21.unaryMinus(), v22,
														v11.unaryMinus() } })))) {
					return IMReverseList<const P>::append(
							IMReverseList<const P>::nil(), p12);
				} else if (cwOrder(
						std::shared_ptr<const std::list<P>>(
								new std::list<P>(
										{ { v11.unaryMinus(),
												v21.unaryMinus(), v22, v12 } })))
						|| cwOrder(
								std::shared_ptr<const std::list<P>>(
										new std::list<P>( { {
												v11.unaryMinus(), v22, v12,
												v21.unaryMinus() } })))) {
					return F2(xs, IMReverseList<const P>::append(res, p12),
							i1, i2, lastSegmentNull);
				} else if (oppositeDir(v11, v12)) {

					return IMReverseList<const P>::append(
							IMReverseList<const P>::append(
									IMReverseList<const P>::nil(), p12),
							(p11.minus(p12).norm() < (p23.minus(p12).norm())) ?
									p11 : p23);
				} else {
					return F1(xs, IMReverseList<const P>::append(res, p12),
							i1, i2, lastSegmentNull);
				}
			} else {
				if (cwOrder(
						std::shared_ptr<const std::list<P>>(
								new std::list<P>( { { v11.unaryMinus(), v22,
										v12 } })))) {
					return F2(xs, IMReverseList<const P>::append(res, p12),
							i1, i2, lastSegmentNull);
				} else {
					return F1(xs, IMReverseList<const P>::append(res, p12),
							i1, i2, lastSegmentNull);
				}
			}
		} else {

			if (oppositeDir(v11, v21)) {

				return IMReverseList<const P>::append(
						IMReverseList<const P>::append(
								IMReverseList<const P>::nil(),
								((p12.minus(p11)).norm()
										< (p12.minus(p22)).norm()) ?
										p11 : p22),
						((p22.minus(p21)).norm() < (p22.minus(p12)).norm()) ?
								p21 : p12);
			} else if (sameDir(v11, v21)) {
				if (ahead(i1, i2)) {
					return F2(xs, IMReverseList<const P>::append(res, p22),
							i1, i2, lastSegmentNull);
				} else {
					return F1(xs, IMReverseList<const P>::append(res, p12),
							i1, i2, lastSegmentNull);
				}
			}

			else {

				const auto collisionPointsInfo =
						GeneralFunctions::getCollisionDirectedLineSegment(
								i1, i2, poly1Points, poly2Points);
				const auto collisionPoint =
						(*collisionPointsInfo).front().gCollisionPoint();

				const auto testFunction =
						[collisionPoint](const P p) {return p.equal(collisionPoint);};
				if (!exists(std::vector<P>( { { p12, p22 } }),
						testFunction)) {
					if (v11.cross(v21) > 0.0) {
						return F2(xs,
								IMReverseList<const P>::append(res,
										collisionPoint), i1, i2,
								lastSegmentNull);
					} else {
						return F1(xs,
								IMReverseList<const P>::append(res,
										collisionPoint), i1, i2,
								lastSegmentNull);
					}
				} else if (p12.equal(collisionPoint)) {
					if (cwOrder(
							std::shared_ptr<const std::list<P>>(
									new std::list<P>(
											{ { v21, v11.unaryMinus(), v12,
													v21.unaryMinus() } })))) {
						return IMReverseList<const P>::append(
								IMReverseList<const P>::nil(),
								collisionPoint);
					} else if (cwOrder(
							std::shared_ptr<const std::list<P>>(
									new std::list<P>( { { v11.unaryMinus(),
											v21, v12 } })))) {
						return F2(xs,
								IMReverseList<const P>::append(res,
										collisionPoint), i1, i2,
								lastSegmentNull);
					} else {
						return F1(xs,
								IMReverseList<const P>::append(res,
										collisionPoint), i1, i2,
								lastSegmentNull);
					}
				} else {

					if (cwOrder(
							std::shared_ptr<const std::list<P>>(
									new std::list<P>(
											{ { v11, v21.unaryMinus(), v22,
													v11.unaryMinus() } })))) {
						return IMReverseList<const P>::append(
								IMReverseList<const P>::nil(),
								collisionPoint);
					} else if (cwOrder(
							std::shared_ptr<const std::list<P>>(
									new std::list<P>( { { v21.unaryMinus(),
											v11, v22 } })))) {
						return F1(xs,
								IMReverseList<const P>::append(res,
										collisionPoint), i1, i2,
								lastSegmentNull);
					} else {
						return F2(xs,
								IMReverseList<const P>::append(res,
										collisionPoint), i1, i2,
								lastSegmentNull);
					}
				}
			}
		}
	}
}

std::shared_ptr<const ConvexCCWPolygon> IntersectionFromCollisionSegments::getIntersectionFromCollisionSegments() const {

	const auto intersectingPolygonCalc =
			[collisionSegments, size1, this]() {

				if ((*collisionSegments).empty()) {

					//        if (pointInside(poly1Points, a => next(a, size1), poly2Points.head)) {
					const auto next1 = [size1, this](const int a) {
						return next(a, size1);
					};
					const auto next2 = [size2, this](const int a) {
						return next(a, size2);
					};
					const auto iden = [](const P a) {return a;};

					if (pointInside<decltype(next1)>(poly1Points, next1, (*poly2Points).front())) {
						return map<std::vector<P>, std::list<P>, decltype(iden)>(*poly2Points, iden);
					}
					else if (pointInside<decltype(next2)>(poly2Points, next2, (*poly1Points).front())) {
						return map<std::vector<P>, std::list<P>, decltype(iden)>(*poly1Points, iden);
					}
					else {
						return std::unique_ptr<std::list<P>>(new std::list<P>());
					}
				}
				else {

					auto results = constructIntersection(
							IMList<const CollisionSegment>::constructFrom(*collisionSegments),
							IMReverseList<const P>::nil(),
							std::shared_ptr<const CollisionSegment>(new CollisionSegment((*collisionSegments).front()))
					);
					return IMReverseList<const P>::constructTo<std::list<P>>(results);

				}
			};
	const auto intersectingPolygon = intersectingPolygonCalc();

	const auto intersectingPolygonSize = (*intersectingPolygon).size();

	if (intersectingPolygonSize == 0) {
		return Empty::getEmpty();
	} else if (intersectingPolygonSize == 1) {
		return std::shared_ptr<const Point>(
				new Point((*intersectingPolygon).front()));
	} else if (intersectingPolygonSize == 2) {
		return Line::create((*intersectingPolygon).front(),
				*(++(*intersectingPolygon).begin()));
	} else { //size >= 3.
		const auto p1 = (*intersectingPolygon).front();
		const auto p2 = *(++(*intersectingPolygon).begin());
		const auto p3 = *(++++(*intersectingPolygon).begin());
		auto rest = *intersectingPolygon;
		rest.pop_front();
		rest.pop_front();
		rest.pop_front();

		const auto iden = [](const P p) {return p;};
		auto restVector = map<std::list<P>, std::vector<P>, decltype(iden)>(
				rest, iden);

		return Polygon::createUtterlyUnsafelyNotChecked(p1, p2, p3,
				std::shared_ptr<std::vector<P>>(std::move(restVector)));
	}

}

}
