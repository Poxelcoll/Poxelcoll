/* Functional.hpp */

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

#ifndef POXELCOLL_FUNCTIONAL_FUNCTIONAL_HPP_
#define POXELCOLL_FUNCTIONAL_FUNCTIONAL_HPP_

#include <memory>
#include <algorithm>
#include <map>
#include <iostream>

namespace poxelcoll {

namespace functional {

/** Test whether a predicate testFunction holds true for any element in the given collection.
 *
 * @param collection a collection of some type of element T
 * @param testFunction a predicate that takes an element of type T and gives a truth-value for the element
 * @return whether the predicate testFunction holds for at least one element in the given collection
 */
template <class E, class F>
bool exists(const E & collection, const F & testFunction) {

	const auto startI = collection.begin();
	const auto endI = collection.end();

	auto i = startI;
	while (i != endI) {
		if (testFunction(*i)) {
			return true;
		}

		i++;
	}

	return false;
}

template <class E, class F>
bool forall(const E & collection, const F & testFunction) {

	const auto startI = collection.begin();
	const auto endI = collection.end();

	auto i = startI;
	while (i != endI) {
		if (!testFunction(*i)) {
			return false;
		}

		i++;
	}

	return true;
}

/** Constructs a new collection by concatenating the first
 * collection with the second collection.
 *
 * The collections must be of the same type.
 *
 * This has been tested for vectors, deques, lists and sets.
 *
 * The current implementation assumes that insertion at the "end()"
 * is always relatively efficient.
 *
 * For instance: addAll(list(1, 2, 3, -10), list(42, 42)) == list(1, 2, 3, -10, 42, 42)
 * For instance: addAll(set(1, 2, 3, -10), set(42, 42)) == set(1, 2, 3, -10, 42)
 *
 * @param collection1 the first collection
 * @param collection2 the second collection
 * @return the result of concatenating collection1 with collection 2
 */
template <class E>
std::unique_ptr<E> addAll(const E & collection1, const E & collection2) {

	E* res = new E();
	{
		const auto startI1 = collection1.begin();
		const auto endI1 = collection1.end();

		auto i = startI1;
		while (i != endI1) {
			res->insert(res->end(), *i);
			i++;
		}
	}
	{
		const auto startI2 = collection2.begin();
		const auto endI2 = collection2.end();

		auto i = startI2;
		while (i != endI2) {
			res->insert(res->end(), *i);
			i++;
		}
	}

	return std::unique_ptr<E>(res);
}

template <class A, class B = A, class F>
std::unique_ptr<B> map(const A & collection, const F & transformationFunction) {

	B* resultCollection = new B();
	{
		const auto startI = collection.begin();
		const auto endI = collection.end();

		auto i = startI;
		while (i != endI) {
			resultCollection->insert(resultCollection->end(), transformationFunction(*i));
			i++;
		}
	}
	return std::unique_ptr<B>(resultCollection);
}

template <class A, class B>
B sum(const A & collection) {

	B sum = B();
	{
		const auto startI = collection.begin();
		const auto endI = collection.end();

		auto i = startI;
		while (i != endI) {
			sum += *i;
			i++;
		}
	}
	return sum;
}

template <class A, class B, class F>
B reduceDefault(const A & collection, const B & defaultValue, const F & reductionFunction) {

	if (collection.size() < 1) {
		return defaultValue;
	}
	else if (collection.size() == 1) {
		return *collection.begin();
	}
	else {
		//NOTE: Collection size is >= 2.

		B reductionValue = *collection.begin();
		{
			const auto startPlusOneI = collection.begin() + 1;
			const auto endI = collection.end();

			auto i = startPlusOneI;
			while (i != endI) {
				reductionValue = reductionFunction(reductionValue, *i);
				i++;
			}
		}
		return reductionValue;
	}
}

template <class A, class F>
std::unique_ptr<std::vector<typename A::value_type>> sortByToVector(const A & collection, const F & sortFunction){

	auto copyOfCollection = new std::vector<typename A::value_type>();

	{
		const auto startI = collection.begin();
		const auto endI = collection.end();

		auto i = startI;
		while (i != endI) {
			copyOfCollection->push_back(*i);
			i++;
		}
	}

	{
		const auto startI = copyOfCollection->begin();
		const auto endI = copyOfCollection->end();
		std::stable_sort(startI, endI, sortFunction);
	}

	return std::unique_ptr<std::vector<typename A::value_type>>(copyOfCollection);
}

template <class A, class E, class F>
std::unique_ptr<std::map<E, std::shared_ptr<A>>> groupBy(const A & collection, const F & groupByFunction) {

	auto resultMap = new std::map<E, std::shared_ptr<A>>();

	{
		const auto startI = collection.begin();
		const auto endI = collection.end();

		auto i = startI;
		while (i != endI) {
			E value = groupByFunction(*i);
			if (resultMap->count(value) < 1) {
				auto newCollection = std::shared_ptr<A>(new A());
				(*newCollection).insert((*newCollection).end(), *i);
				resultMap->insert(std::pair<E, std::shared_ptr<A>>(value, std::move(newCollection)));
			}
			else {
				auto foundPointer = resultMap->find(value);
				(*(*foundPointer).second).insert((*(*foundPointer).second).end(), *i);
			}
			i++;
		}
	}

	return std::unique_ptr<std::map<E, std::shared_ptr<A>>>(resultMap);
}

template <class A, class E>
const E minDefault(const A & collection, const E & defaultValue) {

	if (collection.empty()) {
		return defaultValue;
	}
	else {

		E value = *collection.begin();

		const auto startIPlusOne = collection.begin()++;
		const auto endI = collection.end();

		for (auto i = startIPlusOne; i != endI; i++) {
			if (value > *i) { //Min.
				value = *i;
			}
		}

		return value;
	}
}

template <class A, class E>
const E maxDefault(const A & collection, const E & defaultValue) {

	if (collection.empty()) {
		return defaultValue;
	}
	else {

		E value = *collection.begin();

		const auto startIPlusOne = collection.begin()++;
		const auto endI = collection.end();

		for (auto i = startIPlusOne; i != endI; i++) {
			if (value < *i) { //Max.
				value = *i;
			}
		}

		return value;
	}
}

template <class A, class F>
std::unique_ptr<A> filter(const A & collection, const F & filterFunction) {
	if (collection.empty()) {
		return std::unique_ptr<A>(new A());
	}
	else {

		A* filteredCollection = new A();

		const auto startI = collection.begin();
		const auto endI = collection.end();

		for (auto i = startI; i != endI; i++) {
			if (filterFunction(*i)) { //Max.
				filteredCollection->push_back(*i);
			}
		}

		return std::unique_ptr<A>(filteredCollection);
	}
}

template <class A, class B, class E>
std::unique_ptr<std::vector<E>> zip(const A & collection1, const B & collection2) {

	const auto beginI1 = collection1.begin();
	const auto beginI2 = collection2.begin();
	const auto endI1 = collection1.end();
	const auto endI2 = collection2.end();

//	typedef A::value_type ele1Type;
//	typedef B::value_type ele2Type;

	auto result = new std::vector<E>();

	auto i1 = beginI1;
	auto i2 = beginI2;

	for (; i1 != endI1 && i2 != endI2; i1++, i2++) {
		result->push_back(E(*i1, *i2));
	}

	return std::unique_ptr<std::vector<E>>(result);
}

template <class E>
std::unique_ptr<std::vector<E>> until(const E beginning, const E ending, const E by) {

	if (by == 0) {
		std::cerr << "Illegal 'by' argument." << std::endl;
		throw 1;
	}
	else {
		auto result = new std::vector<E>();

		if (by > 0) {
			for (auto i = beginning; i < ending; i += by) { // "<" since "until".
				result->push_back(i);
			}
		}
		else {
			for (auto i = beginning; i > ending; i += by) { // ">" since "until".
				result->push_back(i);
			}
		}

		return std::unique_ptr<std::vector<E>>(result);
	}
}

template <class E>
std::unique_ptr<std::vector<E>> to(const E beginning, const E ending, const E by) {

	if (by == 0) {
		std::cerr << "Illegal 'by' argument." << std::endl;
		throw 1;
	}
	else {
		auto result = new std::vector<E>();

		if (by > 0) {
			for (auto i = beginning; i <= ending; i += by) { // "<=" since "to".
				result->push_back(i);
			}
		}
		else {
			for (auto i = beginning; i >= ending; i += by) { // ">=" since "to".
				result->push_back(i);
			}
		}

		return std::unique_ptr<std::vector<E>>(result);
	}
}

template <class A, class R, class F>
std::unique_ptr<R> foldLeft(const A & collection, const R & initResult, const F & transformFunction) {

	auto result = initResult;

	const auto initI = collection.begin();
	const auto endI = collection.end();

	for (auto i = initI; i != endI; i++) {
		result = transformFunction(result, *i);
	}

	return std::unique_ptr<R>(new R(result));
}

template <class A, class B>
std::unique_ptr<B> flatten(const A & collection) {

	B* resultCollection = new B();
	{
		const auto startI = collection.begin();
		const auto endI = collection.end();

		auto i = startI;
		while (i != endI) {

			const auto startInnerI = (*i).begin();
			const auto endInnerI = (*i).end();

			auto innerI = startInnerI;
			while (innerI != endInnerI) {
				resultCollection->insert(resultCollection->end(), *innerI);
				innerI++;
			}

			i++;
		}
	}
	return std::unique_ptr<B>(resultCollection);
}

}

}

#endif /* POXELCOLL_FUNCTIONAL_FUNCTIONAL_HPP_ */
