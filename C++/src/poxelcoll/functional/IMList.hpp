/* IMList.hpp */

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

#ifndef POXELCOLL_FUNCTIONAL_IMLIST_HPP_
#define POXELCOLL_FUNCTIONAL_IMLIST_HPP_

#include <memory>
#include <iostream>

namespace poxelcoll {

namespace functional {

template<class E>
class IMListNode;
template<class E>
class IMListNil;

template<class E>
class IMList {
private:
	typedef std::shared_ptr<const IMList<const E>> IML_SH; //Immutable list shared pointer.
	typedef std::shared_ptr<const E> IMLE_SH; //Immutable list element shared pointer.
public:
	typedef size_t size_type;

public:
	virtual ~IMList() {

	}

	/**
	 * Selects all elements except the first.
	 */
	virtual const IML_SH tailNull() const = 0;

	/**
	 * Selects the first element.
	 */
	virtual const IMLE_SH headNull() const = 0;

	virtual const size_type size() const = 0;

	static const IML_SH nil() {
		return IML_SH(new IMListNil<E>());
	}

	static const IML_SH prepend(const E & e, const IML_SH tail) {
		return IML_SH(new IMListNode<E>(IMLE_SH(new E(e)), tail));
	}

	static const IML_SH prepend2(const IMLE_SH & e, const IML_SH tail) {
		if (e.get() == 0) {
			std::cerr << "Argument may not be null." << std::endl;
			throw 1;
		}
		else {
			return IML_SH(new IMListNode<E>(e, tail));
		}
	}

	static const IML_SH create(const E & e) {
		return IML_SH(new IMListNode<E>(
				IMLE_SH(new E(e)),
				nil()
		));
	}

	static const IML_SH create2(const IMLE_SH & e) {
		return IML_SH(new IMListNode<E>(
				e,
				nil()
		));
	}

	template <class A>
	static const IML_SH constructFrom(const A & collection) {

		IML_SH result = nil();

		const auto rEnd = collection.rend();

		for (auto ri = collection.rbegin(); ri != rEnd; ri++) {
			const auto value = *ri;
			result = prepend(value, result);
		}

		return result;
	}

	template <class A>
	static std::unique_ptr<A> constructTo(const IML_SH imList) {

		A* a = new A();

		auto currentList = imList;

		while ((*currentList).size() != 0) {
			a->push_back((*(*currentList).headNull()));
			currentList = (*currentList).tailNull();
		}

		return std::unique_ptr<A>(a);
	}
};

template<class E>
class IMListNode: public virtual IMList<E> {
private:
	typedef std::shared_ptr<const IMList<const E>> IML_SH;
	typedef std::shared_ptr<const E> IMLE_SH;
	typedef size_t size_type;

private:
	const IMLE_SH element;
	const IML_SH tail;
	const size_type mySize;
	friend class IMList<E> ;

	IMListNode(const IMLE_SH aElement, const IML_SH aTail) :
			element(aElement), tail(aTail), mySize(
					aTail.get() != 0 ? (*aTail).size() + 1 : 1) {
	}

public:

	const IML_SH tailNull() const {
		return tail;
	}

	const IMLE_SH headNull() const {
		return element;
	}

	const size_type size() const {
		return mySize;
	}
};

template<class E>
class IMListNil: public virtual IMList<E> {
private:
	typedef std::shared_ptr<const IMList<const E>> IML_SH;
	typedef std::shared_ptr<const E> IMLE_SH;
	typedef size_t size_type;

public:

	const IML_SH tailNull() const {
		return IML_SH(0);
	}

	const IMLE_SH headNull() const {
		return IMLE_SH(0);
	}

	const size_type size() const {
		return 0;
	}
};

}

}

#endif /* POXELCOLL_FUNCTIONAL_IMLIST_HPP_ */
