/* IMReverseList.hpp */

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

#ifndef POXELCOLL_FUNCTIONAL_IMREVERSELIST_HPP_
#define POXELCOLL_FUNCTIONAL_IMREVERSELIST_HPP_

#include <list>

namespace poxelcoll {

namespace functional {

template<class E>
class IMReverseListNode;
template<class E>
class IMReverseListNil;

template<class E>
class IMReverseList {
private:
	typedef std::shared_ptr<const IMReverseList<const E>> IML_SH; //Immutable list shared pointer.
	typedef std::shared_ptr<const E> IMLE_SH; //Immutable list element shared pointer.
public:
	typedef size_t size_type;

public:
	virtual ~IMReverseList() {}

	/**
	 * Selects all elements except the last.
	 */
	virtual const IML_SH initNull() const = 0;

	/**
	 * Selects the last element.
	 */
	virtual const IMLE_SH lastNull() const = 0;

	virtual const size_type size() const = 0;

	static const IML_SH nil() {
		return IML_SH(new IMReverseListNil<E>());
	}

	static const IML_SH append(const IML_SH init, const E e) {
		return IML_SH(new IMReverseListNode<E>(init, IMLE_SH(new E(e))));
	}

	static const IML_SH create(const E e) {
		return IML_SH(new IMReverseListNode<E>(
				nil(),
				IMLE_SH(new E(e))
		));
	}

	static const IML_SH addAll(const IML_SH firstPart, const IML_SH secondPart) {

		auto secondPartList = constructTo<std::list<E>>(secondPart);

		auto result = firstPart;

		while ((*secondPartList).size() != 0) {
			result = append(result, (*secondPartList).front());
			(*secondPartList).pop_front();
		}

		return result;
	}

	template <class A>
	static std::unique_ptr<A> constructTo(const IML_SH imReverseList) {

		A* a = new A();

		auto currentList = imReverseList;

		while ((*currentList).size() != 0) {
			a->push_front((*(*currentList).lastNull()));
			currentList = (*currentList).initNull();
		}

		return std::unique_ptr<A>(a);
	}
};

template<class E>
class IMReverseListNode: public virtual IMReverseList<E> {
private:
	typedef std::shared_ptr<const IMReverseList<const E>> IML_SH;
	typedef std::shared_ptr<const E> IMLE_SH;
	typedef size_t size_type;

private:
	const IML_SH init;
	const IMLE_SH element;
	const size_type mySize;
	friend class IMReverseList<E> ;

	IMReverseListNode(const IML_SH aInit, const IMLE_SH aElement) :
			init(aInit), element(aElement), mySize(
					aInit.get() != 0 ? (*aInit).size() + 1 : 1) {
	}

public:
	const IML_SH initNull() const {
		return init;
	}

	const IMLE_SH lastNull() const {
		return element;
	}

	const size_type size() const {
		return mySize;
	}
};

template<class E>
class IMReverseListNil: public virtual IMReverseList<E> {
private:
	typedef std::shared_ptr<const IMReverseList<const E>> IML_SH;
	typedef std::shared_ptr<const E> IMLE_SH;
	typedef size_t size_type;

public:
	const IML_SH initNull() const {
		return IML_SH(0);
	}

	const IMLE_SH lastNull() const {
		return IMLE_SH(0);
	}

	const size_type size() const {
		return 0;
	}
};

}

}

#endif /* POXELCOLL_FUNCTIONAL_IMREVERSELIST_HPP_ */
