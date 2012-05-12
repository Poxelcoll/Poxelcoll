/* Either.hpp */

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

#ifndef POXELCOLL_FUNCTIONAL_EITHER_HPP_
#define POXELCOLL_FUNCTIONAL_EITHER_HPP_

namespace poxelcoll {

namespace functional {

template <class A, class B>
class Either {
private:
	const std::shared_ptr<const A> left;
	const std::shared_ptr<const B> right;
	const bool isLeft;
private:
	Either(
			const std::shared_ptr<const A> aLeft,
			const std::shared_ptr<const B> aRight,
			const bool aIsLeft):
				left(aLeft),
				right(aRight),
				isLeft(aIsLeft)
		{

	}

public:
	static Either createLeft(const std::shared_ptr<const A> left) {
		if (left.get() == 0) {
			std::cerr << "Given left argument is null." << std::endl;
			throw 1;
		}
		else {
			return Either(
				left,
				std::shared_ptr<const B>(0),
				true
			);
		}
	}

	static Either createRight(const std::shared_ptr<const B> right) {
		if (right.get() == 0) {
			std::cerr << "Given right argument is null." << std::endl;
			throw 1;
		}
		else {
			return Either(
				std::shared_ptr<const A>(0),
				right,
				false
			);
		}
	}

	const std::shared_ptr<const A> getLeft() const {
		if (isLeft) {
			return left;
		}
		else {
			std::cerr << "Tried to get null left." << std::endl;
			throw 1;
		}
	}

	const std::shared_ptr<const B> getRight() const {
		if (!isLeft) {
			return right;
		}
		else {
			std::cerr << "Tried to get null right." << std::endl;
			throw 1;
		}
	}

	const bool getIsLeft() const {
		return isLeft;
	}

	const bool getIsRight() const {
		return !isLeft;
	}
};

}

}

#endif /* POXELCOLL_FUNCTIONAL_EITHER_HPP_ */
