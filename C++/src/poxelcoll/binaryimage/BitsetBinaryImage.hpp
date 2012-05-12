/* BitsetBinaryImage.hpp */

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

#ifndef POXELCOLL_BINARYIMAGE_BITSETBINARYIMAGE_HPP_
#define POXELCOLL_BINARYIMAGE_BITSETBINARYIMAGE_HPP_

#include <bitset>
#include <deque>
#include <memory>

#include <boost/dynamic_bitset.hpp>

#include "BinaryImage.hpp"
#include "BinaryImageFactory.hpp"

#include "../functional/Functional.hpp"

using namespace poxelcoll::functional;

namespace poxelcoll {

/** \ingroup poxelcollbinaryimage
 *
 * A binary image utilising a bitset to get specific performance characteristics.
 *
 * Notably, a bitset generally requires relatively very little memory,
 * while accessing points is a bit more expensive compared to other methods.
 * See for instance http://en.wikipedia.org/wiki/Bitset.
 */
class BitsetBinaryImage: public virtual BinaryImage {

private:

	const boost::dynamic_bitset<>* imageSourceRows;
	const unsigned int myWidth;
	const unsigned int myHeight;

public:

	BitsetBinaryImage(const boost::dynamic_bitset<>* imSourRs,
			const unsigned int width, const unsigned int height);

	~BitsetBinaryImage();
	const unsigned int width() const;
	const unsigned int height() const;

	const bool hasPoint(unsigned int x, unsigned int y) const;

	static BitsetBinaryImage* createUnsafe(int width, int height,
			boost::dynamic_bitset<>* imageSourceRows) {
		return new BitsetBinaryImage(imageSourceRows, width, height);
	}
};

/** \ingroup poxelcollbinaryimage
 *
 * The factory for the bitset binary image.
 */
class BitsetBinaryImageFactory: public virtual BinaryImageFactory {

public:
	const BinaryImage* createNull(
			const std::deque<std::deque<bool>>& imageSourceRows) const;
};

}

#endif /* POXELCOLL_BINARYIMAGE_BITSETBINARYIMAGE_HPP_ */
