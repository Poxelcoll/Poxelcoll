/* SimpleBinaryImage.hpp */

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

#ifndef POXELCOLL_BINARYIMAGE_SIMPLEBINARYIMAGE_HPP_
#define POXELCOLL_BINARYIMAGE_SIMPLEBINARYIMAGE_HPP_

#include <memory>
#include <iostream>

#include "../functional/Functional.hpp"

#include "BinaryImage.hpp"
#include "BinaryImageFactory.hpp"

using namespace poxelcoll::functional;

namespace poxelcoll {

/** \ingroup poxelcollbinaryimage
 *
 * A simple implementation of the binary image trait.
 */
class SimpleBinaryImage: public virtual BinaryImage {

private:

	const std::deque<std::deque<bool> >* imageSourceRows;
	const unsigned int myWidth;
	const unsigned int myHeight;

public:

	SimpleBinaryImage(const std::deque<std::deque<bool> >* imSourRs,
			const unsigned int width, const unsigned int height);

	const unsigned int width() const;

	const unsigned int height() const;

	const bool hasPoint(unsigned int x, unsigned int y) const;
};

/** \ingroup poxelcollbinaryimage
 *
 * The factory for the simple implementation of the binary image trait.
 */
class SimpleBinaryImageFactory: public virtual BinaryImageFactory {

public:
	const BinaryImage* createNull(
			const std::deque<std::deque<bool>>& imageSourceRows) const;
};

}

#endif /* POXELCOLL_BINARYIMAGE_SIMPLEBINARYIMAGE_HPP_ */
