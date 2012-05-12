/* BitsetBinaryImage.cpp */

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

#include "BitsetBinaryImage.hpp"

namespace poxelcoll {

BitsetBinaryImage::BitsetBinaryImage(const boost::dynamic_bitset<>* imSourRs,
		const unsigned int width, const unsigned int height) :
		myWidth(width), myHeight(height) {
	imageSourceRows = imSourRs;
}
;

BitsetBinaryImage::~BitsetBinaryImage() {
	delete imageSourceRows;
}
const unsigned int BitsetBinaryImage::width() const {
	return myWidth;
}
const unsigned int BitsetBinaryImage::height() const {
	return myHeight;
}

const bool BitsetBinaryImage::hasPoint(unsigned int x, unsigned int y) const {
	return (*imageSourceRows)[x + y * myWidth];
}

const BinaryImage* BitsetBinaryImageFactory::createNull(
		const std::deque<std::deque<bool>>& imageSourceRows) const {
	const auto height = imageSourceRows.size();
	if (height < 1) {
		std::cerr << "Invalid height" << std::endl;
		return 0;
	} else {
		const auto firstRow = imageSourceRows[0]; //Size is at least 1, so this is fine.
		const auto width = firstRow.size();

		if (exists(imageSourceRows,
				[&width](std::deque<bool> a) {return a.size() != width;})) {

			std::cerr << "Not the same length" << std::endl;
			return 0;
		} else {

			unsigned int r;
			unsigned int c;
			const int bitsetSize = width * height;

			boost::dynamic_bitset<>* bs = new boost::dynamic_bitset<>(
					bitsetSize);

			for (r = 0; r < height; r++) {
				for (c = 0; c < width; c++) {
					if (imageSourceRows[r][c]) {
						(*bs)[c + r * width] = 1;
					}
				}
			}

			return BitsetBinaryImage::createUnsafe(width, height, bs);
		}
	}
}

}
