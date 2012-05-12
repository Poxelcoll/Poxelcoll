
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

/**
 * \defgroup poxelcollbinaryimage poxelcoll_binaryimage
 * \ingroup poxelcoll
 * 
 * The binary image package contains classes to represent
 * binary images, as well as factory methods for them.
 *
 * See BinaryImage for what is understood by a "binary image".
 *
 * The purpose of the binary image package in the library is to have a
 * central, simple and efficient representation of binary images
 * that supports different implementations. This is useful, since binary images
 * can be implemented in several different ways, each with different
 * performance characteristics (eg., a binary image supported by a bitset),
 * and by having one single representation, changing the used binary image
 * and profiling it for performance becomes significantly easier.
 */
