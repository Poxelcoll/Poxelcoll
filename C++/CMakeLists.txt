cmake_minimum_required (VERSION 2.6)
project (Poxelcoll)

include_directories ("src/poxelcoll/")

file( GLOB HEADER_FILES
	src/poxelcoll/*.hpp
	src/poxelcoll/mask/*.hpp
	src/poxelcoll/binaryimage/*.hpp
	src/poxelcoll/collision/pairwise/*.hpp
	src/poxelcoll/collision/pixelperfect/*.hpp
	src/poxelcoll/geometry/convexccwpolygon/*.hpp
	src/poxelcoll/geometry/matrix/*.hpp
)
file( GLOB SOURCE_FILES
	src/poxelcoll/*.cpp
	src/poxelcoll/mask/*.cpp
	src/poxelcoll/binaryimage/*.cpp
	src/poxelcoll/collision/pairwise/*.cpp
	src/poxelcoll/collision/pixelperfect/*.cpp
	src/poxelcoll/geometry/convexccwpolygon/*.cpp
	src/poxelcoll/geometry/matrix/*.cpp
)

SET(Poxelcoll_SRCS ${HEADER_FILES} ${SOURCE_FILES})

ADD_LIBRARY( Poxelcoll SHARED ${Poxelcoll_SRCS} )

add_definitions (-std=gnu++0x -D__GXX_EXPERIMENTAL_CXX0X__ -fPIC)

