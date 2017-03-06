cmake_minimum_required(VERSION 2.8.3)
project(generic_tests)

find_package(catkin REQUIRED)
find_package(roscpp REQUIRED)

include_directories( include ${catkin_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})

include_directories( include ${SNDFILE_INCDIR} )
link_directories( "${SNDFILE_LIBDIR}" )

include_directories( include ${POCO_INCLUDE_DIR} )
link_directories( "${POCO_LIB_DIR}" )

include_directories( include ${PNG_INCDIR} )
link_directories( "${PNG_LIBDIR}" )

FILE( GLOB executables RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp )

foreach( executable ${executables} )
	get_filename_component( executable_name ${executable} NAME_WE )
	add_definitions( "-std=c++11" )
	add_executable( ${executable_name} ${executable} )
	target_link_libraries( ${executable_name} ${roscpp_LIBRARIES} ${catkin_LIBRARIES} ${SNDFILE_LIBS} ${POCO_LIBS} ${PNG_LIBS} messages atomics )
endforeach()
