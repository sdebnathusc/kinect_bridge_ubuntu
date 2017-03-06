include_directories( include ${POCO_INCDIR} )
link_directories( "${POCO_LIBDIR}" )

add_definitions( "-std=c++11" )
FILE( GLOB src_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp )

if( WIN32 )
	add_library( atomics STATIC ${src_files} )
else()
	add_library( atomics SHARED ${src_files} )
endif()
