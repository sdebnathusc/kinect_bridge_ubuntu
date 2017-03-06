include_directories( include ${SNDFILE_INCDIR} )
link_directories( "${SNDFILE_LIBDIR}" )

include_directories( include ${POCO_INCDIR} )
link_directories( "${POCO_LIBDIR}" )

include_directories( include ${PNG_INCDIR} )
link_directories( "${PNG_LIBDIR}" )

add_definitions( "-std=c++11" )
FILE( GLOB src_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp )

if( WIN32 )
	add_library( messages STATIC ${src_files} )
else()
	add_library( messages SHARED ${src_files} )
endif()
