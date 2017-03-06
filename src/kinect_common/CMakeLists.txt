include_directories( include ${SNDFILE_INCDIR} )
link_directories( "${SNDFILE_LIBDIR}" )

include_directories( include ${POCO_INCDIR} )
link_directories( "${POCO_LIBDIR}" )

include_directories( include ${PNG_INCDIR} )
link_directories( "${PNG_LIBDIR}" )

include_directories( include ${KINECT_INCDIR} )
link_directories( "${KINECT_LIBDIR}" )

FILE( GLOB src_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp )
add_library( kinect_common ${src_files} )
