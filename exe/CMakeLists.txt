#include_directories( "${SNDFILE_INCDIR}" )
#link_directories( "${SNDFILE_LIBDIR}" )
#
#include_directories( "${POCO_INCDIR}" )
#link_directories( "${POCO_LIBDIR}" )
#
#include_directories( "${PNG_INCDIR}" )
#link_directories( "${PNG_LIBDIR}" )
#
#set( EXE_SOURCES
##		test_binary_io.cpp
##		test_compressed_io.cpp
##		test_compressed_io_file.cpp
##		test_compressed_io_file_thread.cpp
##		test_serialize_struct.cpp
##		test_websocket_server.cpp
##		test_websocket_client.cpp
##		test_sizeof_string_class.cpp
##		test_thread_pool.cpp
##		test_thread_pool_messaging.cpp
##		test_thread_pool_mem_pool_messaging.cpp
##		test_atomic.cpp
##		test_atomic2.cpp
##		test_atomic3.cpp
##		test_thread_pool_messaging_atomics.cpp
##		test_thread_pool_messaging_atomics_mem_pool.cpp
##		test_mem_pool_allocator.cpp
##		test_mem_block_allocator.cpp
##		test_mem_pool_buffer.cpp
##		test_thread_pool_messaging_atomics_mem_pool2.cpp
##		test_message.cpp
##		test_compressed_binary_io.cpp
##		test_message2.cpp
##		test_counting_stream.cpp
##		test_message3.cpp
##		test_message4.cpp
##		test_message8.cpp
##		test_message9.cpp
##		test_message8_2.cpp
##		test_template_partial_int.cpp
##		test_wait_for.cpp
##		test_container_is_same.cpp
#)
#
#foreach( exe_source ${EXE_SOURCES} )
#	get_filename_component( exe_source_name ${exe_source} NAME_WE )
#	set( exe_source_src ${exe_source} )
#
#	add_executable( ${exe_source_name} ${exe_source_src} )
#
#	target_link_libraries( ${exe_source_name} ${POCO_LIBS} ${PNG_LIBS} ${SNDFILE_LIBS} messages atomics )
#
#	file( GLOB png_dlls ${PNG_DLLDIR}/*.dll )
#	foreach( dll ${png_dlls} )
#		add_custom_command( TARGET ${exe_source_name} POST_BUILD
#				COMMAND ${CMAKE_COMMAND} -E copy_if_different
#				${dll}
#				$<TARGET_FILE_DIR:${exe_source_name}> )
#	endforeach()
#
#	file( GLOB sndfile_dlls ${SNDFILE_DLLDIR}/*.dll )
#	foreach( dll ${sndfile_dlls} )
#		add_custom_command( TARGET ${exe_source_name} POST_BUILD
#				COMMAND ${CMAKE_COMMAND} -E copy_if_different
#				${dll}
#				$<TARGET_FILE_DIR:${exe_source_name}> )
#	endforeach()
#
#	file( GLOB poco_dlls ${POCO_DLLDIR}/*.dll )
#	foreach( dll ${poco_dlls} )
#		add_custom_command( TARGET ${exe_source_name} POST_BUILD
#				COMMAND ${CMAKE_COMMAND} -E copy_if_different
#				${dll}
#				$<TARGET_FILE_DIR:${exe_source_name}> )
#	endforeach()
#endforeach( exe_source )
#
if( WIN32 )
	add_subdirectory( kinect_server )
else()
	add_subdirectory( kinect_client )
endif()
add_subdirectory( generic_tests )
