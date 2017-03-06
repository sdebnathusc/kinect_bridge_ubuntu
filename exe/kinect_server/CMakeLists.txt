# includes for this project
include_directories( ${CMAKE_CURRENT_SOURCE_DIRECTORY} )

include_directories( "${SPEECH_INCDIR}" )
link_directories( "${SPEECH_LIBDIR}" )

include_directories( "${KINECT_INCDIR}" )
link_directories( "${KINECT_LIBDIR}" )

include_directories( "${SNDFILE_INCDIR}" )
link_directories( "${SNDFILE_LIBDIR}" )

include_directories( "${POCO_INCDIR}" )
link_directories( "${POCO_LIBDIR}" )

include_directories( "${PNG_INCDIR}" )
link_directories( "${PNG_LIBDIR}" )

add_executable( test_kinect_device_RGB test_kinect_device_RGB.cpp )
target_link_libraries( test_kinect_device_RGB ${POCO_LIBS} ${PNG_LIBS} ${SNDFILE_LIBS} ${KINECT_LIBS} kinect_common messages atomics )

add_executable( test_kinect_device_depth test_kinect_device_depth.cpp )
target_link_libraries( test_kinect_device_depth ${POCO_LIBS} ${PNG_LIBS} ${SNDFILE_LIBS} ${KINECT_LIBS} kinect_common messages atomics )

add_executable( test_kinect_device_infrared test_kinect_device_infrared.cpp )
target_link_libraries( test_kinect_device_infrared ${POCO_LIBS} ${PNG_LIBS} ${SNDFILE_LIBS} ${KINECT_LIBS} kinect_common messages atomics )

add_executable( test_kinect_device_audio test_kinect_device_audio.cpp )
target_link_libraries( test_kinect_device_audio ${POCO_LIBS} ${PNG_LIBS} ${SNDFILE_LIBS} ${KINECT_LIBS} kinect_common messages atomics )

add_executable( kinect_logger kinect_logger.cpp )
target_link_libraries( kinect_logger ${POCO_LIBS} ${PNG_LIBS} ${SNDFILE_LIBS} ${KINECT_LIBS} kinect_common messages atomics )

add_executable( test_kinect_device_skeleton test_kinect_device_skeleton.cpp )
target_link_libraries( test_kinect_device_skeleton ${POCO_LIBS} ${KINECT_LIBS} kinect_common messages atomics )

add_executable( test_kinect_device_speech test_kinect_device_speech.cpp )
target_link_libraries( test_kinect_device_speech ${POCO_LIBS} ${KINECT_LIBS} kinect_common messages atomics )

add_executable( kinect_server kinect_server.cpp )
target_link_libraries( kinect_server ${POCO_LIBS} ${PNG_LIBS} ${SNDFILE_LIBS} ${KINECT_LIBS} kinect_common messages atomics )

set( executables
		kinect_logger
		kinect_server
		test_kinect_device_RGB
		test_kinect_device_depth
		test_kinect_device_infrared
		test_kinect_device_audio
		test_kinect_device_skeleton
		test_kinect_device_speech
)

# copy all DLLs alongside our exe
foreach( executable ${executables} )

# libpng
	file( GLOB png_dlls ${PNG_DLLDIR}/*.dll )
	foreach( dll ${png_dlls} )
		add_custom_command( TARGET ${executable} POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different
				${dll}
				$<TARGET_FILE_DIR:${executable}> )
	endforeach()

# sndfile
	file( GLOB sndfile_dlls ${SNDFILE_DLLDIR}/*.dll )
	foreach( dll ${sndfile_dlls} )
		add_custom_command( TARGET ${executable} POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different
				${dll}
				$<TARGET_FILE_DIR:${executable}> )
	endforeach()

# poco
	if( ${BUILD_TYPE} STREQUAL x64 )
		foreach( poco_lib ${POCO_LIBNAMES} )
			add_custom_command( TARGET ${executable} POST_BUILD
					COMMAND ${CMAKE_COMMAND} -E copy_if_different
					${POCO_DLLDIR}/Poco${poco_lib}64.dll
					$<TARGET_FILE_DIR:${executable}> )
		endforeach()
	else()
		foreach( poco_lib ${POCO_LIBNAMES} )
			add_custom_command( TARGET ${executable} POST_BUILD
					COMMAND ${CMAKE_COMMAND} -E copy_if_different
					${POCO_DLLDIR}/Poco${poco_lib}.dll
					$<TARGET_FILE_DIR:${executable}> )
		endforeach()
	endif()
endforeach()

set( speech_executables
	kinect_logger
	kinect_server
	test_kinect_device_speech
)
foreach( executable ${speech_executables} )
# copy grammar file alongside binaries
	add_custom_command( TARGET ${executable} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
		${CMAKE_CURRENT_SOURCE_DIR}/grammar.grxml
		$<TARGET_FILE_DIR:${executable}> )
endforeach()
