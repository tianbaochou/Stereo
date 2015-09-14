cmake_minimum_required(VERSION 3.2)
include_directories(
${STEREO_SOURCE_DIR}/external_libs/opencv/include
${STEREO_SOURCE_DIR}/external_libs/qt/include
${STEREO_SOURCE_DIR}/external_libs/directshow/include
${STEREO_SOURCE_DIR}/external_libs/atlmfc/include
)

link_directories(
${STEREO_SOURCE_DIR}/external_libs/opencv/x86/vc12/lib
${STEREO_SOURCE_DIR}/external_libs/qt/lib
${STEREO_SOURCE_DIR}/external_libs/atlmfc/lib
${STEREO_SOURCE_DIR}/external_libs/directshow/lib
)

#Set some libs we used
set(link_extern_libs
optimized opencv_core2411.lib debug opencv_core2411d.lib   #opencv
optimized opencv_video2411.lib debug opencv_video2411d.lib
optimized opencv_highgui2411.lib debug opencv_highgui2411d.lib
optimized opencv_imgproc2411.lib debug opencv_imgproc2411d.lib
optimized opencv_objdetect2411.lib debug opencv_objdetect2411d.lib
optimized opencv_contrib2411.lib debug opencv_contrib2411d.lib
optimized opencv_nonfree2411.lib debug opencv_nonfree2411d.lib
optimized opencv_ml2411.lib debug opencv_ml2411d.lib 
optimized opencv_calib3d2411.lib debug  opencv_calib3d2411d.lib
optimized opencv_features2d2411.lib debug opencv_features2d2411d.lib
optimized opencv_flann2411.lib debug opencv_flann2411d.lib
optimized QtCore4.lib debug QtCored4.lib
)

set(Stereo_src 
distortion_recifition.cpp
distortion_recifition.h
main.cpp
stereo_match.cpp
stereo_match.h
ui.cpp
ui.h
camera.h
camera.cpp
stereo.h
stereo.cpp
stereo_error.h
)

add_executable(Stereo_Vision ${Stereo_src})

#Link all of library
target_link_libraries(Stereo_Vision  ${link_extern_libs})

set_target_properties(Stereo_Vision 
    PROPERTIES
    DEBUG_POSTFIX "_D"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${STEREO_SOURCE_DIR}/bin/Debug"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${STEREO_SOURCE_DIR}/bin/Release"
)

# ouput .dll files to bin dir so that applications can use it
add_custom_command(TARGET Stereo_Vision
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${STEREO_SOURCE_DIR}/bin/$<CONFIG>
    COMMAND ${CMAKE_COMMAND} -E copy ${STEREO_SOURCE_DIR}/external_libs/opencv/x86/vc12/bin/opencv_core2411$<$<CONFIG:Debug>:d>.dll ${STEREO_SOURCE_DIR}/bin/$<CONFIG>
    COMMAND ${CMAKE_COMMAND} -E copy ${STEREO_SOURCE_DIR}/external_libs/opencv/x86/vc12/bin/opencv_video2411$<$<CONFIG:Debug>:d>.dll ${STEREO_SOURCE_DIR}/bin/$<CONFIG>
    COMMAND ${CMAKE_COMMAND} -E copy ${STEREO_SOURCE_DIR}/external_libs/opencv/x86/vc12/bin/opencv_highgui2411$<$<CONFIG:Debug>:d>.dll ${STEREO_SOURCE_DIR}/bin/$<CONFIG> 
    COMMAND ${CMAKE_COMMAND} -E copy ${STEREO_SOURCE_DIR}/external_libs/opencv/x86/vc12/bin/opencv_imgproc2411$<$<CONFIG:Debug>:d>.dll ${STEREO_SOURCE_DIR}/bin/$<CONFIG> 
    COMMAND ${CMAKE_COMMAND} -E copy ${STEREO_SOURCE_DIR}/external_libs/opencv/x86/vc12/bin/opencv_objdetect2411$<$<CONFIG:Debug>:d>.dll ${STEREO_SOURCE_DIR}/bin/$<CONFIG> 
    COMMAND ${CMAKE_COMMAND} -E copy ${STEREO_SOURCE_DIR}/external_libs/opencv/x86/vc12/bin/opencv_contrib2411$<$<CONFIG:Debug>:d>.dll ${STEREO_SOURCE_DIR}/bin/$<CONFIG> 
    COMMAND ${CMAKE_COMMAND} -E copy ${STEREO_SOURCE_DIR}/external_libs/opencv/x86/vc12/bin/opencv_nonfree2411$<$<CONFIG:Debug>:d>.dll ${STEREO_SOURCE_DIR}/bin/$<CONFIG> 	
    COMMAND ${CMAKE_COMMAND} -E copy ${STEREO_SOURCE_DIR}/external_libs/opencv/x86/vc12/bin/opencv_ml2411$<$<CONFIG:Debug>:d>.dll ${STEREO_SOURCE_DIR}/bin/$<CONFIG> 	
    COMMAND ${CMAKE_COMMAND} -E copy ${STEREO_SOURCE_DIR}/external_libs/opencv/x86/vc12/bin/opencv_calib3d2411$<$<CONFIG:Debug>:d>.dll ${STEREO_SOURCE_DIR}/bin/$<CONFIG> 		
    COMMAND ${CMAKE_COMMAND} -E copy ${STEREO_SOURCE_DIR}/external_libs/opencv/x86/vc12/bin/opencv_features2d2411$<$<CONFIG:Debug>:d>.dll ${STEREO_SOURCE_DIR}/bin/$<CONFIG> 		
    COMMAND ${CMAKE_COMMAND} -E copy ${STEREO_SOURCE_DIR}/external_libs/opencv/x86/vc12/bin/opencv_flann2411$<$<CONFIG:Debug>:d>.dll ${STEREO_SOURCE_DIR}/bin/$<CONFIG> 		
    COMMAND ${CMAKE_COMMAND} -E copy ${STEREO_SOURCE_DIR}/external_libs/qt/bin/QtCore$<$<CONFIG:Debug>:d>4.dll  ${STEREO_SOURCE_DIR}/bin/$<CONFIG>
    )


