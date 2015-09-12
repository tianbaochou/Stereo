Cmake_minimum_required(VERSION 3.2)
project(STEREO)

set(OpencvDir ${STEREO_SOURCE_DIR}/external_libs/opencv)
set(QtDir ${STEREO_SOURCE_DIR}/external_libs/qt)
set(AtlmfcDir ${STEREO_SOURCE_DIR}/external_libs/atlmfc)
set(DirectShowDir ${STEREO_SOURCE_DIR}/external_libs/directshow)
add_subdirectory(src)
