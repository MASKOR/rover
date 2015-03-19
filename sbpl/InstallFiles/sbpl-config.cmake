# ===================================================================================
#  The SBPL CMake configuration file
#
#             ** File generated automatically, do not modify **
#
#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    FIND_PACKAGE(sbpl REQUIRED )
#    INCLUDE_DIRECTORIES(${SBPL_INCLUDE_DIRS})
#    TARGET_LINK_LIBRARIES(MY_TARGET_NAME ${SBPL_LIBRARIES})
#
#    This file will define the following variables:
#      - SBPL_LIBRARIES      : The list of libraries to links against as absolute path
#      - SBPL_LIBRARY_DIRS   : The directory where lib files are. Calling
#                              LINK_DIRECTORIES with this path is NOT needed.
#      - SBPL_INCLUDE_DIRS   : The SBPL include directories.
#
# Based on the example CMake Tutorial
# http://www.vtk.org/Wiki/CMake/Tutorials/How_to_create_a_ProjectConfig.cmake_file
# and OpenCVConfig.cmake.in from OpenCV
# ===================================================================================

 
set(SBPL_INCLUDE_DIRS "/usr/local/include")
set(SBPL_LIBRARY_DIRS "/usr/local/lib")
 

# Set library names as absolute paths:
set(SBPL_LIBRARIES "/usr/local/lib/libsbpl.so")
