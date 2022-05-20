#####################################################
# Template for building RobWork dependent projects
# - the src should contain code for putting into libraries 
# - the plugins contain code for multiple RobWorkStudio plugins
# - the test for testing 
# - the example dir for illustrativ examples of your project
#
# Use config.cmake to change default settings and edit this
# file for adding additional dependencies to external projects  
#####################################################

#
# Test CMake version
#
CMAKE_MINIMUM_REQUIRED(VERSION 3.15.0)
include(CMakePrintHelpers)
# The name of the project.
PROJECT(PluginUIApp)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_VERBOSE_MAKEFILE True)

# Now set the RW/RWS root (edit this if necessary)
IF(WIN32)
  # On Windows platforms, the RW/RWS installers create the necessary environment variables
  IF(DEFINED RW_ROOT AND DEFINED RWS_ROOT)
    # SET(RW_ROOT "$ENV{RW_ROOT}")
    SET(RWSTUDIO_ROOT "${RWS_ROOT}")
    cmake_print_variables(
      RW_ROOT
      RWSTUDIO_ROOT
    )
  ELSE()
    SET(RW_ROOT "${ROOT}/../../../RobWork")
    SET(RWSTUDIO_ROOT "${ROOT}/../../../RobWorkStudio")
  ENDIF()
ELSEIF(UNIX)
  # On Ubuntu platforms, the Debian packages install into /usr/local or /usr
  IF(EXISTS "/usr/local/RobWork" AND EXISTS "/usr/local/RobWorkStudio")
    SET(RW_ROOT "/usr/local/RobWork")
    SET(RWSTUDIO_ROOT "/usr/local/RobWorkStudio")
  ELSEIF(EXISTS "/usr/RobWork" AND EXISTS "/usr/RobWorkStudio")
    SET(RW_ROOT "/usr/RobWork")
    SET(RWSTUDIO_ROOT "/usr/RobWorkStudio")
  ELSE()
    SET(RW_ROOT "${ROOT}/../../../RobWork")
    SET(RWSTUDIO_ROOT "${ROOT}/../../../RobWorkStudio")
  ENDIF()
ELSE()
  SET(RW_ROOT "${ROOT}/../../../RobWork")
  SET(RWSTUDIO_ROOT "${ROOT}/../../../RobWorkStudio")
ENDIF()

# If not the -DCMAKE_BUILD_TYPE option is given when running CMake, use Release as default.
IF( NOT DEFINED CMAKE_BUILD_TYPE )
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()
MESSAGE("-- Build type: " ${CMAKE_BUILD_TYPE})

#Include default settings for constructing a robwork dependent project
SET(RobWork_DIR ${RW_ROOT}/cmake) 
FIND_PACKAGE(RobWork REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )
MESSAGE("-- RobWork found: " ${RW_ROOT})

#Include default settings for constructing a robworkstudio dependent project
SET(RobWorkStudio_DIR ${RWSTUDIO_ROOT}/cmake)
FIND_PACKAGE(RobWorkStudio REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS} )
MESSAGE("-- RobWorkStudio found: " ${RWSTUDIO_ROOT})

# Set the output dir for generated libraries and binaries
IF(MSVC)
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Library directory" FORCE)
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Archive directory" FORCE)
ELSE()
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)
ENDIF()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Add the source files to build
set(SrcFiles SamplePlugin.cpp resources.qrc)

# The shared library to build:
add_library(SamplePlugin MODULE ${SrcFiles})
target_link_libraries(SamplePlugin PUBLIC ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})
target_include_directories(SamplePlugin PUBLIC ${ROBWORKSTUDIO_INCLUDE_DIRS} ${ROBWORK_INCLUDE_DIRS} )

if(CMAKE_VERSION VERSION_LESS 3.9)
    # Required in CMake < 3.9 to find generated ui_SamplePlugin.h file 
    target_include_directories(SamplePlugin PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
endif()

message("-- " ${PROJECT_NAME} " done!")

