PROJECT(ad16)
cmake_minimum_required(VERSION 2.8)

#LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_PREFIX}")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake) 

set(${PROJECT_NAME}_MAJOR_VERSION 00)
set(${PROJECT_NAME}_MINOR_VERSION 01)
set(${PROJECT_NAME}_PATCH_VERSION 00)
set(${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}.${${PROJECT_NAME}_PATCH_VERSION})
set(${PROJECT_NAME}_SOVERSION ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION})

#Moderate version of the compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fPIC -Wextra -ansi -Wuninitialized")

#use -DCMAKE_BUILD_TYPE=Debug in your cmake command to turn on the coverage option
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 --coverage")

# linker flags
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--export-dynamic")

FIND_PACKAGE(MtcaMappedDevice 00.17 REQUIRED)
include_directories(SYSTEM ${MtcaMappedDevice_INCLUDE_DIRS})

FIND_PACKAGE(mtca4uPy 00.01 REQUIRED)
include_directories(SYSTEM ${mtca4uPy_INCLUDE_DIRS})

FIND_PACKAGE(Boost COMPONENTS thread python REQUIRED)

#For some reason the system library has to be mentioned explicitly to the linker.
#This is strange because the thead library has it as a dependency.
FIND_PACKAGE(Boost COMPONENTS thread system unit_test_framework)
IF(Boost_UNIT_TEST_FRAMEWORK_FOUND)
  set(TESTING_IS_ENABLED "true")
  ENABLE_TESTING()
ELSE(Boost_UNIT_TEST_FRAMEWORK_FOUND)
  message("Boost unit_test_framework not found, disabling testing")
ENDIF(Boost_UNIT_TEST_FRAMEWORK_FOUND)

FIND_PACKAGE(PythonLibs REQUIRED)
include_directories( ${PYTHON_INCLUDE_DIRS} )

include_directories(${CMAKE_SOURCE_DIR}/include)
set(${PROJECT_NAME}_INCLUDE_DIRS ${${PROJECT_NAME}_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include/)
aux_source_directory(${CMAKE_SOURCE_DIR}/src library_sources)

MACRO( COPY_MAPPING_FILES_AND_EXAMPLES )
  foreach( FILE_TO_COPY ad16_scope_fmc25_r1224.mapp )
    configure_file( tests/${FILE_TO_COPY} ${PROJECT_BINARY_DIR}/${FILE_TO_COPY} copyonly)
  endforeach( FILE_TO_COPY )
  foreach( FILE_TO_COPY ad16.dmap ADC_scope.py )
    configure_file( examples/${FILE_TO_COPY} ${PROJECT_BINARY_DIR}/${FILE_TO_COPY} copyonly)
  endforeach( FILE_TO_COPY )
ENDMACRO( COPY_MAPPING_FILES_AND_EXAMPLES )

#Create the executables for automated unit testing.
#Currently we assume that they are all tests contained in one file, so
#each file gives a new executable. This section has to be adapted if this should change.
if(TESTING_IS_ENABLED)
#  include_directories(${CMAKE_SOURCE_DIR}/tests/include)
#  aux_source_directory(${CMAKE_SOURCE_DIR}/tests/src testSources)
#  add_library(${PROJECT_NAME}_TEST_LIBRARY ${testSources} )

  aux_source_directory(${CMAKE_SOURCE_DIR}/tests/executables_src testExecutables)
  foreach( testExecutableSrcFile ${testExecutables})
    #NAME_WE means the base name without path and (longest) extension
    get_filename_component(excutableName ${testExecutableSrcFile} NAME_WE)
    add_executable(${excutableName} ${testExecutableSrcFile})
    target_link_libraries(${excutableName} ${MtcaMappedDevice_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} ${PROJECT_NAME})
    add_test(${excutableName} ${excutableName})
  endforeach( testExecutableSrcFile )

  #The make coverage command is only available in debug mode
  IF(CMAKE_BUILD_TYPE STREQUAL "Debug")
    configure_file(cmake/Makefile.coverage.in
      ${PROJECT_BINARY_DIR}/Makefile.coverage @ONLY)
    add_custom_target(coverage
      make -f Makefile.coverage
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      COMMENT "Generating test coverage documentation" VERBATIM
      )
  ENDIF(CMAKE_BUILD_TYPE STREQUAL "Debug")

endif(TESTING_IS_ENABLED)

# C++ library
add_library(${PROJECT_NAME} SHARED ${library_sources} )
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION} SOVERSION ${${PROJECT_NAME}_SOVERSION})
target_link_libraries(${PROJECT_NAME} ${MtcaMappedDevice_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY})

#configure a header file which contains the version information for use in C++
#configure_file(cmake/version.h.in "${PROJECT_BINARY_DIR}/version.h" @ONLY)

# add a target to generate API documentation with Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
  add_custom_target(doc ALL
    ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen" VERBATIM
    )
  install(DIRECTORY ${CMAKE_BINARY_DIR}/doc DESTINATION .)
else(DOXYGEN_FOUND)
  message("Doxygen not found, documentation will not be build.")
endif(DOXYGEN_FOUND)


#Install the library and the executables
install(TARGETS ${PROJECT_NAME}
            RUNTIME DESTINATION bin
	    LIBRARY DESTINATION lib
  )

# copy mapping files and examples
COPY_MAPPING_FILES_AND_EXAMPLES()

#all include files go into include/PROJECT_NAME
#The exclusion of ${PROJECT_NAME} prevents the recursive installation of the files just being installed.
#The original headers are in include/*.h, the installed ones in include/PROJECT_NAME/*.h.
install(DIRECTORY ${${PROJECT_NAME}_INCLUDE_DIRS} DESTINATION include/${PROJECT_NAME}
          FILES_MATCHING PATTERN "*.h" 
	  PATTERN ".svn" EXCLUDE
	  PATTERN "${PROJECT_NAME}" EXCLUDE
	  )

#create the config files by filling the correct variables into the template (*.cmake.in)
#All variables @VARIABLE@ are replaced with the current values, but they have to be set first....
set(${PROJECT_NAME}_CONFIG_INCLUDE_DIRS ${MtcaMappedDevice_INCLUDE_DIRS})
set(${PROJECT_NAME}_REFERENCE_HEADER "MotorDriverCard.h")
set(${PROJECT_NAME}_CONFIG_LIBRARIES ${MtcaMappedDevice_LIBRARIES})
set(${PROJECT_NAME}_CONFIG_LIBRARY_DIRS ${MtcaMappedDevice_LIBRARY_DIRS})

# The instructions to create a debian package are loaded from a separate file
#include(${CMAKE_SOURCE_DIR}/cmake/prepare_debian_package.cmake)
