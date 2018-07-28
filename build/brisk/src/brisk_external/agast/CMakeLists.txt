cmake_minimum_required(VERSION 2.8.11)

project(agast)

if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# require OpenCV
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS}) 

if(APPLE)
  add_definitions(-Wall -pedantic -DHAVE_OPENCV -DAGAST_GLOG -std=c++11)
else()
  add_definitions(-Wall -Werror -pedantic -DHAVE_OPENCV -DAGAST_GLOG -std=c++0x)
endif()

# build the agast library
add_library(${PROJECT_NAME} STATIC
	src/agast5-8.cc  
	src/agast5-8-nms.cc  
	src/agast7-12d.cc  
	src/agast7-12d-nms.cc  
	src/agast7-12s.cc  
	src/agast7-12s-nms.cc  
	src/ast-detector.cc  
	src/non-maximum-suppression.cc  
	src/oast9-16.cc  
	src/oast9-16-nms.cc)

# and link it                     
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# installation if required
install(TARGETS ${PROJECT_NAME} EXPORT briskTargets ARCHIVE DESTINATION ${INSTALL_LIB_DIR})
install(DIRECTORY include/ DESTINATION ${INSTALL_INCLUDE_DIR} COMPONENT dev FILES_MATCHING PATTERN "*.h")
