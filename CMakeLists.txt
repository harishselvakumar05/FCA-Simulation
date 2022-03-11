
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set (CMAKE_AUTOMOC ON)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)


include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})


set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


add_library(SubscalePlugin_msgs ${PROTO_SRCS})
target_link_libraries(SubscalePlugin_msgs ${PROTOBUF_LIBRARY})

add_library(SubscalePlugin SHARED ${headers_MOC} SubscalePlugin.cc)
target_link_libraries(SubscalePlugin ${GAZEBO_LIBRARIES} 
    	${PROTOBUF_LIBRARIES})
