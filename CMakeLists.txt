cmake_minimum_required(VERSION 3.7...3.25)

if(${CMAKE_VERSION} VERSION_LESS 3.12)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
endif()

project(
    bbstlogger 
    VERSION 1.0
    DESCRIPTION "Data Logger for the Ballon Borne Solar Telescope"
    LANGUAGES CXX
)

add_executable(bbstlogger bbstlogger.cpp SolarPosition.cpp)

set_target_properties(bbstlogger PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
    VERSION ${PROJECT_VERSION}
)

set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
find_library(CMPS14_LIBRARIES NAMES cmps14)
target_link_libraries(bbstlogger PRIVATE ${OpenCV_LIBS} ${WIRINGPI_LIBRARIES} ${CMPS14_LIBRARIES})