cmake_minimum_required (VERSION 2.6)
project (sloop)
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE)

# find opengl, glut
if( WIN32 )
    find_package(OpenGL REQUIRED)
    set(GLUT_FOUND 1)
    set(GLUT_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/3rdparty/freeglut/include")
    set(GLUT_LIBRARY_DIR "${CMAKE_SOURCE_DIR}/3rdparty/freeglut/lib")
    set(GLUT_LIBRARIES "freeglut.lib")
else()
    find_package(OpenGL REQUIRED)
    find_package(GLUT REQUIRED)
endif()

# add 3rdparty libraries
add_subdirectory(3rdparty)

# add sloop
message(STATUS "adding sloop...")
file(GLOB sloop_SRC "src/*.cpp" "include/*.h")
include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories(${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})
include_directories("${CMAKE_SOURCE_DIR}/3rdparty/gear/include")
include_directories("${CMAKE_SOURCE_DIR}/3rdparty/quadprog")
link_directories(${OPENGL_LIBRARY_DIR} ${GLUT_LIBRARY_DIR})
add_executable(sloop ${sloop_SRC})
target_link_libraries(sloop gear quadprog ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})

message(STATUS "sloop added.")

