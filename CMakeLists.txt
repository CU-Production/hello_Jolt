cmake_minimum_required(VERSION 3.15)
project(NVRHI_Example)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(3rd_party)

set(CMAKE_VS_PLATFORM_TOOLSET_HOST_ARCHITECTURE "x64")

# Set runtime library
if (USE_STATIC_MSVC_RUNTIME_LIBRARY)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

if (GENERATE_DEBUG_SYMBOLS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zi")
endif()

if (NOT CPP_RTTI_ENABLED)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /GR-")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /GR")
endif()

if (NOT CPP_EXCEPTIONS_ENABLED)
    string(REPLACE "/EHsc" "" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4577")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
endif()

add_executable(HelloJoltCli helloJoltCli.cpp)
target_link_libraries(HelloJoltCli PRIVATE Jolt::Jolt)

add_executable(helloSokolShapes helloSokolShapes.c)
target_link_libraries(helloSokolShapes PRIVATE sokol hmm)
