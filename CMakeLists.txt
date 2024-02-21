# TPIK: Task Priority Inverse Kinematics

cmake_minimum_required(VERSION 3.2)
project (tpik VERSION 1.0 LANGUAGES CXX)

include(GNUInstallDirs)

# Default to C++17
#if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 17)
#endif()

#if(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
#endif()

option(BUILD_TESTS "Compile tests" OFF)

# Add files in the tree of "Projects" view in QtCreator
file(GLOB_RECURSE SOURCE_FILES
    "include/*.h*"
)
add_custom_target(.Files. SOURCES ${SOURCE_FILES})

add_library(tpik SHARED
        src/Task.cpp
	src/PriorityLevel.cpp
	src/TPIK.cpp
	src/ActionManager.cpp
	src/Solver.cpp	
	src/Action.cpp
	src/iCAT.cpp
	src/CoordinationArmVehicleSolver.cpp
        src/NonReactiveTask.cpp
        src/ReactiveTask.cpp
)
target_link_libraries(tpik rml config++)
set_target_properties(tpik PROPERTIES LINKER_LANGUAGE CXX)
# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(tpik PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

install(TARGETS tpik EXPORT tpikConfig
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/tpik DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
#install(DIRECTORY include/tpik_internal DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT tpikConfig DESTINATION share/tpik/cmake)
# This makes the project importable from the build directory
export(TARGETS tpik FILE tpikConfig.cmake)

#if(BUILD_TESTS)
#    # Every library has unit tests, of course
#   add_executable(tpik_test
#   test/tpikTest.cpp
#   test/TestTask.cpp
#)
#target_link_libraries(tpik_test tpik)
    
#add_test(tpik_test tpik_test)
#endif(BUILD_TESTS)



