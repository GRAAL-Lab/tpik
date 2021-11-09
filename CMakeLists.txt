cmake_minimum_required(VERSION 3.2)
include(GNUInstallDirs)

option(BUILD_TESTS "Compile tests" ON)
project (tpik)

#include_directories(src)
#include_directories(include)
set (CMAKE_CXX_STANDARD 14)
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



