cmake_minimum_required(VERSION 3.2)
include(GNUInstallDirs)
project (tpiklib)
#include_directories(src)
#include_directories(include)
set (CMAKE_CXX_STANDARD 11)
add_library(tpiklib SHARED
	src/Task.cpp
	src/PriorityLevel.cpp
	src/TPIK.cpp
	src/ActionManager.cpp
	src/Solver.cpp	
	src/Action.cpp
	src/iCAT.cpp
)
target_link_libraries(tpiklib rml)
set_target_properties(tpiklib PROPERTIES LINKER_LANGUAGE CXX)
# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(tpiklib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

install(TARGETS tpiklib EXPORT tpiklibConfig
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/tpiklib)
# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT tpiklibConfig DESTINATION share/tpiklib/cmake)
# This makes the project importable from the build directory
export(TARGETS tpiklib FILE tpiklibConfig.cmake)



#add_executable(main src/thirdLevel.cpp RobotLibrary)
##include_directories(main Include )
##target_link_libraries(main  )
#set_target_properties(TPIKlib PROPERTIES PUBLIC_HEADER "*.h")
#include_directories(TPIKlib src)
#include_directories(TPIKlib include)
#install(TARGETS TPIKlib
#    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/src)

#install(DIRECTORY src
#    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
#	FILES_MATCHING PATTERN "*.h")
