# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.17.0_1/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.17.0_1/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/clyako/Desktop/sai2/apps/zoom-chef

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/clyako/Desktop/sai2/apps/zoom-chef/build

# Include any dependencies generated for this target.
include project_starter_code/CMakeFiles/controller_project.dir/depend.make

# Include the progress variables for this target.
include project_starter_code/CMakeFiles/controller_project.dir/progress.make

# Include the compile flags for this target's objects.
include project_starter_code/CMakeFiles/controller_project.dir/flags.make

project_starter_code/CMakeFiles/controller_project.dir/controller.cpp.o: project_starter_code/CMakeFiles/controller_project.dir/flags.make
project_starter_code/CMakeFiles/controller_project.dir/controller.cpp.o: ../project_starter_code/controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/clyako/Desktop/sai2/apps/zoom-chef/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object project_starter_code/CMakeFiles/controller_project.dir/controller.cpp.o"
	cd /Users/clyako/Desktop/sai2/apps/zoom-chef/build/project_starter_code && /Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/controller_project.dir/controller.cpp.o -c /Users/clyako/Desktop/sai2/apps/zoom-chef/project_starter_code/controller.cpp

project_starter_code/CMakeFiles/controller_project.dir/controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/controller_project.dir/controller.cpp.i"
	cd /Users/clyako/Desktop/sai2/apps/zoom-chef/build/project_starter_code && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/clyako/Desktop/sai2/apps/zoom-chef/project_starter_code/controller.cpp > CMakeFiles/controller_project.dir/controller.cpp.i

project_starter_code/CMakeFiles/controller_project.dir/controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/controller_project.dir/controller.cpp.s"
	cd /Users/clyako/Desktop/sai2/apps/zoom-chef/build/project_starter_code && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/clyako/Desktop/sai2/apps/zoom-chef/project_starter_code/controller.cpp -o CMakeFiles/controller_project.dir/controller.cpp.s

# Object files for target controller_project
controller_project_OBJECTS = \
"CMakeFiles/controller_project.dir/controller.cpp.o"

# External object files for target controller_project
controller_project_EXTERNAL_OBJECTS =

../bin/project_starter_code/controller_project: project_starter_code/CMakeFiles/controller_project.dir/controller.cpp.o
../bin/project_starter_code/controller_project: project_starter_code/CMakeFiles/controller_project.dir/build.make
../bin/project_starter_code/controller_project: /Users/clyako/Desktop/sai2/core/sai2-common/build/libsai2-common.a
../bin/project_starter_code/controller_project: /Users/clyako/Desktop/sai2/core/chai3d/build/libchai3d.a
../bin/project_starter_code/controller_project: /Users/clyako/Desktop/sai2/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/project_starter_code/controller_project: /usr/local/lib/libtinyxml2.dylib
../bin/project_starter_code/controller_project: /Users/clyako/Desktop/sai2/core/sai2-simulation/build/libsai2-simulation.a
../bin/project_starter_code/controller_project: /Users/clyako/Desktop/sai2/core/sai2-model/build/libsai2-model.a
../bin/project_starter_code/controller_project: /Users/clyako/Desktop/sai2/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/project_starter_code/controller_project: /usr/local/lib/libtinyxml2.dylib
../bin/project_starter_code/controller_project: /Users/clyako/Desktop/sai2/core/sai2-model/rbdl/build/librbdl.dylib
../bin/project_starter_code/controller_project: /Users/clyako/Desktop/sai2/core/sai2-graphics/build/libsai2-graphics.a
../bin/project_starter_code/controller_project: /Users/clyako/Desktop/sai2/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/project_starter_code/controller_project: /usr/local/lib/libtinyxml2.dylib
../bin/project_starter_code/controller_project: /Users/clyako/Desktop/sai2/core/chai3d/build/libchai3d.a
../bin/project_starter_code/controller_project: /usr/local/lib/libjsoncpp.dylib
../bin/project_starter_code/controller_project: /usr/local/lib/libhiredis.dylib
../bin/project_starter_code/controller_project: /usr/local/lib/libglfw.dylib
../bin/project_starter_code/controller_project: /Users/clyako/Desktop/sai2/core/sai2-primitives/build/libsai2-primitives.a
../bin/project_starter_code/controller_project: /Users/clyako/Desktop/sai2/core/sai2-primitives/../external/ReflexxesTypeII/MacOS/x64/release/lib/libReflexxesTypeII.a
../bin/project_starter_code/controller_project: /Users/clyako/Desktop/sai2/core/sai2-common/build/libsai2-common.a
../bin/project_starter_code/controller_project: /Users/clyako/Desktop/sai2/core/chai3d/build/libchai3d.a
../bin/project_starter_code/controller_project: /Users/clyako/Desktop/sai2/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/project_starter_code/controller_project: /usr/local/lib/libtinyxml2.dylib
../bin/project_starter_code/controller_project: /Users/clyako/Desktop/sai2/core/sai2-simulation/build/libsai2-simulation.a
../bin/project_starter_code/controller_project: /Users/clyako/Desktop/sai2/core/sai2-model/build/libsai2-model.a
../bin/project_starter_code/controller_project: /Users/clyako/Desktop/sai2/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/project_starter_code/controller_project: /usr/local/lib/libtinyxml2.dylib
../bin/project_starter_code/controller_project: /Users/clyako/Desktop/sai2/core/sai2-model/rbdl/build/librbdl.dylib
../bin/project_starter_code/controller_project: /Users/clyako/Desktop/sai2/core/sai2-graphics/build/libsai2-graphics.a
../bin/project_starter_code/controller_project: /Users/clyako/Desktop/sai2/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/project_starter_code/controller_project: /usr/local/lib/libtinyxml2.dylib
../bin/project_starter_code/controller_project: /Users/clyako/Desktop/sai2/core/chai3d/build/libchai3d.a
../bin/project_starter_code/controller_project: /usr/local/lib/libjsoncpp.dylib
../bin/project_starter_code/controller_project: /usr/local/lib/libhiredis.dylib
../bin/project_starter_code/controller_project: /usr/local/lib/libglfw.dylib
../bin/project_starter_code/controller_project: /Users/clyako/Desktop/sai2/core/sai2-primitives/../external/ReflexxesTypeII/MacOS/x64/release/lib/libReflexxesTypeII.a
../bin/project_starter_code/controller_project: project_starter_code/CMakeFiles/controller_project.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/clyako/Desktop/sai2/apps/zoom-chef/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/project_starter_code/controller_project"
	cd /Users/clyako/Desktop/sai2/apps/zoom-chef/build/project_starter_code && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/controller_project.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
project_starter_code/CMakeFiles/controller_project.dir/build: ../bin/project_starter_code/controller_project

.PHONY : project_starter_code/CMakeFiles/controller_project.dir/build

project_starter_code/CMakeFiles/controller_project.dir/clean:
	cd /Users/clyako/Desktop/sai2/apps/zoom-chef/build/project_starter_code && $(CMAKE_COMMAND) -P CMakeFiles/controller_project.dir/cmake_clean.cmake
.PHONY : project_starter_code/CMakeFiles/controller_project.dir/clean

project_starter_code/CMakeFiles/controller_project.dir/depend:
	cd /Users/clyako/Desktop/sai2/apps/zoom-chef/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/clyako/Desktop/sai2/apps/zoom-chef /Users/clyako/Desktop/sai2/apps/zoom-chef/project_starter_code /Users/clyako/Desktop/sai2/apps/zoom-chef/build /Users/clyako/Desktop/sai2/apps/zoom-chef/build/project_starter_code /Users/clyako/Desktop/sai2/apps/zoom-chef/build/project_starter_code/CMakeFiles/controller_project.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : project_starter_code/CMakeFiles/controller_project.dir/depend

