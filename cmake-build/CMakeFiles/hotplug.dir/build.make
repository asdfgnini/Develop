# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zjt/vscode/Develop

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zjt/vscode/Develop/cmake-build

# Include any dependencies generated for this target.
include CMakeFiles/hotplug.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/hotplug.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/hotplug.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/hotplug.dir/flags.make

CMakeFiles/hotplug.dir/main.cpp.o: CMakeFiles/hotplug.dir/flags.make
CMakeFiles/hotplug.dir/main.cpp.o: ../main.cpp
CMakeFiles/hotplug.dir/main.cpp.o: CMakeFiles/hotplug.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zjt/vscode/Develop/cmake-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/hotplug.dir/main.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/hotplug.dir/main.cpp.o -MF CMakeFiles/hotplug.dir/main.cpp.o.d -o CMakeFiles/hotplug.dir/main.cpp.o -c /home/zjt/vscode/Develop/main.cpp

CMakeFiles/hotplug.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hotplug.dir/main.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zjt/vscode/Develop/main.cpp > CMakeFiles/hotplug.dir/main.cpp.i

CMakeFiles/hotplug.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hotplug.dir/main.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zjt/vscode/Develop/main.cpp -o CMakeFiles/hotplug.dir/main.cpp.s

CMakeFiles/hotplug.dir/HotPlug.cpp.o: CMakeFiles/hotplug.dir/flags.make
CMakeFiles/hotplug.dir/HotPlug.cpp.o: ../HotPlug.cpp
CMakeFiles/hotplug.dir/HotPlug.cpp.o: CMakeFiles/hotplug.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zjt/vscode/Develop/cmake-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/hotplug.dir/HotPlug.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/hotplug.dir/HotPlug.cpp.o -MF CMakeFiles/hotplug.dir/HotPlug.cpp.o.d -o CMakeFiles/hotplug.dir/HotPlug.cpp.o -c /home/zjt/vscode/Develop/HotPlug.cpp

CMakeFiles/hotplug.dir/HotPlug.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hotplug.dir/HotPlug.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zjt/vscode/Develop/HotPlug.cpp > CMakeFiles/hotplug.dir/HotPlug.cpp.i

CMakeFiles/hotplug.dir/HotPlug.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hotplug.dir/HotPlug.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zjt/vscode/Develop/HotPlug.cpp -o CMakeFiles/hotplug.dir/HotPlug.cpp.s

# Object files for target hotplug
hotplug_OBJECTS = \
"CMakeFiles/hotplug.dir/main.cpp.o" \
"CMakeFiles/hotplug.dir/HotPlug.cpp.o"

# External object files for target hotplug
hotplug_EXTERNAL_OBJECTS =

hotplug: CMakeFiles/hotplug.dir/main.cpp.o
hotplug: CMakeFiles/hotplug.dir/HotPlug.cpp.o
hotplug: CMakeFiles/hotplug.dir/build.make
hotplug: CMakeFiles/hotplug.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zjt/vscode/Develop/cmake-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable hotplug"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/hotplug.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/hotplug.dir/build: hotplug
.PHONY : CMakeFiles/hotplug.dir/build

CMakeFiles/hotplug.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/hotplug.dir/cmake_clean.cmake
.PHONY : CMakeFiles/hotplug.dir/clean

CMakeFiles/hotplug.dir/depend:
	cd /home/zjt/vscode/Develop/cmake-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zjt/vscode/Develop /home/zjt/vscode/Develop /home/zjt/vscode/Develop/cmake-build /home/zjt/vscode/Develop/cmake-build /home/zjt/vscode/Develop/cmake-build/CMakeFiles/hotplug.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/hotplug.dir/depend

