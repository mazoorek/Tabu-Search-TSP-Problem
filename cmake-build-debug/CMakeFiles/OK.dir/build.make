# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.12

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\JetBrains\CLion 2018.2.5\bin\cmake\win\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\JetBrains\CLion 2018.2.5\bin\cmake\win\bin\cmake.exe" -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\git\OK

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\git\OK\cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/OK.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/OK.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/OK.dir/flags.make

CMakeFiles/OK.dir/main.cpp.obj: CMakeFiles/OK.dir/flags.make
CMakeFiles/OK.dir/main.cpp.obj: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\git\OK\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/OK.dir/main.cpp.obj"
	C:\PROGRA~1\MINGW-~1\X86_64~1.0-P\mingw64\bin\G__~1.EXE  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\OK.dir\main.cpp.obj -c C:\git\OK\main.cpp

CMakeFiles/OK.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OK.dir/main.cpp.i"
	C:\PROGRA~1\MINGW-~1\X86_64~1.0-P\mingw64\bin\G__~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\git\OK\main.cpp > CMakeFiles\OK.dir\main.cpp.i

CMakeFiles/OK.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OK.dir/main.cpp.s"
	C:\PROGRA~1\MINGW-~1\X86_64~1.0-P\mingw64\bin\G__~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\git\OK\main.cpp -o CMakeFiles\OK.dir\main.cpp.s

# Object files for target OK
OK_OBJECTS = \
"CMakeFiles/OK.dir/main.cpp.obj"

# External object files for target OK
OK_EXTERNAL_OBJECTS =

OK.exe: CMakeFiles/OK.dir/main.cpp.obj
OK.exe: CMakeFiles/OK.dir/build.make
OK.exe: CMakeFiles/OK.dir/linklibs.rsp
OK.exe: CMakeFiles/OK.dir/objects1.rsp
OK.exe: CMakeFiles/OK.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\git\OK\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable OK.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\OK.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/OK.dir/build: OK.exe

.PHONY : CMakeFiles/OK.dir/build

CMakeFiles/OK.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\OK.dir\cmake_clean.cmake
.PHONY : CMakeFiles/OK.dir/clean

CMakeFiles/OK.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\git\OK C:\git\OK C:\git\OK\cmake-build-debug C:\git\OK\cmake-build-debug C:\git\OK\cmake-build-debug\CMakeFiles\OK.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/OK.dir/depend

