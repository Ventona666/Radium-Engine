\page basicsCompileRadium Radium Compilation instructions
[TOC]

# Supported compiler and platforms

The following platforms and tool chains have been tested and should work :

 * *Windows* : IDEs: Visual Studio 2019 (2017 is not supported due to embedded cmake version), QtCreator. Command Line: cmake+ninja+MSVC(2017 or 2019) .
 * *Mac OSX* : gcc 10 or higher, Apple clang
 * *Linux* : gcc 8  or higher, clang

See also our Continuous Integration system at https://github.com/STORM-IRIT/Radium-Engine/actions.

Minimal requirements
* OpenGL 4.1+ / GLSL 410+
* CMake 3.15.7+
* Qt5 (minimal version 5.14) or Qt6 (experimental)

# Build instructions
If not already done, follow instructions at \ref dependenciesmanagement.

Radium follows a standard cmake structure, so any IDE supporting cmake should be able to configure and build it.

\note We strongly recommend to have dedicated build and install directories for each build type (Release, Debug).
Remember that compiling Radium in Debug mode needs to have the dependencies compiled and installed in Debug mode
(due to a limitation in assimp).

## Folder structure
Radium-Engine relies on CMake build-chain on all supported platforms.
In most cases, building should be pretty straightforward, provided that cmake can locate the dependencies.

### Installation directory
By default, `${CMAKE_INSTALL_PREFIX}` is set as follow:

- For release build :

~~~{.cmake}
    set(RADIUM_BUNDLE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Bundle-${CMAKE_CXX_COMPILER_ID})
~~~

- For Debug or RelWithDebInfo build

~~~{.cmake}
    set(RADIUM_BUNDLE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Bundle-${CMAKE_CXX_COMPILER_ID}-${CMAKE_BUILD_TYPE})
~~~

It has the following structure, if externals are compiled along Radium
~~~
Bundle-*
 - bin/  include/  lib/  libdata/  LICENSE  README.md  Resources/  share/
~~~
Or if externals are compiled locally :
~~~
Bundle-*
 - bin/  include/  lib/  LICENSE  README.md  Resources/  share/
~~~

### Configure build options

Radium offers the following build options:
~~~bash
// Enable coverage, gcc only. Experimental, need ENABLE_TESTING
RADIUM_ENABLE_COVERAGE:BOOL=OFF
--
// Enable testing. Tests are automatically built with target all
RADIUM_ENABLE_TESTING:BOOL=ON
--
// Include Radium::Core in CMake project
RADIUM_GENERATE_LIB_CORE:BOOL=ON
--
// Include Radium::Engine in CMake project
RADIUM_GENERATE_LIB_ENGINE:BOOL=ON
--
// Include Radium::Gui in CMake project
RADIUM_GENERATE_LIB_GUI:BOOL=ON
--
// Include Radium::IO in CMake project
RADIUM_GENERATE_LIB_IO:BOOL=ON
--
// Include Radium::PluginBase in CMake project
RADIUM_GENERATE_LIB_PLUGINBASE:BOOL=ON
--
// Check submodules during build (will be automatically disabled after run)
RADIUM_GIT_UPDATE_SUBMODULE:BOOL=ON
--
// Install documentation. If RadiumDoc is compiled, install documentation to bundle directory for install target
RADIUM_INSTALL_DOC:BOOL=ON
--
// Value of CMAKE_INSTALL_MESSAGE for dependencies. See documentations of CMAKE_INSTALL_MESSAGE for possible values
RADIUM_EXTERNAL_CMAKE_INSTALL_MESSAGE=NEVER
--
// Disable Radium Log messages
RADIUM_QUIET:BOOL=OFF
--
// Provide loaders based on Assimp library
RADIUM_IO_ASSIMP:BOOL=ON
--
// Provide depricated loaders (to be removed without notice)
RADIUM_IO_DEPRECATED:BOOL=ON
--
// Provide loaders based on TinyPly library
RADIUM_IO_TINYPLY:BOOL=ON
--
// [addExternalFolder] Skip updating Core::external (disable for rebuild)
RADIUM_SKIP_CORE_EXTERNAL:BOOL=ON
--
// [addExternalFolder] Skip updating Engine::external (disable for rebuild)
RADIUM_SKIP_ENGINE_EXTERNAL:BOOL=ON
--
// [addExternalFolder] Skip updating IO::external (disable for rebuild)
RADIUM_SKIP_IO_EXTERNAL:BOOL=ON
--
// Update version file each time the project is compiled (update compilation time in version.cpp)
RADIUM_UPDATE_VERSION:BOOL=ON
--
// Use double precision for Scalar
RADIUM_USE_DOUBLE:BOOL=OFF
~~~

All radium related cmake options (with their current values) can be printed with `cmake -LAH | grep -B1 RADIUM` (on linux like system)

\warning For computers with low RAM capacities (under 12G) we recommend to set the `CMAKE_BUILD_PARALLEL_LEVEL` environment variable to a reasonable value (i.e. 2) to prevent the computer from swapping.


### Dependencies between libraries

The options `RADIUM_GENERATE_LIB_XXXX` allows to enable/disable each Radium library.
The dependencies between libraries are set as follow:

~~~{.cmake}
add_dependencies (${ra_engine_target} PUBLIC Radium::Core)
add_dependencies (${ra_io_target} PUBLIC Radium::Core)
add_dependencies (${ra_pluginbase_target} Radium::Core Radium::Engine)
add_dependencies (${ra_gui_target} PUBLIC Radium::Core Radium::Engine Radium::PluginBase Radium::IO)
~~~

\warning Consistency of `RADIUM_GENERATE_LIB_***` options is not checked wrt. the dependencies.

- When enabled using `RADIUM_GENERATE_LIB_***`, each library has a compilation target: `Core`, `Engine`, ...



## Command line instructions for building (on windows, mac and linux)

Out-of source builds are mandatory, we recommend to follow the usual sequence:

~~~bash
$ mkdir build
$ cd build
$ cmake .. -DQt5_DIR=........ -DCMAKE_BUILD_TYPE=.......
$ make
$ make install
~~~
\note Qt6 support is experimental. To enable it, replace `-DQt5_DIR=path/to/qt5` by `-DQt6_DIR=path/to/qt6`.

\note Running the `install` target is recommended as it will copy all the radium related library in the same place,
generate the cmake packages and bundle applications with their dependencies (on macos and windows).

Note that installation requires write access on the installation directory.

## Integration with Visual Studio (Microsoft Windows)

### Supported versions of MSVC
Radium requires MSVC 2019 or superior, as it relies on:
* C++11/C++14/C++17 features such as `constexpr`,
* cmake built-in support

Our Continuous Integration systems uses Microsoft Compiler 2017, in combination with cmake and ninja.
Using Visual Studio 2017 with cmake support is however not possible: VS is shipped with cmake: 3.12, while Radium requires cmake 3.13 at least. We recommend to use Visual Studio 2019 in that case.
Qt 5.15+ is distributed with binaries precompiled with MSVC 2019, but Qt binaries precompiled with MSVC2017 does not break the build.

### Qt Dependency

Use precompiled libraries for VS 2017 or 2019 - 64 bits (minimal version required: 5.14).
You will probably have to manually point cmake to the Qt folder.

### Getting started with Visual Studio

Thanks to the integrated support of CMake in Visual Studio, you don't need a VS solution to build your project: open the Radium folder (via *File* > *Open* > *Folder ...* or `devenv.exe <foldername>`).
VS should run cmake, generate the target builds (Debug and Release by default).
Other build types can be added by editing `CMakeSettings.json`.

You may have Cmake errors occurring at the first run.
To fix them, you need to edit the VS-specific file `CMakeSettings.json`, via *CMake* > *Change CMake Settings* > path-to-CMakeLists (configuration-name) from the main menu.
For instance, it usually requires to set cmake build types manually, and to give path to Qt libraries.
To fix it, edit `CMakeSettings.json`, such that
~~~json
{
  "configurations": [
    {
      "name": "x64-Release",
      "generator": "Ninja",
      "configurationType": "Release",
      "inheritEnvironments": [ "msvc_x64_x64" ],
      "buildRoot": "C:/Users/XXX/Dev/builds/Radium/${name}",
      "installRoot": "C:/Users/XXX/Dev/Radium-install",
      "cmakeCommandArgs": "-DCMAKE_PREFIX_PATH=C:/Qt-5.15/5.15.0/msvc2017_64",
      "buildCommandArgs": "",
      "ctestCommandArgs": ""
    },
    {
      "name": "x64-Debug",
      "generator": "Ninja",
      "configurationType": "Debug",
      "inheritEnvironments": [ "msvc_x64_x64" ],
      "buildRoot": "C:/Users/XXX/Dev/builds/Radium/${name}",
      "installRoot": "C:/Users/XXX/Dev/Radium-installdbg",
      "cmakeCommandArgs": "-DCMAKE_PREFIX_PATH=C:/Qt-5.15/5.15.0/msvc2017_64",
      "buildCommandArgs": "",
      "ctestCommandArgs": ""
    }
  ]
}
~~~
\note It is strongly encouraged to use `/` separators in your path, instead of `\\`. See https://stackoverflow.com/questions/13737370/cmake-error-invalid-escape-sequence-u
*Note*: When compiling the dependencies you may hit the max path length fixed by Microsoft OS (260 characters). To fix this, you might need to change the path of your build dir to shorten it, or to change the limit on your system, see: https://docs.microsoft.com/en-us/windows/win32/fileio/naming-a-file#enable-long-paths-in-windows-10-version-1607-and-later

### Compilation

Right click on `CMakeList.txt > build > all`.
To install, you need to run any installation target, e.g. `Engine.dll (install)` or to select the menu `<Build>/<Install radiumproject>`
