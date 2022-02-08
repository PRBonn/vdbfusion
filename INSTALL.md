# Installing VDBFusion

Most of the people can do `pip install vdbfusion` and get the Python bindings for this project. If you wish to build the project from source because you are doing modifications you have few options available.

## Build from source in Linux

I spent some extra time trying to make the build work on different Linux distributions like Ubuntu, Debian, CentOs, etc. The build (or superbuild) includes all the necessary tools to pull 3rdparty dependencies. If you are developing something on top of VDBFusion I would recommend to check [Using system installed 3rdparty libraries](#using-system-installed-3rdparty-libraries) instead. More information about building can be found on the [CI/CD configuration](./.gitlab-ci.yml)

### Minimal setup

To build the project from source you only need:

- A C++ compiler
- git
- Python3

If you are using Ubuntu this command should be enough to get you started:

```sh
sudo apt-get update && sudo apt-get install build-essential cmake git python3 python3-dev python3-pip
```

After that you can run `make install` (at the root of this project) to install the python bindings on your system. If you wish to also install the C++ bindings, this is not yet supported on the superbuild. You will first need to install all the 3rdparty dependencies yourself. Check [Using system installed 3rdparty libraries](#using-system-installed-3rdparty-libraries) and then [Installing the C++ API](#installing-the-c-api).

### Using system installed 3rdparty libraries

Assuming you have installed all 3rdparty dependencies (OpenVDB, Eigen3, pybind11). Then you can build the project the same way as before without any extra effort. You can see this [Dockerfile](docker/builder/Dockerfile) to understand how I do it on the CI/CD.

### Installing the C++ API

This is still under development and only supported for dev builds (all 3rdparty dependencies installed locally).

```sh
mkdir -p build && cd build && cmake ..
sudo make install
```

This will install the C++ API on your system that can be found on any other CMake consumer project using the `find_package` command:

```cmake
find_package(VDBFusion REQUIRED)
add_executable(my_example my_example.cpp)
target_link_libraries(my_examples PRIVATE VDBFusion::vdbfusion)
```

If you want to know more about this you can also check the [C++ example](examples/cpp/CMakeLists.txt)

### Build Configuration

Most of the variables you can control are on the root [CMakeLists.txt](CMakeLists.txt). Make sure you check them out if something on your build is not properly working.

## Build from source in Windows/macOS

I don't use Windows or macOS, so initially I'm not providing support for these systems. Although it should be straightforward to build the project there since the only real dependencies are a `C++` compiler and a `Python3` interpreter. You can check the [minimal docker](docker/pip/Dockerfile) setup needed to build VDBFusion and port it to your system.
