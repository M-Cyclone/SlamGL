# SlamGL
A modified ORB-SLAM3 project. The rendering module is changed to use OpenGL(GLFW + GLAD) directly.  

You can run test_ui to run with glfw windows.

# Compile
## Ubuntu 20.04
You can configure this project with cmake. The minimum cmake version should be 3.16, which is the default version on ubuntu 20.04.

The following commands can be used to generate executable binary file.
```
cmake -B build
cmake --build build --parallel 4
```

## Win10
This project only tested on Visual Studio 2019 (Windows 10 SDK 10.0.19041 and MSVC v142) on Win10 platform.  

You can use the following commands to generate Visual Studio's sln files and compile all sources in Visual Studio.
```
cmake -B build -DOpenCV_DIR="(Your MSVC version OpenCV directory which has cmake config files.)"
```

# Dependences
This project depends on ORB-SLAM3 and several OpenGL libraries.

## ORB-SLAM3
You can get the dependencies of ORB-SLAM3 on <https://github.com/UZ-SLAMLab/ORB_SLAM3>. Pangolin because it is not used in this project. The class Viewer/MapViewer/KeyFrameViewer is removed because of the same reason.

## Render
The following libraries are used to render with OpenGL directly.

### GLFW
GLFW is a library used to handle the window on different OS, which means it will be easy to move the codes between OSs. The repository is here <https://github.com/glfw/glfw>. My project use cmake to compile GLFW from source code, but you can just use the precomiled binary files.

### GLAD
GLAD is used to handle OpenGL functions. You can get suitable files on <https://glad.dav1d.de/>.

### Dear ImGui
ImGui is a UI library and it's really easy to use. The repository is here <https://github.com/ocornut/imgui>. In this project only part of the files are used to create ImGui's shared library object. You can change the CMakeLists.txt to use different hardware interfaces.

### GLM
GLM is a linear algebra library witch is very suitable for OpenGL and Vulkan. GLM is used for easier shader data handling (Eigen is not that easy to deal with OpenGL). You can find GLM's source on <https://www.opengl.org/sdk/libs/GLM/>.

## Others

### stb
stb has a lot of header-only files to handled with images and fonts. You can find the stb' header files on <https://github.com/nothings/stb>. This project uses stb_image and stb_image_write. A stb_impl.cpp is also created to make sure the .so file can be compiled.

### spdlog
spdlog is a fast c++ logging library. You can find the source on <https://github.com/gabime/spdlog>. This project makes several macro with spdlog for different levels of log output.
