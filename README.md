# Fourier plotting with raylib CMake Project

The code in here is motivated from an article written by Markus Buchholz, published on medium.

His code can be found on [Github](https://github.com/markusbuchholz/Fourier-Series-in-Cpp-ImGui)

His description can be found on [Medium](https://markus-x-buchholz.medium.com/fourier-series-in-c-imgui-1587e866586e)

The cmake file is based on the example cmake file from the RayLib project.

This provides a base project template which builds with [CMake](https://cmake.org).

## Usage

To compile the example, use one of the following dependending on your build target...

### Desktop

Use the following to build for desktop:

``` bash
cmake -B build
cmake --build build
```
Or if Ninja build is preferred:

```bash
cmake -B build -G Ninja
```

### Web

Compiling for the web requires the [Emscripten SDK](https://emscripten.org/docs/getting_started/downloads.html):

``` bash
mkdir build
cd build
emcmake cmake .. -DPLATFORM=Web -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXE_LINKER_FLAGS="-s USE_GLFW=3" -DCMAKE_EXECUTABLE_SUFFIX=".html"
emmake make
```
