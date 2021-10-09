# Real-time Skeletonization for Sketch-based Modeling (SMI:2021)
<img src="data/banner_thumb.png" alt="ModelingCat" width="250"/>

## Demo
We provide an executable software under directory "demo_exe/".

<img src="data/ModelingCat.gif" alt="ModelingCat" width="500"/>

## Tested Environment
1. Ubuntu20.04 LTS, gcc-7 compiler
2. Windows10, Visual Studio Community 2017 x86_amd64 compiler

## Dependencies
Qt, Libigl(Tetgen, Triangle, CGAL, Eigen), OpenCV. 
Please install CMake, Qt and OpenCV first, and modify the CMakeLists.txt according to your library's install path.

## Compile & Run

### For Linux Users, type in the following commands:
```
mkdir build
cd build
cmake ..
make -j8
./main
```
### For Windows Users
Make sure you install Visual Studio 2017 first.

It's highly recommended using 'visual studio code' IDE and "cmake tools from 'VS code' market place" to compile the code. Then:
1. Select a compiler:  "Ctrl+Shift+P", type in CMake: Select a Kit, select Visual Studio Community 2017 x86_amd64 as your compiler
2. CMake Configuration: "Ctrl+Shift+P", select "CMake Configure". The configuration process will automatically download all dependencies of libigl. 
3. Build and run the program: If the configuration successfully finishes, you will find .sln under directory 'build/'，
open .sln with Visual Studio 2017 and compile.  Ctrl+F5 to run the program, if error occurs, copy all "*.dll" and directory "platforms/" from demo_exe to your "build/Release/" directory.

## Other Issues
### Adapt to CGAL
goto external\libigl\external\cgal\Surface_mesh\include\CGAL\Surface_mesh\Surface_mesh.h, add the following code in ```Class SM_Index```
```cpp
size_type idx()const{return idx_;}
```